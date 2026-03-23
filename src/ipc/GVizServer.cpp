#include "GVizServer.h"
#include "../gui/panels/LogPanel.h"

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <sstream>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace gtsam_viz {

using namespace gviz_ipc;

GVizServer::GVizServer()  = default;
GVizServer::~GVizServer() { stop(); }

// ── Start / Stop ──────────────────────────────────────────────────────────────

bool GVizServer::start() {
    if (running_) return true;

    // Remove stale socket file
    ::unlink(SOCKET_PATH);

    serverFd_ = ::socket(AF_UNIX, SOCK_STREAM, 0);
    if (serverFd_ < 0) {
        GVLOG_ERR("[GVizServer] socket() failed: " + std::string(strerror(errno)));
        return false;
    }

    struct sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    std::strncpy(addr.sun_path, SOCKET_PATH, sizeof(addr.sun_path)-1);

    if (::bind(serverFd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        GVLOG_ERR("[GVizServer] bind() failed: " + std::string(strerror(errno)));
        ::close(serverFd_); serverFd_ = -1;
        return false;
    }

    if (::listen(serverFd_, 1) < 0) {
        GVLOG_ERR("[GVizServer] listen() failed: " + std::string(strerror(errno)));
        ::close(serverFd_); serverFd_ = -1;
        return false;
    }

    running_ = true;
    acceptThread_ = std::thread(&GVizServer::acceptLoop, this);

    GVLOG_INFO("[GVizServer] Listening on " + std::string(SOCKET_PATH));
    return true;
}

void GVizServer::stop() {
    running_ = false;
    if (serverFd_ >= 0) { ::shutdown(serverFd_, SHUT_RDWR); ::close(serverFd_); serverFd_ = -1; }
    if (acceptThread_.joinable()) acceptThread_.join();
    ::unlink(SOCKET_PATH);
    clientConnected_ = false;
}

// ── Accept loop (background thread) ──────────────────────────────────────────

void GVizServer::acceptLoop() {
    while (running_) {
        // Use select with timeout so we can check running_ periodically
        fd_set fds; FD_ZERO(&fds); FD_SET(serverFd_, &fds);
        struct timeval tv{1, 0};
        int ret = ::select(serverFd_+1, &fds, nullptr, nullptr, &tv);
        if (ret <= 0) continue;

        int clientFd = ::accept(serverFd_, nullptr, nullptr);
        if (clientFd < 0) continue;

        clientConnected_ = true;
        GVLOG_INFO("[GVizServer] Client connected.");

        recvLoop(clientFd);

        ::close(clientFd);
        clientConnected_ = false;
        GVLOG_INFO("[GVizServer] Client disconnected.");

        // Clear state on disconnect
        std::lock_guard<std::mutex> g(mtx_);
        pending_.reset();
        hasNew_ = false;
    }
}

// ── Recv loop (runs per connected client) ─────────────────────────────────────

bool GVizServer::recvExact(int fd, void* buf, size_t n) {
    size_t got = 0;
    while (got < n) {
        ssize_t r = ::recv(fd, (char*)buf + got, n - got, 0);
        if (r <= 0) return false;
        got += (size_t)r;
    }
    return true;
}

void GVizServer::recvLoop(int clientFd) {
    while (running_ && clientConnected_) {
        // Read header
        GVizMsgHeader hdr{};
        if (!recvExact(clientFd, &hdr, sizeof(hdr))) return;

        if (hdr.magic != MAGIC) {
            GVLOG_WARN("[GVizServer] Bad magic, dropping connection.");
            return;
        }

        GVizFrame frame;
        frame.type   = static_cast<MsgType>(hdr.type);
        frame.seq_id = hdr.seq_id;

        // Label
        if (hdr.label_len > 0) {
            frame.label.resize(hdr.label_len);
            if (!recvExact(clientFd, frame.label.data(), hdr.label_len)) return;
        }

        // Variables
        frame.vars.resize(hdr.n_vars);
        if (hdr.n_vars > 0) {
            if (!recvExact(clientFd, frame.vars.data(),
                           hdr.n_vars * sizeof(GVizVarEntry))) return;
        }

        // Edges
        frame.edges.resize(hdr.n_edges);
        if (hdr.n_edges > 0) {
            if (!recvExact(clientFd, frame.edges.data(),
                           hdr.n_edges * sizeof(GVizEdgeEntry))) return;
        }

        // Overwrite pending (last-write-wins, GUI throttles naturally at 60 Hz)
        {
            std::lock_guard<std::mutex> g(mtx_);
            pending_ = std::move(frame);
            hasNew_  = true;
        }
    }
}

// ── Poll (main/GUI thread) ────────────────────────────────────────────────────

bool GVizServer::poll(FactorGraphState& state) {
    std::optional<GVizFrame> frame;
    {
        std::lock_guard<std::mutex> g(mtx_);
        if (!hasNew_) return false;
        frame   = std::move(pending_);
        hasNew_ = false;
        pending_.reset();
    }
    if (!frame) return false;

    applyFrame(*frame, state);
    return true;
}

// ── Apply frame to FactorGraphState ───────────────────────────────────────────

static VariableType toVarType(gviz_ipc::VarType t) {
    switch (t) {
    case gviz_ipc::VarType::Pose2:  return VariableType::Pose2;
    case gviz_ipc::VarType::Pose3:  return VariableType::Pose3;
    case gviz_ipc::VarType::Point2: return VariableType::Point2;
    case gviz_ipc::VarType::Point3: return VariableType::Point3;
    default:                        return VariableType::Unknown;
    }
}

static FactorType toFactorType(gviz_ipc::FactorType t) {
    switch (t) {
    case gviz_ipc::FactorType::Prior:      return FactorType::Prior;
    case gviz_ipc::FactorType::Between:    return FactorType::Between;
    case gviz_ipc::FactorType::Projection: return FactorType::Projection;
    default:                               return FactorType::Custom;
    }
}

void GVizServer::applyFrame(const GVizFrame& frame, FactorGraphState& state) {

    if (frame.type == MsgType::Clear) {
        state.clear();
        return;
    }

    // For Replace: rebuild variable list completely
    // For Append / ValuesOnly: update existing, add new
    bool replace = (frame.type == MsgType::Replace);

    if (replace) {
        // Directly overwrite variable metadata without touching GTSAM objects
        // (we have no GTSAM graph here — only visual metadata)
        state.clearVisualOnly();
    }

    for (auto& ve : frame.vars) {
        VariableNode vn;
        vn.key      = ve.key;
        // Build label from key
        try {
            gtsam::Symbol sym(ve.key);
            std::ostringstream oss; oss << sym.chr() << sym.index();
            vn.label = oss.str();
        } catch (...) {
            vn.label = "k" + std::to_string(ve.key);
        }
        vn.type       = toVarType(static_cast<gviz_ipc::VarType>(ve.var_type));
        vn.position3d = { ve.position[0], ve.position[1], ve.position[2] };

        // Rebuild glm::mat4 from column-major float[16]
        vn.transform = glm::mat4(
            ve.transform[0],  ve.transform[1],  ve.transform[2],  ve.transform[3],
            ve.transform[4],  ve.transform[5],  ve.transform[6],  ve.transform[7],
            ve.transform[8],  ve.transform[9],  ve.transform[10], ve.transform[11],
            ve.transform[12], ve.transform[13], ve.transform[14], ve.transform[15]
        );

        state.upsertVariable(std::move(vn));
    }

    if (frame.type == MsgType::ValuesOnly) {
        // Only positions changed, skip topology update
        state.notifyChangedPublic();
        return;
    }

    if (replace) state.clearFactorsVisual();

    size_t idx = replace ? 0 : state.factors().size();
    for (auto& ee : frame.edges) {
        FactorNode fn;
        fn.index       = idx++;
        fn.type        = toFactorType(static_cast<gviz_ipc::FactorType>(ee.factor_type));
        fn.error       = ee.error;
        fn.keys        = { ee.key_from, ee.key_to };
        fn.label       = (fn.type == FactorType::Prior)   ? "Prior"
                       : (fn.type == FactorType::Between)  ? "Between"
                       : (fn.type == FactorType::Projection)? "Proj"
                                                            : "Custom";
        // Position = centroid of connected variables
        glm::vec2 sum{0,0}; int cnt = 0;
        for (auto k : fn.keys) {
            for (auto& vn : state.variables())
                if (vn.key == k) { sum += vn.pos; ++cnt; }
        }
        fn.pos = cnt > 0 ? sum / float(cnt) : glm::vec2{0,0};
        state.appendFactorVisual(std::move(fn));
    }

    state.notifyChangedPublic();

    if (!frame.label.empty()) {
        GVLOG_DEBUG("[IPC] seq=" + std::to_string(frame.seq_id)
                    + " \"" + frame.label + "\""
                    + " vars=" + std::to_string(state.variables().size())
                    + " factors=" + std::to_string(state.factors().size()));
    }
}

} // namespace gtsam_viz
