#include "GVizServer.h"
#include "../gui/panels/LogPanel.h"
#include "../graph/ResidualColorScale.h"

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <cmath>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace gtsam_viz {

using namespace gviz_ipc;

GVizServer::GVizServer()  = default;
GVizServer::~GVizServer() { stop(); }

static glm::vec3 toRenderCoords(float x, float y, float z) {
    return {x, y, z};
}

static glm::mat3 toRenderRotation(const glm::mat3& rotation) {
    return rotation;
}

static glm::mat3 toRenderCovariance(const glm::mat3& covariance) {
    return covariance;
}

static glm::mat4 toRenderTransform(const float transform[16]) {
    glm::mat3 rotation(
        transform[0], transform[1], transform[2],
        transform[4], transform[5], transform[6],
        transform[8], transform[9], transform[10]);
    glm::vec3 translation(transform[12], transform[13], transform[14]);

    glm::mat3 renderRotation = toRenderRotation(rotation);
    glm::vec3 renderTranslation = toRenderCoords(translation.x, translation.y, translation.z);

    glm::mat4 out(1.f);
    out[0] = glm::vec4(renderRotation[0], 0.f);
    out[1] = glm::vec4(renderRotation[1], 0.f);
    out[2] = glm::vec4(renderRotation[2], 0.f);
    out[3] = glm::vec4(renderTranslation, 1.f);
    return out;
}

static glm::mat3 rowMajorRotationToRender(const float* data) {
    glm::mat3 rotation(
        data[0], data[3], data[6],
        data[1], data[4], data[7],
        data[2], data[5], data[8]);
    return toRenderRotation(rotation);
}

static void writeRenderRotationRowMajor(const glm::mat3& rotation, float* data) {
    data[0] = rotation[0][0]; data[1] = rotation[1][0]; data[2] = rotation[2][0];
    data[3] = rotation[0][1]; data[4] = rotation[1][1]; data[5] = rotation[2][1];
    data[6] = rotation[0][2]; data[7] = rotation[1][2]; data[8] = rotation[2][2];
}

static void convertVec3Payload(float* data, int offset) {
    glm::vec3 p = toRenderCoords(data[offset], data[offset + 1], data[offset + 2]);
    data[offset] = p.x;
    data[offset + 1] = p.y;
    data[offset + 2] = p.z;
}

static void convertCovarianceRowMajor(float covariance[9]) {
    glm::mat3 cov(
        covariance[0], covariance[3], covariance[6],
        covariance[1], covariance[4], covariance[7],
        covariance[2], covariance[5], covariance[8]);
    glm::mat3 renderCov = toRenderCovariance(cov);
    writeRenderRotationRowMajor(renderCov, covariance);
}

// ── Start / Stop ──────────────────────────────────────────────────────────────

bool GVizServer::start() {
    if (running_) return true;

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

// ── Accept loop ───────────────────────────────────────────────────────────────

void GVizServer::acceptLoop() {
    while (running_) {
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

        std::lock_guard<std::mutex> g(mtx_);
        pending_.reset();
        hasNew_ = false;
    }
}

// ── Recv loop ─────────────────────────────────────────────────────────────────

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
        // ── Header ────────────────────────────────────────────────────────────
        GVizMsgHeader hdr{};
        if (!recvExact(clientFd, &hdr, sizeof(hdr))) return;

        if (hdr.magic != MAGIC_V2) {
            std::ostringstream oss;
            oss << "[GVizServer] Unknown magic 0x" << std::hex << std::uppercase
                << hdr.magic << " — dropping connection (expected protocol v2).";
            GVLOG_WARN(oss.str());
            return;
        }

        GVizFrame frame;
        frame.type   = static_cast<MsgType>(hdr.type);
        frame.seq_id = hdr.seq_id;

        // ── Label ─────────────────────────────────────────────────────────────
        if (hdr.label_len > 0) {
            frame.label.resize(hdr.label_len);
            if (!recvExact(clientFd, frame.label.data(), hdr.label_len)) return;
        }

        // ── Variables ─────────────────────────────────────────────────────────
        frame.vars.resize(hdr.n_vars);
        if (hdr.n_vars > 0)
            if (!recvExact(clientFd, frame.vars.data(),
                           hdr.n_vars * sizeof(GVizVarEntry))) return;

        // ── Edges ─────────────────────────────────────────────────────────────
        frame.edges.resize(hdr.n_edges);
        if (hdr.n_edges > 0)
            if (!recvExact(clientFd, frame.edges.data(),
                           hdr.n_edges * sizeof(GVizEdgeEntry))) return;

        // ── Point clouds (v2) ─────────────────────────────────────────────────
        for (uint16_t ci = 0; ci < hdr.n_clouds; ++ci) {
            GVizCloudHeader ch{};
            if (!recvExact(clientFd, &ch, sizeof(ch))) return;

            DecodedCloud dc;
            dc.point_size = ch.point_size;
            dc.default_color = { ch.default_color[0], ch.default_color[1],
                                  ch.default_color[2], ch.default_color[3] };

            // Positions
            dc.positions.resize(ch.n_points);
            if (ch.n_points > 0)
                if (!recvExact(clientFd, dc.positions.data(),
                               ch.n_points * 3 * sizeof(float))) return;

            // Per-point colors
            if (ch.has_colors && ch.n_points > 0) {
                dc.colors.resize(ch.n_points);
                if (!recvExact(clientFd, dc.colors.data(),
                               ch.n_points * 4)) return;
            }

            frame.point_clouds.push_back(std::move(dc));
        }

        // ── Primitives (v2) ───────────────────────────────────────────────────
        frame.primitives.resize(hdr.n_primitives);
        if (hdr.n_primitives > 0)
            if (!recvExact(clientFd, frame.primitives.data(),
                           hdr.n_primitives * sizeof(GVizPrimEntry))) return;

        // Last-write-wins
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
    case VarType::Pose2:  return VariableType::Pose2;
    case VarType::Pose3:  return VariableType::Pose3;
    case VarType::Point2: return VariableType::Point2;
    case VarType::Point3: return VariableType::Point3;
    default:              return VariableType::Unknown;
    }
}

static gtsam_viz::FactorType toFactorType(gviz_ipc::FactorType t) {
    switch (t) {
    case gviz_ipc::FactorType::Prior:      return gtsam_viz::FactorType::Prior;
    case gviz_ipc::FactorType::Between:    return gtsam_viz::FactorType::Between;
    case gviz_ipc::FactorType::Projection: return gtsam_viz::FactorType::Projection;
    default:                               return gtsam_viz::FactorType::Custom;
    }
}

// Convert DecodedClouds → PointCloud structs for FactorGraphState
static std::vector<PointCloud> toClouds(const std::vector<DecodedCloud>& dcs) {
    std::vector<PointCloud> out;
    out.reserve(dcs.size());
    for (auto& dc : dcs) {
        PointCloud pc;
        pc.point_size    = dc.point_size;
        pc.default_color = { dc.default_color[0], dc.default_color[1],
                              dc.default_color[2], dc.default_color[3] };
        pc.points.reserve(dc.positions.size());
        for (auto& p : dc.positions)
            pc.points.push_back(toRenderCoords(p[0], p[1], p[2]));
        if (!dc.colors.empty()) {
            pc.colors.reserve(dc.colors.size());
            for (auto& c : dc.colors)
                pc.colors.push_back({ c[0], c[1], c[2], c[3] });
        }
        out.push_back(std::move(pc));
    }
    return out;
}

// Convert GVizPrimEntry → Primitive structs
static std::vector<Primitive> toPrimitives(const std::vector<gviz_ipc::GVizPrimEntry>& entries) {
    std::vector<Primitive> out;
    out.reserve(entries.size());
    for (auto& e : entries) {
        Primitive p;
        p.type  = static_cast<PrimType>(e.prim_type);
        p.color = { e.color[0], e.color[1], e.color[2], e.color[3] };
        std::memcpy(p.data, e.data, sizeof(p.data));
        switch (p.type) {
        case PrimType::Line:
        case PrimType::Arrow:
            convertVec3Payload(p.data, 0);
            convertVec3Payload(p.data, 3);
            break;
        case PrimType::Box: {
            convertVec3Payload(p.data, 0);
            glm::mat3 R = rowMajorRotationToRender(&p.data[6]);
            writeRenderRotationRowMajor(R, &p.data[6]);
            break;
        }
        case PrimType::Sphere:
            convertVec3Payload(p.data, 0);
            break;
        case PrimType::Cone:
            convertVec3Payload(p.data, 0);
            convertVec3Payload(p.data, 3);
            break;
        case PrimType::Cylinder:
            convertVec3Payload(p.data, 0);
            convertVec3Payload(p.data, 3);
            break;
        case PrimType::CoordFrame: {
            convertVec3Payload(p.data, 0);
            glm::mat3 R = rowMajorRotationToRender(&p.data[4]);
            writeRenderRotationRowMajor(R, &p.data[4]);
            break;
        }
        }
        out.push_back(p);
    }
    return out;
}

static const char* shortFactorLabel(gtsam_viz::FactorType type) {
    switch (type) {
    case gtsam_viz::FactorType::Prior:      return "Prior";
    case gtsam_viz::FactorType::Between:    return "Between";
    case gtsam_viz::FactorType::Projection: return "Proj";
    default:                     return "Custom";
    }
}

static std::vector<FactorNode> toFactorNodes(const std::vector<gviz_ipc::GVizEdgeEntry>& edges,
                                             const FactorGraphState& state,
                                             size_t firstIndex) {
    std::vector<FactorNode> out;
    out.reserve(edges.size());
    size_t idx = firstIndex;
    for (auto& ee : edges) {
        FactorNode fn;
        fn.index  = idx++;
        fn.type   = toFactorType(static_cast<gviz_ipc::FactorType>(ee.factor_type));
        fn.error  = ee.error;
        fn.errorValid = isResidualValid(fn.error);
        fn.errorFresh = fn.errorValid;
        fn.errorSource = fn.errorValid ? FactorErrorSource::Ipc : FactorErrorSource::None;
        fn.keys   = { ee.key_from, ee.key_to };
        fn.label  = shortFactorLabel(fn.type);
        glm::vec2 sum{0, 0}; int cnt = 0;
        for (auto k : fn.keys)
            for (auto& vn : state.variables())
                if (vn.key == k) { sum += vn.pos; ++cnt; }
        fn.pos = cnt > 0 ? sum / float(cnt) : glm::vec2{0, 0};
        out.push_back(std::move(fn));
    }
    return out;
}

void GVizServer::applyFrame(const GVizFrame& frame, FactorGraphState& state) {

    // ── Standalone cloud/primitive messages ───────────────────────────────────
    if (frame.type == MsgType::Clear) {
        state.clear();
        return;
    }
    if (frame.type == MsgType::PointCloud) {
        state.setPointClouds(toClouds(frame.point_clouds));
        return;
    }
    if (frame.type == MsgType::Primitives) {
        state.setPrimitives(toPrimitives(frame.primitives));
        return;
    }

    // ── Graph update ──────────────────────────────────────────────────────────
    bool replace = (frame.type == MsgType::Replace);

    if (replace) {
        state.clearVisualOnly();
        if (!frame.point_clouds.empty()) state.clearPointClouds();
        if (!frame.primitives.empty())   state.clearPrimitives();
    }

    // Apply variables (includes covariance data if has_covariance=1)
    for (auto& ve : frame.vars) {
        VariableNode vn;
        vn.key = ve.key;
        try {
            gtsam::Symbol sym(ve.key);
            std::ostringstream oss; oss << sym.chr() << sym.index();
            vn.label = oss.str();
        } catch (...) {
            vn.label = "k" + std::to_string(ve.key);
        }
        vn.type       = toVarType(static_cast<VarType>(ve.var_type));
        vn.position3d = toRenderCoords(ve.position[0], ve.position[1], ve.position[2]);
        vn.transform  = toRenderTransform(ve.transform);
        vn.has_covariance = (ve.has_covariance != 0);
        if (vn.has_covariance) {
            std::memcpy(vn.covariance3d, ve.covariance, sizeof(vn.covariance3d));
            convertCovarianceRowMajor(vn.covariance3d);
        }

        state.upsertVariable(std::move(vn));
    }

    if (frame.type == MsgType::ValuesOnly) {
        if (frame.edges.empty()) {
            state.markFactorErrorsStale();
        } else {
            state.refreshVisualFactorErrors(toFactorNodes(frame.edges, state, 0));
        }
        state.notifyChangedPublic();
        return;
    }

    if (replace) state.clearFactorsVisual();

    size_t idx = replace ? 0 : state.factors().size();
    for (auto& fn : toFactorNodes(frame.edges, state, idx)) {
        state.appendFactorVisual(std::move(fn));
    }

    // Embedded clouds/prims in Replace/Append messages
    if (!frame.point_clouds.empty())
        state.setPointClouds(toClouds(frame.point_clouds));
    if (!frame.primitives.empty())
        state.setPrimitives(toPrimitives(frame.primitives));

    state.notifyChangedPublic();

    if (!frame.label.empty()) {
        GVLOG_DEBUG("[IPC] seq=" + std::to_string(frame.seq_id)
                    + " \"" + frame.label + "\""
                    + " vars=" + std::to_string(state.variables().size())
                    + " factors=" + std::to_string(state.factors().size())
                    + " clouds=" + std::to_string(state.pointClouds().size())
                    + " prims=" + std::to_string(state.primitives().size()));
    }
}

} // namespace gtsam_viz
