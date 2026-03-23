#pragma once
/**
 * gviz_client.h  –  Header-only IPC client for GTSAMViz
 * ======================================================
 * Copy this single file into your SLAM project.
 * Dependencies: GTSAM + POSIX sockets (Linux/macOS). No OpenGL, no ImGui.
 *
 * USAGE
 * ──────
 *   #include "gviz_client.h"
 *
 *   GVizClient viz;
 *   viz.connect();                                         // connect to running GUI
 *
 *   viz.publish(graph, values, "keyframe 42");             // full replace
 *   viz.publishValuesOnly(optimized_values, "post-opt");   // positions only
 *   viz.publishAppend(new_factors, new_values, "step N");  // incremental
 *   viz.clear();
 *
 *   viz.disconnect();   // or just let it destruct
 *
 * If the GUI is not running, connect() is a no-op and all publish calls
 * silently do nothing — zero overhead on your SLAM thread.
 *
 * SOCKET PATH: /tmp/gtsam_viz.sock  (same as GVizProtocol.h)
 */

#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>
#include <algorithm>

// ── Protocol constants (duplicated to keep this header self-contained) ────────
namespace gviz_detail {

constexpr uint32_t MAGIC         = 0x4756495A;
constexpr char     SOCKET_PATH[] = "/tmp/gtsam_viz.sock";

enum class MsgType   : uint8_t { Replace=0, Append=1, ValuesOnly=2, Clear=3 };
enum class VarType   : uint8_t { Pose2=0, Pose3=1, Point2=2, Point3=3, Unknown=4 };
enum class FactorType: uint8_t { Prior=0, Between=1, Projection=2, Custom=3 };

#pragma pack(push, 1)
struct MsgHeader {
    uint32_t magic;
    uint8_t  type;
    uint32_t seq_id;
    uint32_t n_vars;
    uint32_t n_edges;
    uint16_t label_len;
};
struct VarEntry {
    uint64_t key;
    float    transform[16]; ///< column-major 4×4
    float    position[3];
    uint8_t  var_type;
};
struct EdgeEntry {
    uint64_t key_from;
    uint64_t key_to;
    uint8_t  factor_type;
    float    error;
};
#pragma pack(pop)

// ── Helpers ───────────────────────────────────────────────────────────────────

inline VarType detectVarType(gtsam::Key key, const gtsam::Values& v) {
    if (!v.exists(key)) return VarType::Unknown;
    try { v.at<gtsam::Pose3>(key);  return VarType::Pose3;  } catch (...) {}
    try { v.at<gtsam::Pose2>(key);  return VarType::Pose2;  } catch (...) {}
    try { v.at<gtsam::Point3>(key); return VarType::Point3; } catch (...) {}
    try { v.at<gtsam::Point2>(key); return VarType::Point2; } catch (...) {}
    return VarType::Unknown;
}

inline VarEntry makeVarEntry(gtsam::Key key, const gtsam::Values& v) {
    VarEntry e{};
    e.key      = key;
    e.var_type = (uint8_t)detectVarType(key, v);

    if (!v.exists(key)) return e;

    // Default: identity
    e.transform[0]=e.transform[5]=e.transform[10]=e.transform[15]=1.f;

    try {
        auto p   = v.at<gtsam::Pose3>(key);
        auto t   = p.translation();
        e.position[0] = (float)t.x();
        e.position[1] = (float)t.y();
        e.position[2] = (float)t.z();
        auto R = p.rotation().matrix();
        // column-major
        e.transform[0]=(float)R(0,0); e.transform[1]=(float)R(1,0); e.transform[2]=(float)R(2,0); e.transform[3]=0;
        e.transform[4]=(float)R(0,1); e.transform[5]=(float)R(1,1); e.transform[6]=(float)R(2,1); e.transform[7]=0;
        e.transform[8]=(float)R(0,2); e.transform[9]=(float)R(1,2); e.transform[10]=(float)R(2,2);e.transform[11]=0;
        e.transform[12]=(float)t.x(); e.transform[13]=(float)t.y(); e.transform[14]=(float)t.z(); e.transform[15]=1;
        return e;
    } catch (...) {}

    try {
        auto p = v.at<gtsam::Pose2>(key);
        e.position[0]=(float)p.x(); e.position[1]=(float)p.y(); e.position[2]=0;
        float c=(float)std::cos(p.theta()), s=(float)std::sin(p.theta());
        e.transform[0]=c;  e.transform[1]=s;  e.transform[2]=0; e.transform[3]=0;
        e.transform[4]=-s; e.transform[5]=c;  e.transform[6]=0; e.transform[7]=0;
        e.transform[8]=0;  e.transform[9]=0;  e.transform[10]=1;e.transform[11]=0;
        e.transform[12]=(float)p.x(); e.transform[13]=(float)p.y(); e.transform[14]=0; e.transform[15]=1;
        return e;
    } catch (...) {}

    try {
        auto p = v.at<gtsam::Point3>(key);
        e.position[0]=(float)p.x(); e.position[1]=(float)p.y(); e.position[2]=(float)p.z();
        e.transform[12]=(float)p.x(); e.transform[13]=(float)p.y(); e.transform[14]=(float)p.z();
        return e;
    } catch (...) {}

    try {
        auto p = v.at<gtsam::Point2>(key);
        e.position[0]=(float)p.x(); e.position[1]=(float)p.y();
        e.transform[12]=(float)p.x(); e.transform[13]=(float)p.y();
        return e;
    } catch (...) {}

    return e;
}

inline FactorType detectFactorType(const gtsam::NonlinearFactor::shared_ptr& f) {
    if (!f) return FactorType::Custom;
    std::string name = typeid(*f).name();
    if (name.find("PriorFactor")   != std::string::npos) return FactorType::Prior;
    if (name.find("BetweenFactor") != std::string::npos) return FactorType::Between;
    return FactorType::Custom;
}

} // namespace gviz_detail

// ── Public client class ───────────────────────────────────────────────────────
class GVizClient {
public:
    GVizClient()  = default;
    ~GVizClient() { disconnect(); }

    // Non-copyable
    GVizClient(const GVizClient&)            = delete;
    GVizClient& operator=(const GVizClient&) = delete;

    /**
     * Connect to the GTSAMViz GUI process.
     * Safe to call even if GUI is not running — returns false silently.
     * You can call connect() again later after the GUI has started.
     */
    bool connect() {
        if (fd_ >= 0) return true;

        fd_ = ::socket(AF_UNIX, SOCK_STREAM, 0);
        if (fd_ < 0) { fd_ = -1; return false; }

        struct sockaddr_un addr{};
        addr.sun_family = AF_UNIX;
        std::strncpy(addr.sun_path, gviz_detail::SOCKET_PATH,
                     sizeof(addr.sun_path)-1);

        if (::connect(fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            ::close(fd_); fd_ = -1;
            return false;
        }
        return true;
    }

    void disconnect() {
        if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
    }

    bool isConnected() const { return fd_ >= 0; }

    /**
     * Full graph replace — send complete snapshot.
     * Replaces everything currently shown in the GUI.
     */
    bool publish(const gtsam::NonlinearFactorGraph& graph,
                 const gtsam::Values& values,
                 const std::string& label = "") {
        return send(graph, values, gviz_detail::MsgType::Replace, label);
    }

    /**
     * Values-only update — update node positions, keep topology.
     * Use after optimization: fast, sends only positions.
     */
    bool publishValuesOnly(const gtsam::Values& values,
                           const std::string& label = "") {
        gtsam::NonlinearFactorGraph empty;
        return send(empty, values, gviz_detail::MsgType::ValuesOnly, label);
    }

    /**
     * Append new factors/variables to existing graph.
     * Use for incremental iSAM2-style updates.
     */
    bool publishAppend(const gtsam::NonlinearFactorGraph& new_factors,
                       const gtsam::Values& new_values,
                       const std::string& label = "") {
        return send(new_factors, new_values, gviz_detail::MsgType::Append, label);
    }

    /// Clear the GUI display
    bool clear() {
        if (!ensureConnected()) return false;
        gviz_detail::MsgHeader hdr{};
        hdr.magic = gviz_detail::MAGIC;
        hdr.type  = (uint8_t)gviz_detail::MsgType::Clear;
        hdr.seq_id = seq_++;
        return sendExact(&hdr, sizeof(hdr));
    }

private:
    int      fd_  = -1;
    uint32_t seq_ = 0;

    bool ensureConnected() {
        if (fd_ >= 0) return true;
        return connect();     // auto-reconnect
    }

    bool sendExact(const void* buf, size_t n) {
        const char* p = (const char*)buf;
        size_t sent = 0;
        while (sent < n) {
            ssize_t r = ::send(fd_, p+sent, n-sent, MSG_NOSIGNAL);
            if (r <= 0) { disconnect(); return false; }
            sent += (size_t)r;
        }
        return true;
    }

    bool send(const gtsam::NonlinearFactorGraph& graph,
              const gtsam::Values& values,
              gviz_detail::MsgType type,
              const std::string& label) {

        if (!ensureConnected()) return false;

        using namespace gviz_detail;

        // Build var entries
        std::vector<VarEntry> vars;
        vars.reserve(values.size());
        for (const auto& kv : values)
            vars.push_back(makeVarEntry(kv.key, values));

        // Build edge entries
        std::vector<EdgeEntry> edges;
        edges.reserve(graph.size());
        for (size_t i = 0; i < graph.size(); ++i) {
            auto& f = graph[i];
            if (!f || f->keys().size() < 1) continue;
            // Emit one edge per adjacent key pair
            auto& keys = f->keys();
            FactorType ft = detectFactorType(f);
            float err = 0.f;
            try { err = (float)f->error(values); } catch (...) {}

            if (keys.size() == 1) {
                // Unary (prior) — encode as self-loop so the GUI can show it
                edges.push_back({ keys[0], keys[0], (uint8_t)ft, err });
            } else {
                for (size_t k = 0; k+1 < keys.size(); ++k)
                    edges.push_back({ keys[k], keys[k+1], (uint8_t)ft, err });
            }
        }

        // Header
        MsgHeader hdr{};
        hdr.magic     = MAGIC;
        hdr.type      = (uint8_t)type;
        hdr.seq_id    = seq_++;
        hdr.n_vars    = (uint32_t)vars.size();
        hdr.n_edges   = (uint32_t)edges.size();
        hdr.label_len = (uint16_t)std::min(label.size(), (size_t)65535);

        if (!sendExact(&hdr, sizeof(hdr)))                        return false;
        if (hdr.label_len > 0 &&
            !sendExact(label.data(), hdr.label_len))              return false;
        if (!vars.empty() &&
            !sendExact(vars.data(), vars.size()*sizeof(VarEntry))) return false;
        if (!edges.empty() &&
            !sendExact(edges.data(), edges.size()*sizeof(EdgeEntry))) return false;

        return true;
    }
};
