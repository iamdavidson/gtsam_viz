#pragma once
/**
 * GVizClient.h  –  Header-only SLAM → GTSAMViz IPC client  (Protocol v2)
 * =========================================================================
 *
 * Kopiere NUR diese Datei + GVizProtocol.h in dein Projekt.
 * Abhängigkeiten: GTSAM, Eigen (kommt automatisch über GTSAM), POSIX.
 *
 * ── BASISNUTZUNG ────────────────────────────────────────────────────────────
 *
 *   GVizClient viz;
 *   viz.connect();
 *   viz.publish(graph, values, "keyframe 42");
 *   viz.publishValuesWithErrors(graph, optimized_values);
 *   viz.clear();
 *   viz.disconnect();
 *
 * ── KOVARIANZ (Ellipsoide anzeigen) ─────────────────────────────────────────
 *
 *   GVizClient::CovarianceMap covs;
 *   for (auto key : keys)
 *       covs[key] = marginals.marginalCovariance(key).block<3,3>(3,3); // Pose3: off=3
 *   viz.publish(graph, values, "kf42", &covs);
 *
 * ── PUNKTWOLKEN ──────────────────────────────────────────────────────────────
 *
 *   std::vector<Eigen::Vector3f> pts = ...;
 *   viz.publishPointCloud(pts, "lidar scan");
 *
 *   // Mit per-Punkt-Farben:
 *   std::vector<Eigen::Vector4f> colors = ...;  // RGBA [0,1]
 *   viz.publishPointCloud(pts, colors, "colored scan");
 *
 * ── PRIMITIVE (SceneBuilder) ─────────────────────────────────────────────────
 *
 *   viz.scene()
 *      .arrow({0,0,0}, {1,0,0}, {1,0,0,1})          // roter Pfeil
 *      .box({2,0,0}, {0.5f,0.5f,0.5f}, {0,1,0,1})   // grüne Box
 *      .sphere({4,0,0}, 0.3f, {0,0,1,1})             // blauer Ball
 *      .coordFrame(pose_matrix, 0.5f)                  // Koordinatenrahmen
 *      .send(viz, "obstacles");
 *
 * ── THREAD-SICHERHEIT ────────────────────────────────────────────────────────
 *   Alle Methoden sind thread-safe (ein Mutex schützt den Socket).
 *   publish() blockiert maximal ~1 ms (Kernel-Buffer, Loopback).
 */

#include "GVizProtocol.h"

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/inference/Symbol.h>

#include <Eigen/Core>

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <algorithm>
#include <cstring>
#include <cstdint>
#include <cmath>
#include <mutex>
#include <string>
#include <vector>
#include <array>
#include <unordered_map>

// ── Forward declaration for SceneBuilder ──────────────────────────────────────
class GVizClient;

// ─────────────────────────────────────────────────────────────────────────────
//  SceneBuilder  –  Fluent builder for 3-D primitives
// ─────────────────────────────────────────────────────────────────────────────
class SceneBuilder {
public:
    SceneBuilder() = default;

    /// Line segment
    SceneBuilder& line(const Eigen::Vector3f& from, const Eigen::Vector3f& to,
                       const Eigen::Vector4f& color = Eigen::Vector4f(1,1,1,1),
                       float width = 1.f) {
        auto& e = addEntry(gviz_ipc::PrimType::Line, color);
        setVec3(e.data, 0, from); setVec3(e.data, 3, to); e.data[6] = width;
        return *this;
    }

    /// Arrow (shaft + auto cone head, 20 % of length)
    SceneBuilder& arrow(const Eigen::Vector3f& from, const Eigen::Vector3f& to,
                        const Eigen::Vector4f& color = Eigen::Vector4f(1,1,1,1)) {
        auto& e = addEntry(gviz_ipc::PrimType::Arrow, color);
        setVec3(e.data, 0, from); setVec3(e.data, 3, to);
        return *this;
    }

    /// Axis-aligned or rotated box (solid, Phong-shaded)
    SceneBuilder& box(const Eigen::Vector3f& center,
                      const Eigen::Vector3f& half_extents,
                      const Eigen::Vector4f& color = Eigen::Vector4f(1,1,1,1),
                      const Eigen::Matrix3f& rotation = Eigen::Matrix3f::Identity()) {
        auto& e = addEntry(gviz_ipc::PrimType::Box, color);
        setVec3(e.data, 0, center);
        setVec3(e.data, 3, half_extents);
        // Row-major 3×3 rotation into data[6..14]
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                e.data[6 + r*3 + c] = rotation(r, c);
        return *this;
    }

    /// Sphere (solid, Phong-shaded)
    SceneBuilder& sphere(const Eigen::Vector3f& center, float radius,
                         const Eigen::Vector4f& color = Eigen::Vector4f(1,1,1,1)) {
        auto& e = addEntry(gviz_ipc::PrimType::Sphere, color);
        setVec3(e.data, 0, center); e.data[3] = radius;
        return *this;
    }

    /// Cone  apex → base (solid)
    SceneBuilder& cone(const Eigen::Vector3f& apex,
                       const Eigen::Vector3f& base_center, float base_radius,
                       const Eigen::Vector4f& color = Eigen::Vector4f(1,1,1,1)) {
        auto& e = addEntry(gviz_ipc::PrimType::Cone, color);
        setVec3(e.data, 0, apex); setVec3(e.data, 3, base_center);
        e.data[6] = base_radius;
        return *this;
    }

    /// Cylinder: center, axis direction (world space), radius, half-height (solid)
    SceneBuilder& cylinder(const Eigen::Vector3f& center,
                           const Eigen::Vector3f& axis,
                           float radius, float half_height,
                           const Eigen::Vector4f& color = Eigen::Vector4f(1,1,1,1)) {
        auto& e = addEntry(gviz_ipc::PrimType::Cylinder, color);
        setVec3(e.data, 0, center); setVec3(e.data, 3, axis);
        e.data[6] = radius; e.data[7] = half_height;
        return *this;
    }

    /// Coordinate frame from a full 4×4 column-major transform (like OpenGL)
    SceneBuilder& coordFrame(const Eigen::Matrix4f& transform,
                              float axis_length = 0.5f) {
        Eigen::Vector3f pos  = transform.block<3,1>(0,3);
        Eigen::Matrix3f rot  = transform.block<3,3>(0,0);
        return coordFrame(pos, rot, axis_length);
    }

    /// Coordinate frame from position + rotation matrix
    SceneBuilder& coordFrame(const Eigen::Vector3f& position,
                              const Eigen::Matrix3f& rotation,
                              float axis_length = 0.5f) {
        auto& e = addEntry(gviz_ipc::PrimType::CoordFrame, Eigen::Vector4f(1,1,1,1));
        setVec3(e.data, 0, position);
        e.data[3] = axis_length;
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                e.data[4 + r*3 + c] = rotation(r, c);
        return *this;
    }

    bool empty() const { return prims_.empty(); }
    void clear()       { prims_.clear(); }
    size_t size() const { return prims_.size(); }

    /// Transmit all primitives to the viewer (replaces current primitive layer).
    bool send(GVizClient& client, const std::string& label = "");

private:
    std::vector<gviz_ipc::GVizPrimEntry> prims_;

    gviz_ipc::GVizPrimEntry& addEntry(gviz_ipc::PrimType type,
                                       const Eigen::Vector4f& color) {
        prims_.emplace_back();
        auto& e = prims_.back();
        std::memset(&e, 0, sizeof(e));
        e.prim_type = static_cast<uint8_t>(type);
        e.color[0] = color[0]; e.color[1] = color[1];
        e.color[2] = color[2]; e.color[3] = color[3];
        return e;
    }

    static void setVec3(float* data, int offset, const Eigen::Vector3f& v) {
        data[offset]   = v[0];
        data[offset+1] = v[1];
        data[offset+2] = v[2];
    }

    friend class GVizClient;
};

// ─────────────────────────────────────────────────────────────────────────────
//  GVizClient
// ─────────────────────────────────────────────────────────────────────────────
class GVizClient {
public:
    using FactorTypeAlias = gviz_ipc::FactorType;
    static constexpr FactorTypeAlias Prior      = FactorTypeAlias::Prior;
    static constexpr FactorTypeAlias Between    = FactorTypeAlias::Between;
    static constexpr FactorTypeAlias Projection = FactorTypeAlias::Projection;
    static constexpr FactorTypeAlias Custom     = FactorTypeAlias::Custom;

    /// Map from variable key → 3×3 position covariance (XYZ, world space).
    /// For Pose3: extract block<3,3>(3,3) from the 6×6 GTSAM marginal.
    /// For Point3: the full 3×3 marginal.
    using CovarianceMap = std::unordered_map<gtsam::Key, Eigen::Matrix3d>;

    GVizClient() = default;
    ~GVizClient() { disconnect(); }

    // ── Connection ────────────────────────────────────────────────────────────

    bool connect(const char* socket_path = gviz_ipc::SOCKET_PATH) {
        std::lock_guard<std::mutex> g(mtx_);
        if (fd_ >= 0) return true;
        socketPath_ = socket_path;
        fd_ = ::socket(AF_UNIX, SOCK_STREAM, 0);
        if (fd_ < 0) return false;
        struct sockaddr_un addr{};
        addr.sun_family = AF_UNIX;
        std::strncpy(addr.sun_path, socket_path, sizeof(addr.sun_path)-1);
        if (::connect(fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            ::close(fd_); fd_ = -1; return false;
        }
        seq_ = 0;
        return true;
    }

    void disconnect() {
        std::lock_guard<std::mutex> g(mtx_);
        if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
    }

    bool isConnected() const {
        std::lock_guard<std::mutex> g(mtx_); return fd_ >= 0;
    }

    // ── Graph API (unchanged from v1, covariance optional) ────────────────────

    /// Full graph replacement. Pass covariances to show uncertainty ellipsoids.
    bool publish(const gtsam::NonlinearFactorGraph& graph,
                 const gtsam::Values& values,
                 const std::string& label = "",
                 const CovarianceMap* covs = nullptr) {
        return transmit(gviz_ipc::MsgType::Replace,
                        buildVars(values, covs), buildEdges(graph, values),
                        label, {}, {});
    }

    /// Position-only update (topology unchanged).
    /// Fastest path, but residual colors become stale until errors are sent again.
    bool publishValuesOnly(const gtsam::Values& values,
                           const std::string& label = "") {
        return transmit(gviz_ipc::MsgType::ValuesOnly,
                        buildVars(values, nullptr), {}, label, {}, {});
    }

    /// Position update plus freshly computed factor errors (topology unchanged).
    bool publishValuesWithErrors(const gtsam::NonlinearFactorGraph& graph,
                                 const gtsam::Values& values,
                                 const std::string& label = "") {
        return transmit(gviz_ipc::MsgType::ValuesOnly,
                        buildVars(values, nullptr),
                        buildEdges(graph, values), label, {}, {});
    }

    /// Append new vars/edges (iSAM2 style).
    bool append(const gtsam::NonlinearFactorGraph& new_factors,
                const gtsam::Values& new_values,
                const std::string& label = "") {
        return transmit(gviz_ipc::MsgType::Append,
                        buildVars(new_values, nullptr),
                        buildEdges(new_factors, new_values), label, {}, {});
    }

    /// Append a single edge without GTSAM objects.
    bool appendEdge(gtsam::Key from, gtsam::Key to,
                    FactorTypeAlias ftype = FactorTypeAlias::Between,
                    float error = 0.f) {
        gviz_ipc::GVizEdgeEntry e{};
        e.key_from    = from; e.key_to = to;
        e.factor_type = static_cast<uint8_t>(ftype);
        e.error       = error;
        return transmit(gviz_ipc::MsgType::Append, {}, {e}, "", {}, {});
    }

    /// Clear everything (graph + clouds + primitives).
    bool clear() {
        return transmit(gviz_ipc::MsgType::Clear, {}, {}, "", {}, {});
    }

    // ── Point cloud API ───────────────────────────────────────────────────────

    /// Solid-color point cloud.
    bool publishPointCloud(const std::vector<Eigen::Vector3f>& pts,
                           const std::string& label   = "",
                           float point_size           = 3.f,
                           const Eigen::Vector4f& col = Eigen::Vector4f(1,1,1,1)) {
        CloudPayload cp;
        cp.point_size    = point_size;
        cp.default_color = { col[0], col[1], col[2], col[3] };
        cp.positions.reserve(pts.size());
        for (auto& p : pts) cp.positions.push_back({ p[0], p[1], p[2] });
        return transmit(gviz_ipc::MsgType::PointCloud, {}, {}, label, { std::move(cp) }, {});
    }

    /// Per-point colored point cloud (colors normalized RGBA [0,1]).
    bool publishPointCloud(const std::vector<Eigen::Vector3f>& pts,
                           const std::vector<Eigen::Vector4f>& colors,
                           const std::string& label = "",
                           float point_size         = 3.f) {
        CloudPayload cp;
        cp.point_size = point_size;
        cp.default_color = {1,1,1,1};
        cp.positions.reserve(pts.size());
        for (auto& p : pts) cp.positions.push_back({ p[0], p[1], p[2] });
        size_t n = std::min(pts.size(), colors.size());
        cp.colors.reserve(n);
        for (size_t i = 0; i < n; ++i) {
            auto& c = colors[i];
            cp.colors.push_back({
                static_cast<uint8_t>(std::clamp(c[0], 0.f, 1.f) * 255.f),
                static_cast<uint8_t>(std::clamp(c[1], 0.f, 1.f) * 255.f),
                static_cast<uint8_t>(std::clamp(c[2], 0.f, 1.f) * 255.f),
                static_cast<uint8_t>(std::clamp(c[3], 0.f, 1.f) * 255.f)
            });
        }
        return transmit(gviz_ipc::MsgType::PointCloud, {}, {}, label, { std::move(cp) }, {});
    }

    // ── Scene builder ─────────────────────────────────────────────────────────

    /// Returns a fresh SceneBuilder. Use builder.send(*this, label) to transmit.
    SceneBuilder scene() { return SceneBuilder{}; }

    /// Transmit a finished SceneBuilder (called by SceneBuilder::send).
    bool sendScene(SceneBuilder& b, const std::string& label) {
        return transmit(gviz_ipc::MsgType::Primitives, {}, {}, label,
                        {}, b.prims_);
    }

private:

    // ── Internal cloud payload (pre-serialization) ────────────────────────────
    struct CloudPayload {
        std::vector<std::array<float, 3>>    positions;
        std::vector<std::array<uint8_t, 4>>  colors;       // empty → solid color
        float                                point_size    = 3.f;
        std::array<float, 4>                 default_color = {1,1,1,1};
    };

    // ── Serialisation helpers ─────────────────────────────────────────────────

    static gviz_ipc::VarType detectVarType(gtsam::Key key, const gtsam::Values& v) {
        try { v.at<gtsam::Pose3>(key);  return gviz_ipc::VarType::Pose3;  } catch(...) {}
        try { v.at<gtsam::Pose2>(key);  return gviz_ipc::VarType::Pose2;  } catch(...) {}
        try { v.at<gtsam::Point3>(key); return gviz_ipc::VarType::Point3; } catch(...) {}
        try { v.at<gtsam::Point2>(key); return gviz_ipc::VarType::Point2; } catch(...) {}
        return gviz_ipc::VarType::Unknown;
    }

    static std::vector<gviz_ipc::GVizVarEntry>
    buildVars(const gtsam::Values& values, const CovarianceMap* covs) {
        std::vector<gviz_ipc::GVizVarEntry> out;
        out.reserve(values.size());
        for (const auto& kv : values) {
            gviz_ipc::GVizVarEntry e{};
            e.key      = kv.key;
            e.var_type = static_cast<uint8_t>(detectVarType(kv.key, values));

            switch (static_cast<gviz_ipc::VarType>(e.var_type)) {
            case gviz_ipc::VarType::Pose3: {
                auto p = values.at<gtsam::Pose3>(kv.key);
                auto t = p.translation(); auto R = p.rotation().matrix();
                e.position[0]=(float)t.x(); e.position[1]=(float)t.y(); e.position[2]=(float)t.z();
                // Column-major 4×4
                e.transform[0]=(float)R(0,0); e.transform[1]=(float)R(1,0); e.transform[2]=(float)R(2,0);  e.transform[3]=0;
                e.transform[4]=(float)R(0,1); e.transform[5]=(float)R(1,1); e.transform[6]=(float)R(2,1);  e.transform[7]=0;
                e.transform[8]=(float)R(0,2); e.transform[9]=(float)R(1,2); e.transform[10]=(float)R(2,2); e.transform[11]=0;
                e.transform[12]=(float)t.x(); e.transform[13]=(float)t.y(); e.transform[14]=(float)t.z();  e.transform[15]=1;
                break;
            }
            case gviz_ipc::VarType::Pose2: {
                auto p = values.at<gtsam::Pose2>(kv.key);
                float c=std::cos((float)p.theta()), s=std::sin((float)p.theta());
                e.position[0]=(float)p.x(); e.position[1]=(float)p.y(); e.position[2]=0;
                e.transform[0]=c; e.transform[1]=s;  e.transform[2]=0; e.transform[3]=0;
                e.transform[4]=-s;e.transform[5]=c;  e.transform[6]=0; e.transform[7]=0;
                e.transform[8]=0; e.transform[9]=0;  e.transform[10]=1;e.transform[11]=0;
                e.transform[12]=(float)p.x(); e.transform[13]=(float)p.y(); e.transform[14]=0; e.transform[15]=1;
                break;
            }
            case gviz_ipc::VarType::Point3: {
                auto p = values.at<gtsam::Point3>(kv.key);
                e.position[0]=(float)p.x(); e.position[1]=(float)p.y(); e.position[2]=(float)p.z();
                e.transform[0]=1; e.transform[5]=1; e.transform[10]=1;
                e.transform[12]=(float)p.x(); e.transform[13]=(float)p.y(); e.transform[14]=(float)p.z(); e.transform[15]=1;
                break;
            }
            case gviz_ipc::VarType::Point2: {
                auto p = values.at<gtsam::Point2>(kv.key);
                e.position[0]=(float)p.x(); e.position[1]=(float)p.y(); e.position[2]=0;
                e.transform[0]=1; e.transform[5]=1; e.transform[10]=1;
                e.transform[12]=(float)p.x(); e.transform[13]=(float)p.y(); e.transform[15]=1;
                break;
            }
            default:
                e.transform[0]=1; e.transform[5]=1; e.transform[10]=1; e.transform[15]=1;
            }

            // Covariance
            if (covs) {
                auto it = covs->find(kv.key);
                if (it != covs->end()) {
                    e.has_covariance = 1;
                    // Row-major storage (Eigen default is col-major, so transpose)
                    Eigen::Matrix3f Cf = it->second.cast<float>();
                    for (int r = 0; r < 3; ++r)
                        for (int c = 0; c < 3; ++c)
                            e.covariance[r*3+c] = Cf(r, c);
                }
            }

            out.push_back(e);
        }
        return out;
    }

    static std::vector<gviz_ipc::GVizEdgeEntry>
    buildEdges(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values) {
        std::vector<gviz_ipc::GVizEdgeEntry> out;
        for (size_t i = 0; i < graph.size(); ++i) {
            auto& f = graph[i];
            if (!f || f->keys().size() < 2) continue;
            auto& keys = f->keys();
            for (size_t k = 0; k+1 < keys.size(); ++k) {
                gviz_ipc::GVizEdgeEntry e{};
                e.key_from = keys[k]; e.key_to = keys[k+1];
                std::string tn = typeid(*f).name();
                if      (tn.find("Prior")   != std::string::npos) e.factor_type = (uint8_t)gviz_ipc::FactorType::Prior;
                else if (tn.find("Between") != std::string::npos) e.factor_type = (uint8_t)gviz_ipc::FactorType::Between;
                else if (tn.find("Project") != std::string::npos) e.factor_type = (uint8_t)gviz_ipc::FactorType::Projection;
                else                                               e.factor_type = (uint8_t)gviz_ipc::FactorType::Custom;
                try { if (!values.empty()) e.error = (float)f->error(values); } catch(...) {}
                out.push_back(e);
            }
        }
        return out;
    }

    // ── Socket I/O ────────────────────────────────────────────────────────────

    bool transmit(gviz_ipc::MsgType type,
                  std::vector<gviz_ipc::GVizVarEntry>   vars,
                  std::vector<gviz_ipc::GVizEdgeEntry>  edges,
                  const std::string&                    label,
                  std::vector<CloudPayload>             clouds,
                  std::vector<gviz_ipc::GVizPrimEntry>  prims) {

        std::lock_guard<std::mutex> g(mtx_);
        if (fd_ < 0 && !reconnect()) return false;

        uint16_t label_len = (uint16_t)std::min(label.size(), (size_t)65535u);

        // Compute total payload size
        size_t cloud_bytes = 0;
        for (auto& cp : clouds) {
            cloud_bytes += sizeof(gviz_ipc::GVizCloudHeader);
            cloud_bytes += cp.positions.size() * 3 * sizeof(float);
            cloud_bytes += cp.colors.size()    * 4;
        }

        size_t total = sizeof(gviz_ipc::GVizMsgHeader)
                     + label_len
                     + vars.size()   * sizeof(gviz_ipc::GVizVarEntry)
                     + edges.size()  * sizeof(gviz_ipc::GVizEdgeEntry)
                     + cloud_bytes
                     + prims.size()  * sizeof(gviz_ipc::GVizPrimEntry);

        buf_.resize(total);
        char* p = buf_.data();

        // Header
        gviz_ipc::GVizMsgHeader hdr{};
        hdr.magic        = gviz_ipc::MAGIC_V2;
        hdr.type         = static_cast<uint8_t>(type);
        hdr.version      = gviz_ipc::PROTOCOL_VERSION;
        hdr.seq_id       = ++seq_;
        hdr.n_vars       = (uint32_t)vars.size();
        hdr.n_edges      = (uint32_t)edges.size();
        hdr.n_primitives = (uint32_t)prims.size();
        hdr.n_clouds     = (uint16_t)clouds.size();
        hdr.label_len    = label_len;
        std::memcpy(p, &hdr, sizeof(hdr)); p += sizeof(hdr);

        // Label
        std::memcpy(p, label.data(), label_len); p += label_len;

        // Vars
        if (!vars.empty()) {
            size_t sz = vars.size() * sizeof(gviz_ipc::GVizVarEntry);
            std::memcpy(p, vars.data(), sz); p += sz;
        }
        // Edges
        if (!edges.empty()) {
            size_t sz = edges.size() * sizeof(gviz_ipc::GVizEdgeEntry);
            std::memcpy(p, edges.data(), sz); p += sz;
        }
        // Clouds
        for (auto& cp : clouds) {
            gviz_ipc::GVizCloudHeader ch{};
            ch.n_points   = (uint32_t)cp.positions.size();
            ch.has_colors = cp.colors.empty() ? 0 : 1;
            ch.point_size = cp.point_size;
            ch.default_color[0] = cp.default_color[0]; ch.default_color[1] = cp.default_color[1];
            ch.default_color[2] = cp.default_color[2]; ch.default_color[3] = cp.default_color[3];
            std::memcpy(p, &ch, sizeof(ch)); p += sizeof(ch);

            size_t pos_sz = cp.positions.size() * 3 * sizeof(float);
            std::memcpy(p, cp.positions.data(), pos_sz); p += pos_sz;

            if (ch.has_colors) {
                size_t col_sz = cp.colors.size() * 4;
                std::memcpy(p, cp.colors.data(), col_sz); p += col_sz;
            }
        }
        // Primitives
        if (!prims.empty()) {
            size_t sz = prims.size() * sizeof(gviz_ipc::GVizPrimEntry);
            std::memcpy(p, prims.data(), sz);
        }

        ssize_t sent = ::send(fd_, buf_.data(), total, MSG_NOSIGNAL);
        if (sent < 0 || (size_t)sent != total) {
            ::close(fd_); fd_ = -1;
            return false;
        }
        return true;
    }

    bool reconnect() {
        fd_ = ::socket(AF_UNIX, SOCK_STREAM, 0);
        if (fd_ < 0) return false;
        struct sockaddr_un addr{};
        addr.sun_family = AF_UNIX;
        std::strncpy(addr.sun_path, socketPath_.c_str(), sizeof(addr.sun_path)-1);
        if (::connect(fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            ::close(fd_); fd_ = -1; return false;
        }
        return true;
    }

    mutable std::mutex  mtx_;
    int                 fd_  = -1;
    uint32_t            seq_ = 0;
    std::string         socketPath_;
    std::vector<char>   buf_;
};

// ── SceneBuilder::send implementation ─────────────────────────────────────────
inline bool SceneBuilder::send(GVizClient& client, const std::string& label) {
    return client.sendScene(*this, label);
}
