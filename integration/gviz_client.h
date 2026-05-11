#pragma once
/**
 * gviz_client.h  –  Standalone header-only IPC client for GTSAMViz  (Protocol v2)
 * ==================================================================================
 * Copy THIS SINGLE FILE into your SLAM project — no other GTSAMViz headers needed.
 * Dependencies: GTSAM, Eigen (already a GTSAM dep), POSIX sockets (Linux/macOS).
 *
 * ── BASIC USAGE ──────────────────────────────────────────────────────────────────
 *
 *   GVizClient viz;
 *   viz.connect();                                         // connect to running GUI
 *   viz.publish(graph, values, "keyframe 42");             // full replace
 *   viz.publishValuesWithErrors(graph, optimized_values, "post-opt");
 *   viz.append(new_factors, new_values, "step N");         // incremental
 *   viz.clear();
 *   viz.disconnect();   // or just let it destruct
 *
 *   If the GUI is not running connect() is a no-op and all publish calls silently
 *   do nothing — zero overhead on your SLAM thread.
 *
 * ── COVARIANCE ELLIPSOIDS ────────────────────────────────────────────────────────
 *
 *   GVizClient::CovarianceMap covs;
 *   gtsam::Marginals marginals(graph, values);
 *   for (auto key : values.keys()) {
 *       // For Pose3: block<3,3>(3,3) = XYZ sub-block of the 6×6 marginal
 *       covs[key] = marginals.marginalCovariance(key).block<3,3>(3,3);
 *   }
 *   viz.publish(graph, values, "kf42", &covs);
 *
 * ── POINT CLOUDS ─────────────────────────────────────────────────────────────────
 *
 *   std::vector<Eigen::Vector3f> pts = { ... };
 *   viz.publishPointCloud(pts, "lidar scan");                 // solid white
 *   viz.publishPointCloud(pts, "lidar scan", 4.f,
 *                         Eigen::Vector4f(1,0.5,0,1));        // solid orange, 4px
 *
 *   // Per-point RGBA colors [0,1]:
 *   std::vector<Eigen::Vector4f> colors = { ... };
 *   viz.publishPointCloud(pts, colors, "colored scan");
 *
 * ── GEOMETRY PRIMITIVES (SceneBuilder) ───────────────────────────────────────────
 *
 *   viz.scene()
 *      .line({0,0,0}, {1,0,0}, {1,1,0,1})                 // yellow line
 *      .arrow({0,0,0}, {0,1,0}, {0,1,0,1})                // green arrow
 *      .box({2,0,0}, {0.5f,0.5f,0.5f}, {0,0,1,1})         // blue box
 *      .sphere({4,0,0}, 0.3f, {1,0,0,1})                  // red sphere
 *      .cone({0,1,0}, {0,0,0}, 0.5f, {1,0.5f,0,1})        // orange cone
 *      .cylinder({0,0,0}, {0,0,1}, 0.2f, 1.f, {0,1,1,1}) // cyan cylinder
 *      .coordFrame(pose4x4, 0.5f)                          // coordinate frame
 *      .send(viz, "obstacles");
 *
 * ── THREAD-SAFETY ────────────────────────────────────────────────────────────────
 *   All methods are thread-safe (single mutex protects the socket).
 *   publish() blocks at most ~1 ms (kernel loopback buffer).
 *
 * ── SOCKET PATH ──────────────────────────────────────────────────────────────────
 *   Default: /tmp/gtsam_viz.sock  (same default as the GTSAMViz GUI)
 */

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
#include <cstdint>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <array>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

// ══════════════════════════════════════════════════════════════════════════════
//  Protocol v2  (embedded — no separate GVizProtocol.h needed)
// ══════════════════════════════════════════════════════════════════════════════
namespace gviz_ipc {

constexpr uint32_t MAGIC_V2         = 0x325A5647u;  // "GVZ2"
constexpr char     SOCKET_PATH[]    = "/tmp/gtsam_viz.sock";
constexpr uint8_t  PROTOCOL_VERSION = 2;

enum class MsgType   : uint8_t { Replace=0, Append=1, ValuesOnly=2, Clear=3, PointCloud=4, Primitives=5 };
enum class VarType   : uint8_t { Pose2=0, Pose3=1, Point2=2, Point3=3, Unknown=4 };
enum class FactorType: uint8_t { Prior=0, Between=1, Projection=2, Custom=3, BearingRange=4 };
enum class PrimType  : uint8_t { Line=0, Arrow=1, Box=2, Sphere=3, Cone=4, Cylinder=5, CoordFrame=6 };

#pragma pack(push, 1)

// Wire layout per message:
//   GVizMsgHeader   26 B  (fixed)
//   char[label_len]        UTF-8, no null terminator
//   GVizVarEntry[]  122 B × n_vars
//   GVizEdgeEntry[]  21 B × n_edges
//   ── repeated n_clouds times ──────────────────────────
//   GVizCloudHeader  28 B
//   float[3 × n_points]         XYZ positions
//   uint8_t[4 × n_points]       RGBA8 per-point colors (only if has_colors=1)
//   ─────────────────────────────────────────────────────
//   GVizPrimEntry[]  84 B × n_primitives

struct GVizMsgHeader {
    uint32_t magic;        ///< MAGIC_V2
    uint8_t  type;         ///< MsgType
    uint8_t  version;      ///< PROTOCOL_VERSION
    uint32_t seq_id;
    uint32_t n_vars;
    uint32_t n_edges;
    uint32_t n_primitives;
    uint16_t n_clouds;
    uint16_t label_len;
};
static_assert(sizeof(GVizMsgHeader) == 26, "");

struct GVizVarEntry {
    uint64_t key;
    float    transform[16];  ///< Column-major 4×4 world pose
    float    position[3];    ///< World XYZ (convenience copy of transform col 3)
    uint8_t  var_type;       ///< VarType
    uint8_t  has_covariance; ///< 1 → covariance[] valid
    float    covariance[9];  ///< Row-major 3×3 position covariance (world space)
};
static_assert(sizeof(GVizVarEntry) == 122, "");

struct GVizEdgeEntry {
    uint64_t key_from;
    uint64_t key_to;
    uint8_t  factor_type; ///< FactorType
    float    error;
};
static_assert(sizeof(GVizEdgeEntry) == 21, "");

struct GVizCloudHeader {
    uint32_t n_points;
    uint8_t  has_colors;       ///< 1 = per-point RGBA8 follows positions
    uint8_t  pad[3];
    float    default_color[4]; ///< RGBA [0,1] when has_colors=0
    float    point_size;       ///< Render diameter in pixels
};
static_assert(sizeof(GVizCloudHeader) == 28, "");

// PrimType data[] layout:
//   Line:       data[0..2]=from, data[3..5]=to, data[6]=width
//   Arrow:      data[0..2]=from, data[3..5]=to
//   Box:        data[0..2]=center, data[3..5]=half_extents, data[6..14]=rot3x3 row-major
//   Sphere:     data[0..2]=center, data[3]=radius
//   Cone:       data[0..2]=apex, data[3..5]=base_center, data[6]=base_radius
//   Cylinder:   data[0..2]=center, data[3..5]=axis, data[6]=radius, data[7]=half_height
//   CoordFrame: data[0..2]=pos, data[3]=axis_length, data[4..12]=rot3x3 row-major
struct GVizPrimEntry {
    uint8_t prim_type;
    uint8_t pad[3];
    float   color[4];   ///< RGBA [0,1]
    float   data[16];
};
static_assert(sizeof(GVizPrimEntry) == 84, "");

#pragma pack(pop)
} // namespace gviz_ipc


// ══════════════════════════════════════════════════════════════════════════════
//  Forward declaration
// ══════════════════════════════════════════════════════════════════════════════
class GVizClient;


// ══════════════════════════════════════════════════════════════════════════════
//  SceneBuilder  –  Fluent builder for 3-D primitives
// ══════════════════════════════════════════════════════════════════════════════
class SceneBuilder {
public:
    SceneBuilder() = default;

    /// Line segment between two points
    SceneBuilder& line(const Eigen::Vector3f& from, const Eigen::Vector3f& to,
                       const Eigen::Vector4f& color = {1,1,1,1},
                       float width = 1.f) {
        auto& e = add(gviz_ipc::PrimType::Line, color);
        v3(e.data, 0, from); v3(e.data, 3, to); e.data[6] = width;
        return *this;
    }

    /// Arrow with automatic cone head (20 % of shaft length)
    SceneBuilder& arrow(const Eigen::Vector3f& from, const Eigen::Vector3f& to,
                        const Eigen::Vector4f& color = {1,1,1,1}) {
        auto& e = add(gviz_ipc::PrimType::Arrow, color);
        v3(e.data, 0, from); v3(e.data, 3, to);
        return *this;
    }

    /// Solid box — optionally rotated
    SceneBuilder& box(const Eigen::Vector3f& center,
                      const Eigen::Vector3f& half_extents,
                      const Eigen::Vector4f& color    = {1,1,1,1},
                      const Eigen::Matrix3f& rotation = Eigen::Matrix3f::Identity()) {
        auto& e = add(gviz_ipc::PrimType::Box, color);
        v3(e.data, 0, center); v3(e.data, 3, half_extents);
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                e.data[6 + r*3 + c] = rotation(r, c);
        return *this;
    }

    /// Solid sphere
    SceneBuilder& sphere(const Eigen::Vector3f& center, float radius,
                         const Eigen::Vector4f& color = {1,1,1,1}) {
        auto& e = add(gviz_ipc::PrimType::Sphere, color);
        v3(e.data, 0, center); e.data[3] = radius;
        return *this;
    }

    /// Solid cone — apex to base center
    SceneBuilder& cone(const Eigen::Vector3f& apex,
                       const Eigen::Vector3f& base_center, float base_radius,
                       const Eigen::Vector4f& color = {1,1,1,1}) {
        auto& e = add(gviz_ipc::PrimType::Cone, color);
        v3(e.data, 0, apex); v3(e.data, 3, base_center); e.data[6] = base_radius;
        return *this;
    }

    /// Solid cylinder
    SceneBuilder& cylinder(const Eigen::Vector3f& center,
                           const Eigen::Vector3f& axis,
                           float radius, float half_height,
                           const Eigen::Vector4f& color = {1,1,1,1}) {
        auto& e = add(gviz_ipc::PrimType::Cylinder, color);
        v3(e.data, 0, center); v3(e.data, 3, axis);
        e.data[6] = radius; e.data[7] = half_height;
        return *this;
    }

    /// Coordinate frame from a 4×4 column-major transform (OpenGL convention)
    SceneBuilder& coordFrame(const Eigen::Matrix4f& T, float axis_length = 0.5f) {
        return coordFrame(T.block<3,1>(0,3), T.block<3,3>(0,0), axis_length);
    }

    /// Coordinate frame from position + rotation
    SceneBuilder& coordFrame(const Eigen::Vector3f& pos,
                              const Eigen::Matrix3f& rot,
                              float axis_length = 0.5f) {
        auto& e = add(gviz_ipc::PrimType::CoordFrame, {1,1,1,1});
        v3(e.data, 0, pos); e.data[3] = axis_length;
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                e.data[4 + r*3 + c] = rot(r, c);
        return *this;
    }

    bool   empty() const { return prims_.empty(); }
    void   clear()       { prims_.clear(); }
    size_t size()  const { return prims_.size(); }

    /// Transmit all queued primitives to the viewer (replaces current primitive layer).
    bool send(GVizClient& client, const std::string& label = "");

private:
    std::vector<gviz_ipc::GVizPrimEntry> prims_;

    gviz_ipc::GVizPrimEntry& add(gviz_ipc::PrimType type,
                                  const Eigen::Vector4f& color) {
        prims_.emplace_back();
        auto& e = prims_.back();
        std::memset(&e, 0, sizeof(e));
        e.prim_type = static_cast<uint8_t>(type);
        e.color[0] = color[0]; e.color[1] = color[1];
        e.color[2] = color[2]; e.color[3] = color[3];
        return e;
    }

    static void v3(float* d, int off, const Eigen::Vector3f& v) {
        d[off]=v[0]; d[off+1]=v[1]; d[off+2]=v[2];
    }

    friend class GVizClient;
};


// ══════════════════════════════════════════════════════════════════════════════
//  GVizClient
// ══════════════════════════════════════════════════════════════════════════════
class GVizClient {
public:
    /// Map from variable key → 3×3 position covariance in world space.
    /// For Pose3 use:  marginals.marginalCovariance(key).block<3,3>(3,3)
    /// For Point3 use: marginals.marginalCovariance(key)  (already 3×3)
    using CovarianceMap = std::unordered_map<gtsam::Key, Eigen::Matrix3d>;

    GVizClient()  = default;
    ~GVizClient() { disconnect(); }
    GVizClient(const GVizClient&)            = delete;
    GVizClient& operator=(const GVizClient&) = delete;

    // ── Connection ────────────────────────────────────────────────────────────

    /// Connect to a running GTSAMViz GUI.  Safe to call when GUI is not yet
    /// started — returns false silently; all publish calls will be no-ops.
    bool connect(const char* socket_path = gviz_ipc::SOCKET_PATH) {
        std::lock_guard<std::mutex> g(mtx_);
        if (fd_ >= 0) return true;
        socketPath_ = socket_path;
        fd_ = ::socket(AF_UNIX, SOCK_STREAM, 0);
        if (fd_ < 0) { fd_ = -1; return false; }
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
        std::lock_guard<std::mutex> g(mtx_);
        return fd_ >= 0;
    }

    // ── Graph API ─────────────────────────────────────────────────────────────

    /// Full graph replacement.  Pass covariances to show uncertainty ellipsoids.
    bool publish(const gtsam::NonlinearFactorGraph& graph,
                 const gtsam::Values&               values,
                 const std::string&                 label = "",
                 const CovarianceMap*               covs  = nullptr) {
        return transmit(gviz_ipc::MsgType::Replace,
                        buildVars(values, covs), buildEdges(graph, values),
                        label, {}, {});
    }

    /// Position-only update — topology (edges) unchanged.
    /// Use after every optimization iteration; very fast.
    /// Residual colors become stale until errors are sent again.
    bool publishValuesOnly(const gtsam::Values& values,
                           const std::string&   label = "") {
        return transmit(gviz_ipc::MsgType::ValuesOnly,
                        buildVars(values, nullptr), {}, label, {}, {});
    }

    /// Position update plus freshly computed factor errors; topology unchanged.
    bool publishValuesWithErrors(const gtsam::NonlinearFactorGraph& graph,
                                 const gtsam::Values&               values,
                                 const std::string&                 label = "") {
        return transmit(gviz_ipc::MsgType::ValuesOnly,
                        buildVars(values, nullptr),
                        buildEdges(graph, values), label, {}, {});
    }

    /// Append new variables and factors (iSAM2-style incremental update).
    bool append(const gtsam::NonlinearFactorGraph& new_factors,
                const gtsam::Values&               new_values,
                const std::string&                 label = "") {
        return transmit(gviz_ipc::MsgType::Append,
                        buildVars(new_values, nullptr),
                        buildEdges(new_factors, new_values), label, {}, {});
    }

    /// Clear everything (graph + point clouds + primitives).
    bool clear() {
        return transmit(gviz_ipc::MsgType::Clear, {}, {}, "", {}, {});
    }

    // ── Point cloud API ───────────────────────────────────────────────────────

    /// Publish a solid-color point cloud (replaces previous cloud layer).
    bool publishPointCloud(const std::vector<Eigen::Vector3f>& pts,
                           const std::string&  label      = "",
                           float               point_size = 3.f,
                           const Eigen::Vector4f& color   = {1,1,1,1}) {
        CloudPayload cp;
        cp.point_size    = point_size;
        cp.default_color = { color[0], color[1], color[2], color[3] };
        cp.positions.reserve(pts.size());
        for (auto& p : pts) cp.positions.push_back({ p[0], p[1], p[2] });
        return transmit(gviz_ipc::MsgType::PointCloud, {}, {}, label,
                        { std::move(cp) }, {});
    }

    /// Publish a per-point RGBA colored point cloud.  colors[i] is RGBA [0,1].
    bool publishPointCloud(const std::vector<Eigen::Vector3f>& pts,
                           const std::vector<Eigen::Vector4f>& colors,
                           const std::string& label      = "",
                           float              point_size = 3.f) {
        CloudPayload cp;
        cp.point_size    = point_size;
        cp.default_color = {1,1,1,1};
        cp.positions.reserve(pts.size());
        for (auto& p : pts) cp.positions.push_back({ p[0], p[1], p[2] });
        size_t n = std::min(pts.size(), colors.size());
        cp.colors.reserve(n);
        for (size_t i = 0; i < n; ++i) {
            auto& c = colors[i];
            auto clamp = [](float v){ return static_cast<uint8_t>(
                              std::min(1.f, std::max(0.f, v)) * 255.f); };
            cp.colors.push_back({ clamp(c[0]), clamp(c[1]), clamp(c[2]), clamp(c[3]) });
        }
        return transmit(gviz_ipc::MsgType::PointCloud, {}, {}, label,
                        { std::move(cp) }, {});
    }

    // ── Scene builder ─────────────────────────────────────────────────────────

    /// Returns a fresh SceneBuilder.  Call .send(*this, label) when done.
    SceneBuilder scene() { return SceneBuilder{}; }

    /// Called by SceneBuilder::send() — usually not invoked directly.
    bool sendScene(SceneBuilder& b, const std::string& label) {
        return transmit(gviz_ipc::MsgType::Primitives, {}, {}, label,
                        {}, b.prims_);
    }

private:
    // ── Internal types ────────────────────────────────────────────────────────
    struct CloudPayload {
        std::vector<std::array<float,   3>>  positions;
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
                e.transform[0] =(float)R(0,0); e.transform[1] =(float)R(1,0); e.transform[2] =(float)R(2,0); e.transform[3] =0;
                e.transform[4] =(float)R(0,1); e.transform[5] =(float)R(1,1); e.transform[6] =(float)R(2,1); e.transform[7] =0;
                e.transform[8] =(float)R(0,2); e.transform[9] =(float)R(1,2); e.transform[10]=(float)R(2,2); e.transform[11]=0;
                e.transform[12]=(float)t.x();  e.transform[13]=(float)t.y();  e.transform[14]=(float)t.z();  e.transform[15]=1;
                break;
            }
            case gviz_ipc::VarType::Pose2: {
                auto p = values.at<gtsam::Pose2>(kv.key);
                float c=std::cos((float)p.theta()), s=std::sin((float)p.theta());
                e.position[0]=(float)p.x(); e.position[1]=(float)p.y(); e.position[2]=0;
                e.transform[0]=c;  e.transform[1]=s;  e.transform[2]=0; e.transform[3]=0;
                e.transform[4]=-s; e.transform[5]=c;  e.transform[6]=0; e.transform[7]=0;
                e.transform[8]=0;  e.transform[9]=0;  e.transform[10]=1;e.transform[11]=0;
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
                e.position[0]=(float)p.x(); e.position[1]=(float)p.y();
                e.transform[0]=1; e.transform[5]=1; e.transform[10]=1;
                e.transform[12]=(float)p.x(); e.transform[13]=(float)p.y(); e.transform[15]=1;
                break;
            }
            default:
                e.transform[0]=1; e.transform[5]=1; e.transform[10]=1; e.transform[15]=1;
            }

            if (covs) {
                auto it = covs->find(kv.key);
                if (it != covs->end()) {
                    e.has_covariance = 1;
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
            if (!f || f->keys().size() < 1) continue;
            auto& keys = f->keys();
            std::string tn = typeid(*f).name();
            uint8_t ft;
            if      (tn.find("Prior")   != std::string::npos) ft = (uint8_t)gviz_ipc::FactorType::Prior;
            else if (tn.find("Between") != std::string::npos) ft = (uint8_t)gviz_ipc::FactorType::Between;
            else if (tn.find("BearingRange") != std::string::npos) ft = (uint8_t)gviz_ipc::FactorType::BearingRange;
            else if (tn.find("Project") != std::string::npos) ft = (uint8_t)gviz_ipc::FactorType::Projection;
            else                                               ft = (uint8_t)gviz_ipc::FactorType::Custom;
            float err = 0.f;
            try { if (!values.empty()) err = (float)f->error(values); } catch(...) {}
            if (keys.size() == 1) {
                out.push_back({ keys[0], keys[0], ft, err });  // unary → self-loop
            } else {
                for (size_t k = 0; k+1 < keys.size(); ++k)
                    out.push_back({ keys[k], keys[k+1], ft, err });
            }
        }
        return out;
    }

    // ── Socket I/O ────────────────────────────────────────────────────────────

    bool transmit(gviz_ipc::MsgType                      type,
                  std::vector<gviz_ipc::GVizVarEntry>    vars,
                  std::vector<gviz_ipc::GVizEdgeEntry>   edges,
                  const std::string&                      label,
                  std::vector<CloudPayload>               clouds,
                  std::vector<gviz_ipc::GVizPrimEntry>   prims) {

        std::lock_guard<std::mutex> g(mtx_);
        if (fd_ < 0 && !reconnect()) return false;

        uint16_t llen = (uint16_t)std::min(label.size(), (size_t)65535u);

        size_t cloud_bytes = 0;
        for (auto& cp : clouds) {
            cloud_bytes += sizeof(gviz_ipc::GVizCloudHeader);
            cloud_bytes += cp.positions.size() * 3 * sizeof(float);
            cloud_bytes += cp.colors.size()    * 4;
        }

        size_t total = sizeof(gviz_ipc::GVizMsgHeader)
                     + llen
                     + vars.size()  * sizeof(gviz_ipc::GVizVarEntry)
                     + edges.size() * sizeof(gviz_ipc::GVizEdgeEntry)
                     + cloud_bytes
                     + prims.size() * sizeof(gviz_ipc::GVizPrimEntry);

        buf_.resize(total);
        char* p = buf_.data();

        gviz_ipc::GVizMsgHeader hdr{};
        hdr.magic        = gviz_ipc::MAGIC_V2;
        hdr.type         = static_cast<uint8_t>(type);
        hdr.version      = gviz_ipc::PROTOCOL_VERSION;
        hdr.seq_id       = ++seq_;
        hdr.n_vars       = (uint32_t)vars.size();
        hdr.n_edges      = (uint32_t)edges.size();
        hdr.n_primitives = (uint32_t)prims.size();
        hdr.n_clouds     = (uint16_t)clouds.size();
        hdr.label_len    = llen;
        std::memcpy(p, &hdr, sizeof(hdr));  p += sizeof(hdr);
        std::memcpy(p, label.data(), llen); p += llen;

        if (!vars.empty()) {
            size_t sz = vars.size() * sizeof(gviz_ipc::GVizVarEntry);
            std::memcpy(p, vars.data(), sz); p += sz;
        }
        if (!edges.empty()) {
            size_t sz = edges.size() * sizeof(gviz_ipc::GVizEdgeEntry);
            std::memcpy(p, edges.data(), sz); p += sz;
        }
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
        if (!prims.empty()) {
            size_t sz = prims.size() * sizeof(gviz_ipc::GVizPrimEntry);
            std::memcpy(p, prims.data(), sz);
        }

        ssize_t sent = ::send(fd_, buf_.data(), total, MSG_NOSIGNAL);
        if (sent < 0 || (size_t)sent != total) {
            ::close(fd_); fd_ = -1; return false;
        }
        return true;
    }

    bool reconnect() {
        fd_ = ::socket(AF_UNIX, SOCK_STREAM, 0);
        if (fd_ < 0) { fd_ = -1; return false; }
        struct sockaddr_un addr{};
        addr.sun_family = AF_UNIX;
        std::strncpy(addr.sun_path, socketPath_.c_str(), sizeof(addr.sun_path)-1);
        if (::connect(fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            ::close(fd_); fd_ = -1; return false;
        }
        return true;
    }

    mutable std::mutex mtx_;
    int                fd_  = -1;
    uint32_t           seq_ = 0;
    std::string        socketPath_;
    std::vector<char>  buf_;
};

// SceneBuilder::send needs GVizClient to be complete
inline bool SceneBuilder::send(GVizClient& client, const std::string& label) {
    return client.sendScene(*this, label);
}
