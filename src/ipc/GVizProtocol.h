#pragma once
/**
 * GVizProtocol.h  –  Binary frame format for gtsam_viz IPC  (Protocol v2)
 * =========================================================================
 * Zero external dependencies — plain C POD structs with #pragma pack.
 * Copy this file + GVizClient.h into your SLAM project; no other deps needed.
 *
 * Wire layout per message
 * ───────────────────────
 *   GVizMsgHeader    26 bytes  (fixed)
 *   char[label_len]            (UTF-8, no null terminator)
 *   GVizVarEntry[]   122 bytes × n_vars
 *   GVizEdgeEntry[]   21 bytes × n_edges
 *   ── repeated n_clouds times ──────────────────────────────────────────
 *   GVizCloudHeader  28 bytes
 *   float[3 × n_points]        (XYZ positions, tightly packed)
 *   uint8_t[4 × n_points]      (RGBA8 per-point colors, only if has_colors=1)
 *   ─────────────────────────────────────────────────────────────────────
 *   GVizPrimEntry[]   84 bytes × n_primitives
 *
 * MsgType semantics
 * ─────────────────
 *   Replace    – clear graph (vars+edges); apply new vars/edges/clouds/prims
 *   Append     – add vars/edges/clouds/prims to existing state
 *   ValuesOnly – update var positions only; topology unchanged
 *   Clear      – wipe everything (graph + clouds + primitives)
 *   PointCloud – replace cloud layers only; graph untouched
 *   Primitives – replace primitive layer only; graph untouched
 *
 * PrimType data[] layout
 * ──────────────────────
 *   Line:       [from.xyz(3), to.xyz(3), width(1)]
 *   Arrow:      [from.xyz(3), to.xyz(3)]
 *   Box:        [center.xyz(3), half_extents.xyz(3), rot_row_major(9)]
 *   Sphere:     [center.xyz(3), radius(1)]
 *   Cone:       [apex.xyz(3), base_center.xyz(3), base_radius(1)]
 *   Cylinder:   [center.xyz(3), axis.xyz(3), radius(1), half_height(1)]
 *   CoordFrame: [pos.xyz(3), axis_length(1), rot_row_major(9)]
 */

#include <cstdint>

namespace gviz_ipc {

// "GVZ2" in memory order (little-endian uint32)
constexpr uint32_t MAGIC_V2    = 0x325A5647u;
constexpr char     SOCKET_PATH[] = "/tmp/gtsam_viz.sock";
constexpr uint8_t  PROTOCOL_VERSION = 2;

// ── Message type ──────────────────────────────────────────────────────────────
enum class MsgType : uint8_t {
    Replace    = 0,
    Append     = 1,
    ValuesOnly = 2,
    Clear      = 3,
    PointCloud = 4,
    Primitives = 5,
};

// ── Variable / factor type ────────────────────────────────────────────────────
enum class VarType : uint8_t {
    Pose2   = 0,
    Pose3   = 1,
    Point2  = 2,
    Point3  = 3,
    Unknown = 4,
};

enum class FactorType : uint8_t {
    Prior      = 0,
    Between    = 1,
    Projection = 2,
    Custom     = 3,
};

// ── Primitive type ────────────────────────────────────────────────────────────
enum class PrimType : uint8_t {
    Line       = 0,
    Arrow      = 1,
    Box        = 2,
    Sphere     = 3,
    Cone       = 4,
    Cylinder   = 5,
    CoordFrame = 6,
};

#pragma pack(push, 1)

// ── Message header  (26 bytes) ────────────────────────────────────────────────
struct GVizMsgHeader {
    uint32_t magic;         ///< Must equal MAGIC_V2
    uint8_t  type;          ///< MsgType
    uint8_t  version;       ///< = PROTOCOL_VERSION (2)
    uint32_t seq_id;
    uint32_t n_vars;
    uint32_t n_edges;
    uint32_t n_primitives;
    uint16_t n_clouds;
    uint16_t label_len;
};
static_assert(sizeof(GVizMsgHeader) == 26, "GVizMsgHeader size mismatch");

// ── Variable entry  (122 bytes) ───────────────────────────────────────────────
struct GVizVarEntry {
    uint64_t key;
    float    transform[16];  ///< Column-major 4×4 world pose
    float    position[3];    ///< World XYZ (convenience duplicate of transform col3)
    uint8_t  var_type;       ///< VarType
    uint8_t  has_covariance; ///< 1 → covariance[] is valid
    float    covariance[9];  ///< Row-major 3×3 position covariance in world space
};
static_assert(sizeof(GVizVarEntry) == 122, "GVizVarEntry size mismatch");

// ── Factor edge entry  (21 bytes, unchanged from v1) ──────────────────────────
struct GVizEdgeEntry {
    uint64_t key_from;
    uint64_t key_to;
    uint8_t  factor_type; ///< FactorType
    float    error;
};
static_assert(sizeof(GVizEdgeEntry) == 21, "GVizEdgeEntry size mismatch");

// ── Point cloud header  (28 bytes) ────────────────────────────────────────────
/// Followed immediately by:
///   float[3 × n_points]           – XYZ positions
///   uint8_t[4 × n_points]         – RGBA8 (only when has_colors = 1)
struct GVizCloudHeader {
    uint32_t n_points;
    uint8_t  has_colors;       ///< 1 = per-point RGBA8 follows after positions
    uint8_t  pad[3];
    float    default_color[4]; ///< RGBA [0,1] used when has_colors = 0
    float    point_size;       ///< Render point diameter in pixels
};
static_assert(sizeof(GVizCloudHeader) == 28, "GVizCloudHeader size mismatch");

// ── Primitive entry  (84 bytes) ───────────────────────────────────────────────
struct GVizPrimEntry {
    uint8_t  prim_type; ///< PrimType
    uint8_t  pad[3];
    float    color[4];  ///< RGBA [0,1]
    float    data[16];  ///< Type-specific payload (see PrimType docs above)
};
static_assert(sizeof(GVizPrimEntry) == 84, "GVizPrimEntry size mismatch");

#pragma pack(pop)

} // namespace gviz_ipc
