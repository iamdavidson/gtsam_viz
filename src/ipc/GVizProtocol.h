#pragma once
/**
 * GVizProtocol.h  –  Shared binary frame format for gtsam_viz IPC
 * ================================================================
 * Plain C POD structs — zero external dependencies.
 * Included by both the server (gtsam_viz) and the client (SLAM backend).
 *
 * Unix Domain Socket path: /tmp/gtsam_viz.sock
 *
 * Wire format per message:
 *   GVizMsgHeader   (fixed 18 bytes)
 *   char[label_len] (variable, UTF-8)
 *   GVizVarEntry[]  (n_vars × 93 bytes)
 *   GVizEdgeEntry[] (n_edges × 21 bytes)
 */

#include <cstdint>

namespace gviz_ipc {

constexpr uint32_t MAGIC        = 0x4756495A;  // "GVIZ"
constexpr char     SOCKET_PATH[] = "/tmp/gtsam_viz.sock";

/// Message type
enum class MsgType : uint8_t {
    Replace    = 0,  ///< Full graph replace
    Append     = 1,  ///< Add new vars/edges to existing graph
    ValuesOnly = 2,  ///< Update positions only, keep topology
    Clear      = 3,  ///< Clear everything
};

/// Variable type (mirrors gtsam_viz::VariableType)
enum class VarType : uint8_t {
    Pose2   = 0,
    Pose3   = 1,
    Point2  = 2,
    Point3  = 3,
    Unknown = 4,
};

/// Factor type (mirrors gtsam_viz::FactorType)
enum class FactorType : uint8_t {
    Prior      = 0,
    Between    = 1,
    Projection = 2,
    Custom     = 3,
};

#pragma pack(push, 1)

/// Fixed-size message header (18 bytes)
struct GVizMsgHeader {
    uint32_t magic;       ///< Must equal MAGIC
    uint8_t  type;        ///< MsgType
    uint32_t seq_id;
    uint32_t n_vars;
    uint32_t n_edges;
    uint16_t label_len;   ///< Bytes of label string that follow immediately
};
static_assert(sizeof(GVizMsgHeader) == 19, "Header size mismatch");

/// Per-variable entry (93 bytes)
struct GVizVarEntry {
    uint64_t key;
    float    transform[16]; ///< Column-major 4×4 pose matrix
    float    position[3];   ///< World-space translation (convenience duplicate)
    uint8_t  var_type;      ///< VarType
};
static_assert(sizeof(GVizVarEntry) == 85, "VarEntry size mismatch");

/// Per-factor-edge entry (21 bytes)
struct GVizEdgeEntry {
    uint64_t key_from;
    uint64_t key_to;
    uint8_t  factor_type;  ///< FactorType
    float    error;
};
static_assert(sizeof(GVizEdgeEntry) == 21, "EdgeEntry size mismatch");

#pragma pack(pop)

} // namespace gviz_ipc
