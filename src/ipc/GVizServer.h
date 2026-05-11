#pragma once
/**
 * GVizServer  –  Unix Domain Socket server inside the gtsam_viz process.
 *
 * Runs a background accept+recv thread.
 * GuiManager calls poll() once per frame to drain the queue and update state.
 */

#include "GVizProtocol.h"
#include "../graph/FactorGraphState.h"

#include <atomic>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>
#include <array>

namespace gtsam_viz {

// ── Decoded point cloud ready for the GUI thread ──────────────────────────────
struct DecodedCloud {
    std::vector<std::array<float, 3>>    positions;
    std::vector<std::array<uint8_t, 4>>  colors;   // empty → use default_color
    float                                point_size    = 3.f;
    std::array<float, 4>                 default_color = {1, 1, 1, 1};
};

/// Decoded frame ready for consumption by the GUI thread
struct GVizFrame {
    gviz_ipc::MsgType                        type;
    uint32_t                                 seq_id;
    std::string                              label;
    std::vector<gviz_ipc::GVizVarEntry>      vars;
    std::vector<gviz_ipc::GVizEdgeEntry>     edges;
    std::vector<DecodedCloud>                point_clouds;   // v2: decoded clouds
    std::vector<gviz_ipc::GVizPrimEntry>     primitives;     // v2: decoded prims
};

class GVizServer {
public:
    GVizServer();
    ~GVizServer();

    bool start();
    void stop();

    bool isRunning()   const { return running_.load(); }
    bool isConnected() const { return clientConnected_.load(); }

    /// Call once per GUI frame (main thread only).
    bool poll(FactorGraphState& state);

private:
    void acceptLoop();
    void recvLoop(int clientFd);
    bool recvExact(int fd, void* buf, size_t n);

    void applyFrame(const GVizFrame& frame, FactorGraphState& state);

    int               serverFd_  = -1;
    std::atomic<bool> running_{false};
    std::atomic<bool> clientConnected_{false};

    std::thread       acceptThread_;

    mutable std::mutex        mtx_;
    std::optional<GVizFrame>  pending_;
    bool                      hasNew_  = false;
};

} // namespace gtsam_viz
