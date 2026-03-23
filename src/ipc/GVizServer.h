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

namespace gtsam_viz {

/// Decoded frame ready for consumption by the GUI thread
struct GVizFrame {
    gviz_ipc::MsgType              type;
    uint32_t                       seq_id;
    std::string                    label;
    std::vector<gviz_ipc::GVizVarEntry>  vars;
    std::vector<gviz_ipc::GVizEdgeEntry> edges;
};

class GVizServer {
public:
    GVizServer();
    ~GVizServer();

    /// Start listening on the Unix Domain Socket.
    /// Returns false if the socket could not be created.
    bool start();

    /// Stop the server and clean up the socket file.
    void stop();

    bool isRunning()   const { return running_.load(); }
    bool isConnected() const { return clientConnected_.load(); }

    /// Call once per GUI frame (main thread only).
    /// Applies the latest pending frame to `state` and returns true if anything changed.
    bool poll(FactorGraphState& state);

private:
    void acceptLoop();
    void recvLoop(int clientFd);
    bool recvExact(int fd, void* buf, size_t n);

    void applyFrame(const GVizFrame& frame, FactorGraphState& state);

    int              serverFd_  = -1;
    std::atomic<bool> running_{false};
    std::atomic<bool> clientConnected_{false};

    std::thread      acceptThread_;

    // Double buffer: recvLoop writes pending_, poll() swaps it
    mutable std::mutex           mtx_;
    std::optional<GVizFrame>     pending_;
    bool                         hasNew_  = false;
    uint32_t                     lastSeq_ = 0;
};

} // namespace gtsam_viz
