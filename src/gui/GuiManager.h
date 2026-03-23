#pragma once
#include <imgui.h>
#include "../graph/FactorGraphState.h"
#include "../renderer/Renderer3D.h"
#include "panels/OptimizerPanel.h"
#include "panels/LogPanel.h"
#include "panels/Viewport3DPanel.h"
#include "../bridge/GraphBridge.h"
#include "../ipc/GVizServer.h"
#include <memory>

namespace gtsam_viz {

class GuiManager {
public:
    GuiManager();
    ~GuiManager() = default;

    bool init(Renderer3D& renderer, GVizServer& server);
    void draw();

    FactorGraphState& graphState() { return state_; }

private:
    void drawMainMenuBar();
    void drawDockspace();
    void drawAboutWindow();
    void applyDarkTheme();

    FactorGraphState state_;

    std::unique_ptr<OptimizerPanel>   optimizerPanel_;
    std::unique_ptr<LogPanel>         logPanel_;
    std::unique_ptr<Viewport3DPanel>  viewport3DPanel_;
    std::unique_ptr<BridgePanel>      bridgePanel_;
    GVizServer*                       server_ = nullptr;

    bool showAbout_      = false;
    bool showImGuiDemo_  = false;
    bool showImPlotDemo_ = false;

    ImGuiID dockspaceId_ = 0;
    GVizServer ipcServer_;
    bool    firstFrame_  = true;
};

} // namespace gtsam_viz
