#pragma once
#include <imgui.h>
#include "../graph/FactorGraphState.h"
#include "../renderer/Renderer3D.h"
#include "panels/OptimizerPanel.h"
#include "panels/LogPanel.h"
#include "panels/Viewport3DPanel.h"
#include "panels/GraphViewPanel.h"
#include "panels/InspectorPanel.h"
#include "../bridge/GraphBridge.h"
#include "../ipc/GVizServer.h"
#include <memory>
#include <optional>

namespace gtsam_viz {

class GuiManager {
public:
    GuiManager();
    ~GuiManager() = default;

    bool init(Renderer3D& renderer, GVizServer& server, float uiScale = 1.f);
    void draw();

    FactorGraphState& graphState() { return state_; }

private:
    void drawMainMenuBar();
    void drawDockspace();
    void drawAboutWindow();
    void drawSettingsWindow();
    void applyDarkTheme(float uiScale);
    void saveSettings();

    FactorGraphState state_;

    std::unique_ptr<OptimizerPanel>   optimizerPanel_;
    std::unique_ptr<LogPanel>         logPanel_;
    std::unique_ptr<Viewport3DPanel>  viewport3DPanel_;
    std::unique_ptr<GraphViewPanel>   graphViewPanel_;
    std::unique_ptr<InspectorPanel>   inspectorPanel_;
    std::unique_ptr<BridgePanel>      bridgePanel_;
    Renderer3D*                       renderer_ = nullptr;
    GVizServer*                       server_ = nullptr;

    std::optional<gtsam::Key>         selectedVariable_;
    std::optional<size_t>             selectedFactor_;

    bool showAbout_      = false;
    bool showImGuiDemo_  = false;
    bool showImPlotDemo_ = false;

    ImGuiID dockspaceId_ = 0;
    float   uiScale_     = 1.f;
    bool    firstFrame_  = true;
};

} // namespace gtsam_viz
