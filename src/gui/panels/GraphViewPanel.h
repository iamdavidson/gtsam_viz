#pragma once
#include <imgui.h>
#include "../../graph/FactorGraphState.h"
#include "../../graph/GraphLayout.h"
#include "../../graph/ResidualColorScale.h"
#include <optional>

namespace gtsam_viz {

class GraphViewPanel {
public:
    explicit GraphViewPanel(FactorGraphState& state);

    void draw();

    // Selection (shared with other panels)
    std::optional<gtsam::Key>  selectedVariable;
    std::optional<size_t>      selectedFactor;

private:
    void drawToolbar();
    void drawCanvas();
    void drawNode(ImDrawList* dl, const VariableNode& vn,
                  ImVec2 origin, float zoom);
    void drawFactor(ImDrawList* dl, const FactorNode& fn,
                    ImVec2 origin, float zoom);
    void drawEdge(ImDrawList* dl, ImVec2 a, ImVec2 b, ImU32 col, float thickness=1.5f);
    void handleCanvasInteraction(ImVec2 canvasPos, ImVec2 canvasSize);

    ImVec2 worldToScreen(glm::vec2 world, ImVec2 origin) const;
    glm::vec2 screenToWorld(ImVec2 screen, ImVec2 origin) const;

    // Error → color mapping (green → yellow → red)
    ImU32 errorColor(const FactorNode& fn) const;
    ImU32 typeColor(VariableType t) const;

    FactorGraphState& state_;
    GraphLayout       layout_;

    // Canvas state
    ImVec2    pan_      = {0, 0};
    float     zoom_     = 1.f;
    bool      dragging_ = false;
    bool      nodeDrag_ = false;
    int       dragVarIdx_    = -1;
    int       dragFactorIdx_ = -1;
    ImVec2    lastMousePos_  = {0,0};

    // Options
    bool showFactorNodes_  = true;
    bool showLabels_       = true;
    bool showErrorColors_  = true;
    bool layoutRunning_    = false;
    bool showGrid_         = true;

    float nodeRadius_   = 22.f;
    float factorSize_   = 14.f;

    ResidualStats residualStats_;
};

} // namespace gtsam_viz
