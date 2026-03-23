#pragma once
#include <imgui.h>
#include "../../renderer/Renderer3D.h"
#include "../../graph/FactorGraphState.h"

namespace gtsam_viz {

class Viewport3DPanel {
public:
    Viewport3DPanel(FactorGraphState& state, Renderer3D& renderer);
    void draw();

private:
    void drawToolbar();
    void handleCameraInput(ImVec2 imagePos, ImVec2 imageSize);

    FactorGraphState& state_;
    Renderer3D&       renderer_;

    bool orbiting_ = false;
    bool panning_  = false;

    // Grid rebuild request
    int   pendingGridHalf_ = -1;
    float pendingGridStep_ = -1.f;
};

} // namespace gtsam_viz
