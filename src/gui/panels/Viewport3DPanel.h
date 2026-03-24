#pragma once
#include <imgui.h>
#include "../../renderer/Renderer3D.h"
#include "../../graph/FactorGraphState.h"
#include <string>

namespace gtsam_viz {

/// All user-tunable sensitivity values — owned by Viewport3DPanel,
/// drawn in the Settings panel, saved/loaded via GuiManager.
struct CameraSettings {
    float orbitSensitivity    = 0.40f;   ///< LMB orbit (deg per pixel)
    float zoomScrollSensitivity = 0.10f; ///< Scroll zoom (fraction of distance)
    float zoomDragSensitivity = 0.005f;  ///< RMB drag zoom
    float panSensitivity      = 0.010f;  ///< MMB pan (fraction of distance per pixel)
    float shiftPanSensitivity = 0.002f;  ///< Shift+LMB pan (smaller than MMB)
    float wasdSensitivity     = 0.80f;   ///< WASD / arrow speed factor

    void save(const std::string& path) const;
    void load(const std::string& path);
};

class Viewport3DPanel {
public:
    Viewport3DPanel(FactorGraphState& state, Renderer3D& renderer);
    void draw();

    /// Draw the sensitivity sliders – called from the Settings panel.
    void drawSettings();

    CameraSettings& settings() { return cfg_; }

private:
    void drawToolbar();
    void handleCameraInput(ImVec2 imagePos, ImVec2 imageSize);

    FactorGraphState& state_;
    Renderer3D&       renderer_;
    CameraSettings    cfg_;

    bool orbiting_     = false;
    bool panning_      = false;
    bool shiftPanning_ = false;

    int   pendingGridHalf_ = -1;
    float pendingGridStep_ = -1.f;
};

} // namespace gtsam_viz
