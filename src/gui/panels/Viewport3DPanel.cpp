#include "Viewport3DPanel.h"
#include <imgui.h>
#include <string>
#include <cmath>
#include <fstream>
#include <sstream>
#include <algorithm>

namespace gtsam_viz {

// ── CameraSettings save / load ────────────────────────────────────────────────
void CameraSettings::save(const std::string& path) const {
    std::ofstream f(path);
    if (!f) return;
    f << "orbitSensitivity="      << orbitSensitivity      << "\n";
    f << "zoomScrollSensitivity=" << zoomScrollSensitivity  << "\n";
    f << "zoomDragSensitivity="   << zoomDragSensitivity    << "\n";
    f << "panSensitivity="        << panSensitivity         << "\n";
    f << "shiftPanSensitivity="   << shiftPanSensitivity    << "\n";
    f << "wasdSensitivity="       << wasdSensitivity        << "\n";
}

void CameraSettings::load(const std::string& path) {
    std::ifstream f(path);
    if (!f) return;
    std::string line;
    while (std::getline(f, line)) {
        auto eq = line.find('=');
        if (eq == std::string::npos) continue;
        std::string key = line.substr(0, eq);
        try {
            float val = std::stof(line.substr(eq + 1));
            if      (key == "orbitSensitivity")      orbitSensitivity      = val;
            else if (key == "zoomScrollSensitivity")  zoomScrollSensitivity = val;
            else if (key == "zoomDragSensitivity")    zoomDragSensitivity   = val;
            else if (key == "panSensitivity")         panSensitivity        = val;
            else if (key == "shiftPanSensitivity")    shiftPanSensitivity   = val;
            else if (key == "wasdSensitivity")        wasdSensitivity       = val;
        } catch (...) {}
    }
}

// ── Viewport3DPanel ───────────────────────────────────────────────────────────
Viewport3DPanel::Viewport3DPanel(FactorGraphState& state, Renderer3D& renderer)
    : state_(state), renderer_(renderer)
{
    cfg_.load("settings.cfg");
}

void Viewport3DPanel::draw() {
    drawToolbar();

    ImVec2 avail = ImGui::GetContentRegionAvail();
    int w = std::max(1, (int)avail.x);
    int h = std::max(1, (int)avail.y);

    renderer_.resize(w, h);
    GLuint tex = renderer_.render(state_);

    ImVec2 imgPos = ImGui::GetCursorScreenPos();
    ImGui::Image((ImTextureID)(intptr_t)tex, avail, {0,1}, {1,0});

    handleCameraInput(imgPos, avail);

    // Overlay: camera info + follow indicator
    auto& cam = renderer_.camera();
    char info[160];
    snprintf(info, sizeof(info),
             "Yaw %.0f°  Pitch %.0f°  Dist %.1f%s",
             cam.yaw, cam.pitch, cam.distance,
             renderer_.followMode ? "  [FOLLOW]" : "");
    ImDrawList* dl = ImGui::GetWindowDrawList();
    float uiScale = std::max(1.f, ImGui::GetFontSize() / 16.f);
    dl->AddText({imgPos.x + 8 * uiScale, imgPos.y + 8 * uiScale},
                IM_COL32(245,242,242,200), info);

    // Key hint overlay (bottom-left)
    const char* hint = "LMB: Orbit  Shift+LMB / MMB: Pan  RMB: Zoom  Scroll: Zoom  WASD/Arrows: Pan  Ctrl+S: Save";
    ImVec2 hintSz = ImGui::CalcTextSize(hint);
    dl->AddText({imgPos.x + 8 * uiScale, imgPos.y + avail.y - hintSz.y - 6 * uiScale},
                IM_COL32(160,155,155,150), hint);

    if (renderer_.colorEdgesByError && renderer_.showTrajectory) {
        const ResidualStats& stats = renderer_.residualStats();
        float x0 = imgPos.x + avail.x - 230 * uiScale;
        float y0 = imgPos.y + 10 * uiScale;
        float w0 = 210 * uiScale;
        float h0 = 48 * uiScale;
        dl->AddRectFilled({x0, y0}, {x0 + w0, y0 + h0}, IM_COL32(0,0,0,125), 4.f);
        const int steps = 32;
        float gx = x0 + 10 * uiScale, gy = y0 + 10 * uiScale;
        float gw = 135 * uiScale, gh = 8 * uiScale;
        for (int i = 0; i < steps; ++i) {
            double err = stats.scale * double(i) / double(steps - 1);
            ImU32 col = (ImU32)residualColorU32(err, stats.scale, renderer_.edgeErrorScale);
            float xa = gx + gw * float(i) / steps;
            float xb = gx + gw * float(i + 1) / steps + 1.f;
            dl->AddRectFilled({xa, gy}, {xb, gy + gh}, col);
        }
        char sText[96];
        snprintf(sText, sizeof(sText), "scale %.4g  p95 %.4g", stats.scale, stats.p95);
        dl->AddText({gx, gy + 14 * uiScale}, IM_COL32(220,218,218,210), sText);
        if (stats.staleCount > 0) {
            char stale[64];
            snprintf(stale, sizeof(stale), "%d stale", stats.staleCount);
            dl->AddText({x0 + 150 * uiScale, gy - 3 * uiScale},
                        IM_COL32(255,190,80,230), stale);
        }
    }
}

void Viewport3DPanel::drawToolbar() {
    auto& R = renderer_;
    float uiScale = std::max(1.f, ImGui::GetFontSize() / 16.f);

    // Row 1: graph toggles
    ImGui::Checkbox("Axes",        &R.showAxes);       ImGui::SameLine();
    ImGui::Checkbox("Trajectory",  &R.showTrajectory); ImGui::SameLine();
    if (R.showTrajectory) {
        ImGui::Checkbox("Fehlerfarben", &R.colorEdgesByError);
        if (R.colorEdgesByError) {
            ImGui::SameLine();
            ImGui::SetNextItemWidth(76 * uiScale);
            ImGui::SliderFloat("Scale##edgeerr", &R.edgeErrorScale, 0.05f, 10.f, "%.2f");
        }
        ImGui::SameLine();
    }
    ImGui::Checkbox("Kovarianzen", &R.showCovEllipsoids);
    if (R.showCovEllipsoids) {
        ImGui::SameLine();
        ImGui::SetNextItemWidth(70 * uiScale);
        ImGui::SliderFloat("σ×##cov", &R.covScale, 1.f, 10.f, "%.1f");
    }
    ImGui::SameLine(0,16);
    ImGui::Checkbox("Punktwolken", &R.showPointClouds);
    if (R.showPointClouds) {
        ImGui::SameLine();
        ImGui::SetNextItemWidth(62 * uiScale);
        ImGui::SliderFloat("px##ps", &R.globalPointSize, 0.f, 20.f, "%.0f");
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("0 = pro-Wolke-Größe");
    }
    ImGui::SameLine(0,16);
    ImGui::Checkbox("Primitive", &R.showPrimitives);
    ImGui::SameLine(0,20);
    ImGui::Checkbox("Follow", &R.followMode);

    // Row 2: sizes + grid + view presets
    ImGui::SetNextItemWidth(92 * uiScale);
    ImGui::SliderFloat("Axis##al", &R.axisLength, 0.05f, 5.f);   ImGui::SameLine();
    ImGui::SetNextItemWidth(82 * uiScale);
    ImGui::SliderFloat("Node##ns", &R.nodeSize,   0.01f, 1.f);

    ImGui::SameLine(0,20);
    ImGui::SetNextItemWidth(82 * uiScale);
    int gh = R.gridHalf;
    if (ImGui::SliderInt("Grid##gh", &gh, 5, 200)) R.gridHalf = gh;
    ImGui::SameLine();
    ImGui::SetNextItemWidth(72 * uiScale);
    float gs = R.gridStep;
    if (ImGui::SliderFloat("Step##gs", &gs, 0.1f, 20.f, "%.1f")) R.gridStep = gs;

    ImGui::SameLine(0,20);
    if (ImGui::Button("Front")) { R.camera().yaw=180; R.camera().pitch=0; }
    ImGui::SameLine();
    if (ImGui::Button("Top"))   { R.camera().yaw=180; R.camera().pitch=89; }
    ImGui::SameLine();
    if (ImGui::Button("Side"))  { R.camera().yaw=-90; R.camera().pitch=10; }
    ImGui::SameLine();
    if (ImGui::Button("Reset")) { R.camera()=Camera{}; }

    ImGui::Separator();
}

void Viewport3DPanel::drawSettings() {
    ImGui::SeparatorText("Camera Sensitivity");
    ImGui::Spacing();

    float uiScale = std::max(1.f, ImGui::GetFontSize() / 16.f);

    ImGui::SetNextItemWidth(240 * uiScale);
    ImGui::SliderFloat("Orbit  (LMB drag)",   &cfg_.orbitSensitivity,      0.05f,  2.0f,  "%.3f");
    ImGui::SetNextItemWidth(240 * uiScale);
    ImGui::SliderFloat("Zoom   (Scroll)",     &cfg_.zoomScrollSensitivity, 0.01f,  0.50f, "%.3f");
    ImGui::SetNextItemWidth(240 * uiScale);
    ImGui::SliderFloat("Zoom   (RMB drag)",   &cfg_.zoomDragSensitivity,   0.001f, 0.02f, "%.4f");
    ImGui::SetNextItemWidth(240 * uiScale);
    ImGui::SliderFloat("Pan    (MMB drag)",   &cfg_.panSensitivity,        0.001f, 0.05f, "%.4f");
    ImGui::SetNextItemWidth(240 * uiScale);
    ImGui::SliderFloat("Pan    (Shift+LMB)",  &cfg_.shiftPanSensitivity,   0.001f, 0.02f, "%.4f");
    ImGui::SetNextItemWidth(240 * uiScale);
    ImGui::SliderFloat("WASD / Pfeiltasten",  &cfg_.wasdSensitivity,       0.05f,  5.0f,  "%.2f");

    ImGui::Spacing();
    if (ImGui::Button("Defaults zurücksetzen"))
        cfg_ = CameraSettings{};

    ImGui::Spacing();
    ImGui::TextDisabled("Ctrl+S  –  Einstellungen speichern");
}

void Viewport3DPanel::handleCameraInput(ImVec2 imagePos, ImVec2 imageSize) {
    ImGuiIO& io   = ImGui::GetIO();
    ImVec2   mpos = io.MousePos;
    auto&    cam  = renderer_.camera();

    bool hovered = mpos.x >= imagePos.x && mpos.x < imagePos.x + imageSize.x &&
                   mpos.y >= imagePos.y && mpos.y < imagePos.y + imageSize.y;

    // ── Mouse ─────────────────────────────────────────────────────────────────
    // Shift+LMB → pan;  plain LMB → orbit
    if (hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        if (io.KeyShift) shiftPanning_ = true;
        else             orbiting_     = true;
    }
    if (ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
        orbiting_     = false;
        shiftPanning_ = false;
    }

    // Orbit (LMB, no Shift)
    if (orbiting_ && ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
        cam.orbit(-io.MouseDelta.x * cfg_.orbitSensitivity,
                 -io.MouseDelta.y * cfg_.orbitSensitivity);
    }

    // Pan: MMB drag  (Camera::pan internally scales by distance * 0.01,
    // so we normalise the user factor against that baseline)
    if (hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Middle)) panning_ = true;
    if (ImGui::IsMouseReleased(ImGuiMouseButton_Middle))            panning_ = false;

    if (panning_ && ImGui::IsMouseDragging(ImGuiMouseButton_Middle)) {
        float s = cfg_.panSensitivity / 0.01f;
        cam.pan({io.MouseDelta.x * s, io.MouseDelta.y * s});
    }

    // Pan: Shift+LMB (lower sensitivity by default)
    if (shiftPanning_ && ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
        float s = cfg_.shiftPanSensitivity / 0.01f;
        cam.pan({io.MouseDelta.x * s, io.MouseDelta.y * s});
    }

    // Right drag → zoom (dolly)
    if (hovered && ImGui::IsMouseDragging(ImGuiMouseButton_Right)) {
        float delta = -io.MouseDelta.y * cam.distance * cfg_.zoomDragSensitivity;
        cam.zoom(delta);
    }

    // Scroll → zoom
    if (hovered && io.MouseWheel != 0) {
        cam.zoom(io.MouseWheel * cam.distance * cfg_.zoomScrollSensitivity);
    }

    // ── Keyboard: WASD / Arrow keys ──────────────────────────────────────────
    // IsWindowFocused instead of WantCaptureKeyboard so keys always work when
    // the viewport is the active panel.
    bool winFocused = ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows);
    if (hovered && winFocused) {
        float dt   = io.DeltaTime;
        float fwd  = 0.f, rght = 0.f;

        if (ImGui::IsKeyDown(ImGuiKey_W)         || ImGui::IsKeyDown(ImGuiKey_UpArrow))    fwd  -= dt;
        if (ImGui::IsKeyDown(ImGuiKey_S)         || ImGui::IsKeyDown(ImGuiKey_DownArrow))  fwd  += dt;
        if (ImGui::IsKeyDown(ImGuiKey_D)         || ImGui::IsKeyDown(ImGuiKey_RightArrow)) rght -= dt;
        if (ImGui::IsKeyDown(ImGuiKey_A)         || ImGui::IsKeyDown(ImGuiKey_LeftArrow))  rght += dt;

        if (fwd != 0.f || rght != 0.f)
            cam.moveGround(fwd * cfg_.wasdSensitivity,
                           rght * cfg_.wasdSensitivity);
    }
}

} // namespace gtsam_viz
