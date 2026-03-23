#include "Viewport3DPanel.h"
#include <imgui.h>
#include <string>
#include <cmath>

namespace gtsam_viz {

Viewport3DPanel::Viewport3DPanel(FactorGraphState& state, Renderer3D& renderer)
    : state_(state), renderer_(renderer) {}

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
    dl->AddText({imgPos.x+8, imgPos.y+8}, IM_COL32(180,210,240,210), info);

    // Key hint overlay (bottom-left)
    const char* hint = "LMB: Orbit  MMB: Pan  RMB: Zoom  Scroll: Zoom  WASD/Arrows: Pan";
    ImVec2 hintSz = ImGui::CalcTextSize(hint);
    dl->AddText({imgPos.x+8, imgPos.y+avail.y-hintSz.y-6},
                IM_COL32(120,130,150,160), hint);
}

void Viewport3DPanel::drawToolbar() {
    auto& R = renderer_;

    // Row 1: toggles
    ImGui::Checkbox("Axes",        &R.showAxes);       ImGui::SameLine();
    ImGui::Checkbox("Trajectory",  &R.showTrajectory); ImGui::SameLine();
    ImGui::Checkbox("Covariances", &R.showCovEllipsoids);
    if (R.showCovEllipsoids) {
        ImGui::SameLine();
        ImGui::SetNextItemWidth(60);
        ImGui::SliderFloat("σ×##cov", &R.covScale, 1.f, 10.f, "%.1f");
    }
    ImGui::SameLine(0,20);
    ImGui::Checkbox("Follow", &R.followMode);

    // Row 2: sizes + grid + view presets
    ImGui::SetNextItemWidth(80);
    ImGui::SliderFloat("Axis##al", &R.axisLength, 0.05f, 5.f);   ImGui::SameLine();
    ImGui::SetNextItemWidth(70);
    ImGui::SliderFloat("Node##ns", &R.nodeSize,   0.01f, 1.f);

    ImGui::SameLine(0,20);
    ImGui::SetNextItemWidth(70);
    int gh = R.gridHalf;
    if (ImGui::SliderInt("Grid##gh", &gh, 5, 200)) R.gridHalf = gh;
    ImGui::SameLine();
    ImGui::SetNextItemWidth(60);
    float gs = R.gridStep;
    if (ImGui::SliderFloat("Step##gs", &gs, 0.1f, 20.f, "%.1f")) R.gridStep = gs;

    ImGui::SameLine(0,20);
    if (ImGui::Button("Front")) { R.camera().yaw=-90; R.camera().pitch=0; }
    ImGui::SameLine();
    if (ImGui::Button("Top"))   { R.camera().yaw=-90; R.camera().pitch=89; }
    ImGui::SameLine();
    if (ImGui::Button("Side"))  { R.camera().yaw=0;   R.camera().pitch=10; }
    ImGui::SameLine();
    if (ImGui::Button("Reset")) { R.camera()=Camera{}; }

    ImGui::Separator();
}

void Viewport3DPanel::handleCameraInput(ImVec2 imagePos, ImVec2 imageSize) {
    ImGuiIO& io   = ImGui::GetIO();
    ImVec2   mpos = io.MousePos;
    auto&    cam  = renderer_.camera();

    bool hovered = mpos.x >= imagePos.x && mpos.x < imagePos.x + imageSize.x &&
                   mpos.y >= imagePos.y && mpos.y < imagePos.y + imageSize.y;

    // ── Mouse: RViz2-style ────────────────────────────────────────────────────
    // Left drag  → orbit
    if (hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left))  orbiting_ = true;
    if (ImGui::IsMouseReleased(ImGuiMouseButton_Left))             orbiting_ = false;
    if (orbiting_ && ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
        cam.orbit(io.MouseDelta.x * 0.4f, -io.MouseDelta.y * 0.4f);
        renderer_.followMode = false;  // disable follow on manual orbit
    }

    // Middle drag → pan
    if (hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Middle)) panning_ = true;
    if (ImGui::IsMouseReleased(ImGuiMouseButton_Middle))            panning_ = false;
    if (panning_ && ImGui::IsMouseDragging(ImGuiMouseButton_Middle)) {
        cam.pan({io.MouseDelta.x, io.MouseDelta.y});
        renderer_.followMode = false;
    }

    // Right drag → zoom (RViz2 style: right mouse = dolly)
    if (hovered && ImGui::IsMouseDragging(ImGuiMouseButton_Right)) {
        float delta = -io.MouseDelta.y * cam.distance * 0.005f;
        cam.zoom(delta);
    }

    // Scroll → zoom
    if (hovered && io.MouseWheel != 0) {
        cam.zoom(io.MouseWheel * cam.distance * 0.1f);
    }

    // ── Keyboard: WASD / Arrow keys (only when viewport is hovered) ───────────
    if (hovered && !io.WantCaptureKeyboard) {
        float dt       = io.DeltaTime;
        float fwd  = 0.f, rght = 0.f;

        if (ImGui::IsKeyDown(ImGuiKey_W)     || ImGui::IsKeyDown(ImGuiKey_UpArrow))    fwd  += dt;
        if (ImGui::IsKeyDown(ImGuiKey_S)     || ImGui::IsKeyDown(ImGuiKey_DownArrow))  fwd  -= dt;
        if (ImGui::IsKeyDown(ImGuiKey_D)     || ImGui::IsKeyDown(ImGuiKey_RightArrow)) rght += dt;
        if (ImGui::IsKeyDown(ImGuiKey_A)     || ImGui::IsKeyDown(ImGuiKey_LeftArrow))  rght -= dt;

        if (fwd != 0.f || rght != 0.f) {
            cam.moveGround(fwd, rght);
            renderer_.followMode = false;
        }
    }
}

} // namespace gtsam_viz
