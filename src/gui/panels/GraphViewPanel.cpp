#include "GraphViewPanel.h"
#include <imgui.h>
#include <cmath>
#include <algorithm>
#include <sstream>

namespace gtsam_viz {

static const float NODE_R      = 22.f;
static const float FACTOR_HALF = 12.f;

GraphViewPanel::GraphViewPanel(FactorGraphState& state)
    : state_(state), layout_({}) {}

// ─────────────────────────────────────────────────────────────── draw() ───────
void GraphViewPanel::draw() {
    ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.09f,0.10f,0.12f,1.f));

    drawToolbar();

    // Auto-run layout
    if (layoutRunning_) {
        layout_.step(state_, 3);
    }

    // Recompute max error for colour scale
    double me = 1e-6;
    for (auto& f : state_.factors()) me = std::max(me, f.error);
    maxFactorError_ = me;

    drawCanvas();

    ImGui::PopStyleColor();
}

// ─────────────────────────────────────────────────────────────── toolbar ──────
void GraphViewPanel::drawToolbar() {
    ImGui::Spacing();
    ImGui::SameLine(6);

    // Layout controls
    if (ImGui::Button(layoutRunning_ ? " ■ Stop Layout" : " ▶ Layout")) {
        layoutRunning_ = !layoutRunning_;
        if (layoutRunning_) layout_.resetVelocities(state_);
    }
    ImGui::SameLine();
    if (ImGui::Button("  ○ Circle")) {
        layout_.circularInit(state_, 400, 300, 250);
        layout_.resetVelocities(state_);
    }
    ImGui::SameLine();
    if (ImGui::Button("  ⊞ Grid")) {
        layout_.gridInit(state_, 150, 100, 100);
        layout_.resetVelocities(state_);
    }

    ImGui::SameLine(0, 20);
    ImGui::TextDisabled("|");
    ImGui::SameLine(0, 20);

    ImGui::Checkbox("Factor Nodes", &showFactorNodes_);
    ImGui::SameLine();
    ImGui::Checkbox("Labels", &showLabels_);
    ImGui::SameLine();
    ImGui::Checkbox("Error Colors", &showErrorColors_);
    ImGui::SameLine();
    ImGui::Checkbox("Grid", &showGrid_);

    ImGui::SameLine(0, 20);
    ImGui::TextDisabled("|");
    ImGui::SameLine(0, 20);

    // Zoom
    ImGui::Text("Zoom"); ImGui::SameLine();
    ImGui::SetNextItemWidth(80);
    ImGui::SliderFloat("##zoom", &zoom_, 0.2f, 4.f, "%.2f×");
    ImGui::SameLine();
    if (ImGui::Button("Reset View")) { pan_ = {0,0}; zoom_ = 1.f; }

    // Layout params (collapsible)
    ImGui::SameLine(0, 20);
    if (ImGui::SmallButton("Layout Params…")) ImGui::OpenPopup("LayoutParams");
    if (ImGui::BeginPopup("LayoutParams")) {
        auto& cfg = layout_.config();
        ImGui::SliderFloat("Repulsion",  &cfg.repulsion,  1000, 50000, "%.0f");
        ImGui::SliderFloat("Attraction", &cfg.attraction, 0.001f, 0.2f, "%.4f");
        ImGui::SliderFloat("Damping",    &cfg.damping,    0.5f, 0.99f, "%.2f");
        ImGui::SliderFloat("Time Step",  &cfg.timeStep,   0.001f, 0.05f, "%.3f");
        ImGui::Checkbox("Separate Factor Nodes", &cfg.separateFactorNodes);
        ImGui::EndPopup();
    }

    ImGui::Separator();
}

// ─────────────────────────────────────────────────────────────── canvas ───────
void GraphViewPanel::drawCanvas() {
    ImVec2 cpos  = ImGui::GetCursorScreenPos();
    ImVec2 csize = ImGui::GetContentRegionAvail();
    if (csize.x < 10 || csize.y < 10) return;

    ImDrawList* dl = ImGui::GetWindowDrawList();

    // Background
    dl->AddRectFilled(cpos, {cpos.x+csize.x, cpos.y+csize.y},
                      IM_COL32(20,22,28,255));

    // Grid
    if (showGrid_) {
        float step = 50.f * zoom_;
        float ox   = std::fmod(pan_.x, step);
        float oy   = std::fmod(pan_.y, step);
        ImU32 gcol = IM_COL32(38,42,50,255);
        for (float x = cpos.x+ox; x < cpos.x+csize.x; x += step)
            dl->AddLine({x,cpos.y},{x,cpos.y+csize.y}, gcol);
        for (float y = cpos.y+oy; y < cpos.y+csize.y; y += step)
            dl->AddLine({cpos.x,y},{cpos.x+csize.x,y}, gcol);
    }

    // Clip drawing to canvas
    dl->PushClipRect(cpos, {cpos.x+csize.x, cpos.y+csize.y}, true);

    // ── Draw edges (factor→variable lines) ───────────────────────────────────
    auto& vars    = state_.variables();
    auto& factors = state_.factors();

    auto findVar = [&](gtsam::Key k) -> const VariableNode* {
        for (auto& v : vars) if (v.key == k) return &v;
        return nullptr;
    };

    for (auto& fn : factors) {
        ImU32 ecol = showErrorColors_
            ? errorColor(fn.error, maxFactorError_)
            : IM_COL32(120,125,135,200);

        for (auto k : fn.keys) {
            const VariableNode* vn = findVar(k);
            if (!vn) continue;
            ImVec2 va = worldToScreen(vn->pos, cpos);
            ImVec2 fb = showFactorNodes_
                ? worldToScreen(fn.pos, cpos)
                : worldToScreen(glm::vec2{fn.pos.x, fn.pos.y}, cpos);
            if (showFactorNodes_) {
                drawEdge(dl, va, fb, ecol, 1.5f);
            } else {
                // Draw edges between variables that share this factor
            }
        }

        // If no factor nodes: draw edges between all connected variable pairs
        if (!showFactorNodes_ && fn.keys.size() >= 2) {
            for (size_t i = 0; i+1 < fn.keys.size(); ++i) {
                auto* va = findVar(fn.keys[i]);
                auto* vb = findVar(fn.keys[i+1]);
                if (va && vb)
                    drawEdge(dl, worldToScreen(va->pos, cpos),
                                 worldToScreen(vb->pos, cpos), ecol, 1.5f);
            }
        }
    }

    // ── Draw factor nodes ─────────────────────────────────────────────────────
    if (showFactorNodes_) {
        for (size_t fi = 0; fi < factors.size(); ++fi) {
            auto& fn = factors[fi];
            drawFactor(dl, fn, cpos, zoom_);
        }
    }

    // ── Draw variable nodes ───────────────────────────────────────────────────
    for (size_t vi = 0; vi < vars.size(); ++vi) {
        drawNode(dl, vars[vi], cpos, zoom_);
    }

    dl->PopClipRect();

    // ── Invisible button for interaction ─────────────────────────────────────
    ImGui::SetCursorScreenPos(cpos);
    ImGui::InvisibleButton("##canvas", csize,
        ImGuiButtonFlags_MouseButtonLeft |
        ImGuiButtonFlags_MouseButtonMiddle |
        ImGuiButtonFlags_MouseButtonRight);

    handleCanvasInteraction(cpos, csize);

    // ── Status bar ────────────────────────────────────────────────────────────
    std::string status = "Variables: " + std::to_string(vars.size()) +
                         "  Factors: "  + std::to_string(factors.size());
    if (!vars.empty())
        status += "  Total Error: " + [&]{
            std::ostringstream oss; oss.precision(4); oss << std::fixed;
            oss << state_.totalError(); return oss.str();
        }();
    if (layoutRunning_) {
        float ke = layout_.kineticEnergy(state_);
        status += "  KE: " + [&]{
            std::ostringstream oss; oss.precision(2); oss << std::fixed;
            oss << ke; return oss.str();
        }();
    }
    ImVec2 sp = {cpos.x+6, cpos.y+csize.y-18};
    dl->AddRectFilled({sp.x-2,sp.y-2},{sp.x+400,sp.y+14}, IM_COL32(0,0,0,140));
    dl->AddText(sp, IM_COL32(160,170,190,255), status.c_str());
}

// ─────────────────────────────────────────────────────────────── nodes ────────
void GraphViewPanel::drawNode(ImDrawList* dl, const VariableNode& vn,
                              ImVec2 origin, float zoom) {
    ImVec2 p  = worldToScreen(vn.pos, origin);
    float  r  = nodeRadius_ * zoom;

    // Glow when selected
    bool sel = selectedVariable.has_value() && *selectedVariable == vn.key;
    if (sel) {
        dl->AddCircleFilled(p, r+6, IM_COL32(255,220,80,60));
        dl->AddCircle(p, r+5, IM_COL32(255,220,80,200), 32, 2.f);
    }

    // Fill
    ImU32 fill = typeColor(vn.type);
    dl->AddCircleFilled(p, r, fill);

    // Border
    ImU32 border = sel ? IM_COL32(255,220,80,255) : IM_COL32(200,210,230,180);
    dl->AddCircle(p, r, border, 32, 1.5f);

    // Label
    if (showLabels_ && zoom > 0.5f) {
        ImVec2 ts = ImGui::CalcTextSize(vn.label.c_str());
        dl->AddText({p.x - ts.x/2, p.y - ts.y/2},
                    IM_COL32(240,245,255,255), vn.label.c_str());
    }

    // Small type indicator ring
    const char* typeChar = "";
    switch (vn.type) {
    case VariableType::Pose2:  typeChar = "P2"; break;
    case VariableType::Pose3:  typeChar = "P3"; break;
    case VariableType::Point2: typeChar = "L2"; break;
    case VariableType::Point3: typeChar = "L3"; break;
    default:                   typeChar = "?";  break;
    }
    if (zoom > 0.7f) {
        ImVec2 ts = ImGui::CalcTextSize(typeChar);
        dl->AddText({p.x - ts.x/2, p.y + r + 3},
                    IM_COL32(140,155,180,200), typeChar);
    }
}

void GraphViewPanel::drawFactor(ImDrawList* dl, const FactorNode& fn,
                                ImVec2 origin, float zoom) {
    ImVec2 p    = worldToScreen(fn.pos, origin);
    float  half = factorSize_ * zoom;

    bool sel = selectedFactor.has_value() && *selectedFactor == fn.index;
    if (sel) {
        dl->AddRectFilled({p.x-half-5,p.y-half-5},{p.x+half+5,p.y+half+5},
                          IM_COL32(255,220,80,60));
    }

    ImU32 fill = showErrorColors_
        ? errorColor(fn.error, maxFactorError_)
        : IM_COL32(90,95,115,230);

    dl->AddRectFilled({p.x-half,p.y-half},{p.x+half,p.y+half}, fill);
    ImU32 border = sel ? IM_COL32(255,220,80,255) : IM_COL32(180,185,210,180);
    dl->AddRect({p.x-half,p.y-half},{p.x+half,p.y+half}, border, 2.f, 0, 1.5f);

    if (showLabels_ && zoom > 0.6f && !fn.label.empty()) {
        std::string lbl = fn.label.substr(0, 6);
        ImVec2 ts = ImGui::CalcTextSize(lbl.c_str());
        dl->AddText({p.x-ts.x/2, p.y-ts.y/2},
                    IM_COL32(230,235,255,220), lbl.c_str());
    }
}

void GraphViewPanel::drawEdge(ImDrawList* dl, ImVec2 a, ImVec2 b,
                              ImU32 col, float thickness) {
    dl->AddLine(a, b, col, thickness);
}

// ─────────────────────────────────────────────────────────── interaction ───────
void GraphViewPanel::handleCanvasInteraction(ImVec2 canvasPos, ImVec2 /*canvasSize*/) {
    ImGuiIO& io = ImGui::GetIO();
    bool hovered = ImGui::IsItemHovered();
    ImVec2 mpos  = io.MousePos;

    auto& vars    = state_.variables();
    auto& factors = state_.factors();

    // Hit test helper
    auto hitVar = [&]() -> int {
        for (int i=0; i<(int)vars.size(); ++i) {
            ImVec2 p = worldToScreen(vars[i].pos, canvasPos);
            float dx = mpos.x-p.x, dy = mpos.y-p.y;
            if (dx*dx+dy*dy < (nodeRadius_*zoom_)*(nodeRadius_*zoom_)) return i;
        }
        return -1;
    };
    auto hitFactor = [&]() -> int {
        for (int i=0; i<(int)factors.size(); ++i) {
            ImVec2 p = worldToScreen(factors[i].pos, canvasPos);
            float h  = factorSize_*zoom_;
            if (std::abs(mpos.x-p.x)<h && std::abs(mpos.y-p.y)<h) return i;
        }
        return -1;
    };

    // Mouse press
    if (hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        int vi = hitVar();
        int fi = showFactorNodes_ ? hitFactor() : -1;
        if (vi >= 0) {
            selectedVariable = vars[vi].key;
            selectedFactor   = std::nullopt;
            dragVarIdx_ = vi; nodeDrag_ = true;
        } else if (fi >= 0) {
            selectedFactor   = factors[fi].index;
            selectedVariable = std::nullopt;
            dragFactorIdx_ = fi; nodeDrag_ = true;
        } else {
            selectedVariable = std::nullopt;
            selectedFactor   = std::nullopt;
            dragging_        = true;
        }
        lastMousePos_ = mpos;
    }

    // Mouse drag
    if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
        ImVec2 delta = {mpos.x-lastMousePos_.x, mpos.y-lastMousePos_.y};
        if (nodeDrag_ && dragVarIdx_ >= 0) {
            vars[dragVarIdx_].pos.x += delta.x / zoom_;
            vars[dragVarIdx_].pos.y += delta.y / zoom_;
            vars[dragVarIdx_].velocity = {0,0};
            vars[dragVarIdx_].fixed   = true;
        } else if (nodeDrag_ && dragFactorIdx_ >= 0) {
            factors[dragFactorIdx_].pos.x += delta.x / zoom_;
            factors[dragFactorIdx_].pos.y += delta.y / zoom_;
            factors[dragFactorIdx_].velocity = {0,0};
        } else if (dragging_) {
            pan_.x += delta.x;
            pan_.y += delta.y;
        }
        lastMousePos_ = mpos;
    }

    if (ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
        dragging_      = false;
        nodeDrag_      = false;
        dragVarIdx_    = -1;
        dragFactorIdx_ = -1;
    }

    // Double-click → unpin
    if (hovered && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
        int vi = hitVar();
        if (vi >= 0) vars[vi].fixed = false;
    }

    // Middle mouse / right drag → pan
    if (hovered && ImGui::IsMouseDragging(ImGuiMouseButton_Middle)) {
        pan_.x += io.MouseDelta.x;
        pan_.y += io.MouseDelta.y;
    }

    // Scroll → zoom (centered on mouse)
    if (hovered && io.MouseWheel != 0) {
        float factor = 1.f + io.MouseWheel * 0.12f;
        // Zoom toward mouse position
        glm::vec2 before = screenToWorld({mpos.x, mpos.y}, canvasPos);
        zoom_ = glm::clamp(zoom_ * factor, 0.1f, 8.f);
        glm::vec2 after  = screenToWorld({mpos.x, mpos.y}, canvasPos);
        glm::vec2 diff   = after - before;
        pan_.x += diff.x * zoom_;
        pan_.y += diff.y * zoom_;
    }

    // Right-click context menu
    if (hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Right)) {
        ImGui::OpenPopup("##canvas_ctx");
    }
    if (ImGui::BeginPopup("##canvas_ctx")) {
        ImGui::TextDisabled("Canvas");
        ImGui::Separator();
        if (ImGui::MenuItem("Reset View"))     { pan_={0,0}; zoom_=1.f; }
        if (ImGui::MenuItem("Circle Layout"))  {
            layout_.circularInit(state_, 400, 300, 250);
            layout_.resetVelocities(state_);
        }
        if (ImGui::MenuItem("Unpin All Nodes")) {
            for (auto& v : vars) v.fixed = false;
        }
        ImGui::Separator();
        if (ImGui::MenuItem(layoutRunning_ ? "Stop Layout" : "Run Layout"))
            layoutRunning_ = !layoutRunning_;
        ImGui::EndPopup();
    }
}

// ─────────────────────────────────────────────────────────── utilities ────────
ImVec2 GraphViewPanel::worldToScreen(glm::vec2 w, ImVec2 origin) const {
    return { origin.x + w.x * zoom_ + pan_.x,
             origin.y + w.y * zoom_ + pan_.y };
}
glm::vec2 GraphViewPanel::screenToWorld(ImVec2 s, ImVec2 origin) const {
    return { (s.x - origin.x - pan_.x) / zoom_,
             (s.y - origin.y - pan_.y) / zoom_ };
}

ImU32 GraphViewPanel::errorColor(double err, double maxErr) const {
    float t = (float)glm::clamp(err / (maxErr + 1e-9), 0.0, 1.0);
    // green → yellow → red
    uint8_t r = (uint8_t)(t < 0.5f
        ? glm::mix(30.f, 255.f, t*2.f)
        : 255.f);
    uint8_t g = (uint8_t)(t < 0.5f
        ? glm::mix(200.f, 200.f, t*2.f)
        : glm::mix(200.f, 50.f, (t-0.5f)*2.f));
    uint8_t b = (uint8_t)(glm::mix(80.f, 30.f, t));
    return IM_COL32(r, g, b, 230);
}

ImU32 GraphViewPanel::typeColor(VariableType t) const {
    switch(t) {
    case VariableType::Pose2:  return IM_COL32(60,130,220,230);
    case VariableType::Pose3:  return IM_COL32(100,80,210,230);
    case VariableType::Point2: return IM_COL32(60,190,110,230);
    case VariableType::Point3: return IM_COL32(40,160,130,230);
    default:                   return IM_COL32(130,130,140,230);
    }
}

} // namespace gtsam_viz
