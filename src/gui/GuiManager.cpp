#include "GuiManager.h"
#include "../globals.h"
#include "panels/LogPanel.h"
#include "../examples/LiveBackendExample.h"
#include <imgui.h>
#include <imgui_internal.h>
#include <implot.h>
#include <algorithm>

namespace gtsam_viz {

GuiManager::GuiManager() : state_() {}

bool GuiManager::init(Renderer3D& renderer, GVizServer& server, float uiScale) {
    uiScale_ = std::clamp(uiScale, 0.75f, 2.5f);
    server_ = &server;
    applyDarkTheme(uiScale_);

    optimizerPanel_  = std::make_unique<OptimizerPanel>(state_);
    logPanel_        = std::make_unique<LogPanel>();
    viewport3DPanel_ = std::make_unique<Viewport3DPanel>(state_, renderer);
    bridgePanel_     = std::make_unique<BridgePanel>(GraphBridge::instance());
    bridgePanel_->linkState(&state_);

    state_.onChanged([this]() {
        GVLOG_INFO("Graph updated: "
            + std::to_string(state_.variables().size()) + " variables, "
            + std::to_string(state_.factors().size())   + " factors");
    });

    GVLOG_INFO("GTSAMViz ready. Waiting for backend data…");
    return true;
}

void GuiManager::draw() {
    drawMainMenuBar();
    drawDockspace();

    // ── Global shortcuts ──────────────────────────────────────────────────────
    ImGuiIO& io = ImGui::GetIO();
    if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_S))
        saveSettings();

    // Poll IPC server (~60 Hz) — receives frames from the SLAM backend process
    if (server_ && !bridgePanel_->isPaused())
        server_->poll(state_);

    // ── 3D Viewport (main / central window) ───────────────────────────────────
    ImGui::SetNextWindowSize({900 * uiScale_, 700 * uiScale_}, ImGuiCond_FirstUseEver);
    if (ImGui::Begin("3D Viewport##panel", nullptr,
                     ImGuiWindowFlags_NoScrollbar |
                     ImGuiWindowFlags_NoScrollWithMouse)) {
        viewport3DPanel_->draw();
    }
    ImGui::End();

    // ── Stats panel (right) ───────────────────────────────────────────────────
    ImGui::SetNextWindowSize({360 * uiScale_, 600 * uiScale_}, ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Stats##panel")) {
        optimizerPanel_->draw();
    }
    ImGui::End();

    // ── Bridge panel (right, tabbed with Stats) ───────────────────────────────
    ImGui::SetNextWindowSize({360 * uiScale_, 300 * uiScale_}, ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Live Bridge##panel")) {
        bridgePanel_->draw();
    }
    ImGui::End();

    // ── Log (bottom strip) ────────────────────────────────────────────────────
    ImGui::SetNextWindowSize({900 * uiScale_, 180 * uiScale_}, ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Log##panel")) {
        logPanel_->draw();
    }
    ImGui::End();

    // ── Settings panel ────────────────────────────────────────────────────────
    ImGui::SetNextWindowSize({360 * uiScale_, 280 * uiScale_}, ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Settings##panel")) {
        drawSettingsWindow();
    }
    ImGui::End();

    if (showAbout_)     drawAboutWindow();
    if (showImGuiDemo_) ImGui::ShowDemoWindow(&showImGuiDemo_);
    if (showImPlotDemo_)ImPlot::ShowDemoWindow(&showImPlotDemo_);
}

// ── Menu bar ──────────────────────────────────────────────────────────────────
void GuiManager::drawMainMenuBar() {
    if (!ImGui::BeginMainMenuBar()) return;

    if (ImGui::BeginMenu("File")) {
        if (ImGui::MenuItem("Clear Graph", "Ctrl+N")) {
            state_.clear();
            GVLOG_INFO("Graph cleared.");
        }
        ImGui::Separator();
        if (ImGui::BeginMenu("Demo Backend")) {
            if (ImGui::MenuItem("Growing Chain  (4 Hz)"))  LiveBackendExample::startGrowingChain(4.0).detach();
            if (ImGui::MenuItem("iSAM2 Loop     (5 Hz)"))  LiveBackendExample::startISAM2(5.0).detach();
            if (ImGui::MenuItem("iSAM2 schnell (15 Hz)")) LiveBackendExample::startISAM2(15.0).detach();
            ImGui::EndMenu();
        }
        ImGui::Separator();
        if (ImGui::MenuItem("Quit", "Alt+F4")) g_requestQuit = true;
        ImGui::EndMenu();
    }

    if (ImGui::BeginMenu("View")) {
        if (ImGui::MenuItem("Settings",    "Ctrl+S")) saveSettings();
        ImGui::Separator();
        if (ImGui::MenuItem("ImGui Demo"))  showImGuiDemo_  = true;
        if (ImGui::MenuItem("ImPlot Demo")) showImPlotDemo_ = true;
        ImGui::EndMenu();
    }

    if (ImGui::BeginMenu("Help")) {
        if (ImGui::MenuItem("About")) showAbout_ = true;
        ImGui::EndMenu();
    }

    // Right-aligned status + IPC connection indicator
    double err = state_.values().empty() ? 0.0 : state_.totalError();
    const bool connected = server_ && server_->isConnected();
    const char* connStr = connected ? "● BACKEND" : "○ no backend";
    ImVec4 connCol = connected
        ? ImVec4{0.996f,0.690f,0.365f,1.f} : ImVec4{0.45f,0.43f,0.43f,1.f};
    char buf[120];
    snprintf(buf, sizeof(buf), "Vars: %zu  Factors: %zu  Error: %.5f",
             state_.variables().size(), state_.factors().size(), err);
    float connW = ImGui::CalcTextSize(connStr).x + 24 * uiScale_;
    float tw    = ImGui::CalcTextSize(buf).x + connW + 16 * uiScale_;
    float minX  = ImGui::GetCursorPosX() + 12 * uiScale_;
    ImGui::SetCursorPosX(std::max(minX, ImGui::GetIO().DisplaySize.x - tw));
    ImGui::TextColored(connCol, "%s", connStr);
    ImGui::SameLine();
    ImGui::TextDisabled("%s", buf);

    ImGui::EndMainMenuBar();
}

// ── Dockspace ─────────────────────────────────────────────────────────────────
void GuiManager::drawDockspace() {
    ImGuiViewport* vp   = ImGui::GetMainViewport();
    float          menuH = ImGui::GetFrameHeight();
    ImGui::SetNextWindowPos({vp->Pos.x, vp->Pos.y + menuH});
    ImGui::SetNextWindowSize({vp->Size.x, vp->Size.y - menuH});
    ImGui::SetNextWindowViewport(vp->ID);

    ImGuiWindowFlags df =
        ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse |
        ImGuiWindowFlags_NoResize   | ImGuiWindowFlags_NoMove     |
        ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus |
        ImGuiWindowFlags_NoDocking  | ImGuiWindowFlags_NoBackground;

    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding,   {0,0});
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding,  0.f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize,0.f);
    ImGui::Begin("##DockHost", nullptr, df);
    ImGui::PopStyleVar(3);

    dockspaceId_ = ImGui::GetID("MainDock");
    ImGui::DockSpace(dockspaceId_, {0,0}, ImGuiDockNodeFlags_PassthruCentralNode);

    // ── Build default layout once ─────────────────────────────────────────────
    if (firstFrame_) {
        firstFrame_ = false;
        ImGui::DockBuilderRemoveNode(dockspaceId_);
        ImGui::DockBuilderAddNode(dockspaceId_,
            ImGuiDockNodeFlags_PassthruCentralNode | ImGuiDockNodeFlags_DockSpace);
        ImGui::DockBuilderSetNodeSize(dockspaceId_, vp->Size);

        // Split: center | right sidebar. Keep the sidebar readable after UI scaling.
        ImGuiID center, right;
        float rightRatio = std::clamp((360.f * uiScale_) / std::max(vp->Size.x, 1.f),
                                      0.24f, 0.34f);
        ImGui::DockBuilderSplitNode(dockspaceId_, ImGuiDir_Right, rightRatio, &right, &center);

        // Center: 3D viewport (top, 85%) + Log (bottom, 15%)
        ImGuiID viewport, logStrip;
        ImGui::DockBuilderSplitNode(center, ImGuiDir_Down, 0.15f, &logStrip, &viewport);

        // Right sidebar: Bridge (top ~25%) + Stats (middle ~45%) + Settings (bottom ~30%)
        ImGuiID bridgeNode, statsSettings;
        ImGui::DockBuilderSplitNode(right, ImGuiDir_Down, 0.75f, &statsSettings, &bridgeNode);

        ImGuiID statsNode, settingsNode;
        ImGui::DockBuilderSplitNode(statsSettings, ImGuiDir_Down, 0.40f, &settingsNode, &statsNode);

        ImGui::DockBuilderDockWindow("3D Viewport##panel",  viewport);
        ImGui::DockBuilderDockWindow("Log##panel",          logStrip);
        ImGui::DockBuilderDockWindow("Live Bridge##panel",  bridgeNode);
        ImGui::DockBuilderDockWindow("Stats##panel",        statsNode);
        ImGui::DockBuilderDockWindow("Settings##panel",     settingsNode);
        ImGui::DockBuilderFinish(dockspaceId_);
    }

    ImGui::End();
}

// ── About ─────────────────────────────────────────────────────────────────────
void GuiManager::drawAboutWindow() {
    ImGui::SetNextWindowSize({460 * uiScale_, 260 * uiScale_}, ImGuiCond_Always);
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(),
                            ImGuiCond_Always, {0.5f,0.5f});
    if (!ImGui::Begin("About GTSAMViz", &showAbout_,
                       ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse))
    { ImGui::End(); return; }

    ImGui::TextColored({0.353f,0.478f,0.804f,1.f}, "GTSAMViz  v1.0.0");
    ImGui::Separator(); ImGui::Spacing();
    ImGui::TextWrapped(
        "Live 3D visualizer for GTSAM factor graphs.\n\n"
        "Controls:\n"
        "  LMB drag        – Orbit\n"
        "  Shift+LMB drag  – Pan\n"
        "  MMB drag        – Pan\n"
        "  RMB drag        – Zoom (dolly)\n"
        "  Scroll          – Zoom\n"
        "  WASD / Arrows   – Pan on ground plane");
    ImGui::Spacing();
    if (ImGui::Button("Close")) showAbout_ = false;
    ImGui::End();
}

// ── Theme ─────────────────────────────────────────────────────────────────────
// Palette:
//   #2B2A2A  (0.169, 0.165, 0.165)  – dominant dark base
//   #F5F2F2  (0.961, 0.949, 0.949)  – off-white text / light surfaces
//   #5A7ACD  (0.353, 0.478, 0.804)  – blue accent (interactive)
//   #FEB05D  (0.996, 0.690, 0.365)  – orange accent (highlights / marks)
void GuiManager::applyDarkTheme(float uiScale) {
    ImGuiStyle& s = ImGui::GetStyle();
    s.WindowRounding    = 4.f;  s.FrameRounding  = 3.f;
    s.ScrollbarRounding = 4.f;  s.GrabRounding   = 3.f;
    s.TabRounding       = 3.f;  s.PopupRounding  = 3.f;
    s.WindowBorderSize  = 1.f;  s.FrameBorderSize = 0.f;
    s.ItemSpacing       = {6,4}; s.FramePadding  = {6,3};

    // ── colour aliases ────────────────────────────────────────────────────────
    // base
    constexpr ImVec4 kBase      = {0.169f, 0.165f, 0.165f, 1.f};   // #2B2A2A
    constexpr ImVec4 kBaseDark  = {0.120f, 0.118f, 0.118f, 1.f};   // darker variant
    constexpr ImVec4 kBaseLight = {0.220f, 0.216f, 0.216f, 1.f};   // lighter variant
    constexpr ImVec4 kBorder    = {0.320f, 0.310f, 0.310f, 1.f};
    // text
    constexpr ImVec4 kText      = {0.961f, 0.949f, 0.949f, 1.f};   // #F5F2F2
    constexpr ImVec4 kTextDim   = {0.540f, 0.520f, 0.520f, 1.f};
    // blue accent   #5A7ACD
    constexpr ImVec4 kBlue      = {0.353f, 0.478f, 0.804f, 1.f};
    constexpr ImVec4 kBlueMid   = {0.280f, 0.380f, 0.640f, 1.f};
    constexpr ImVec4 kBlueDim   = {0.210f, 0.285f, 0.480f, 0.8f};
    constexpr ImVec4 kBlueFaint = {0.210f, 0.285f, 0.480f, 0.4f};
    // orange accent #FEB05D
    constexpr ImVec4 kOrange    = {0.996f, 0.690f, 0.365f, 1.f};
    constexpr ImVec4 kOrangeDim = {0.780f, 0.540f, 0.285f, 1.f};

    ImVec4* c = s.Colors;
    // backgrounds
    c[ImGuiCol_WindowBg]           = kBase;
    c[ImGuiCol_ChildBg]            = kBaseLight;
    c[ImGuiCol_PopupBg]            = {0.190f,0.185f,0.185f,0.97f};
    c[ImGuiCol_Border]             = kBorder;
    c[ImGuiCol_BorderShadow]       = {0,0,0,0};
    // text
    c[ImGuiCol_Text]               = kText;
    c[ImGuiCol_TextDisabled]       = kTextDim;
    // frames / inputs
    c[ImGuiCol_FrameBg]            = kBaseLight;
    c[ImGuiCol_FrameBgHovered]     = {0.270f,0.265f,0.265f,1.f};
    c[ImGuiCol_FrameBgActive]      = {0.190f,0.185f,0.185f,1.f};
    // title bar
    c[ImGuiCol_TitleBg]            = kBaseDark;
    c[ImGuiCol_TitleBgActive]      = kBlueDim;
    c[ImGuiCol_TitleBgCollapsed]   = kBaseDark;
    // menu / scrollbar
    c[ImGuiCol_MenuBarBg]          = kBaseDark;
    c[ImGuiCol_ScrollbarBg]        = kBaseDark;
    c[ImGuiCol_ScrollbarGrab]      = {0.310f,0.300f,0.300f,1.f};
    c[ImGuiCol_ScrollbarGrabHovered]= {0.380f,0.370f,0.370f,1.f};
    c[ImGuiCol_ScrollbarGrabActive] = kOrangeDim;
    // interactive marks
    c[ImGuiCol_CheckMark]          = kOrange;
    c[ImGuiCol_SliderGrab]         = kOrangeDim;
    c[ImGuiCol_SliderGrabActive]   = kOrange;
    // buttons  (blue accent)
    c[ImGuiCol_Button]             = kBlueDim;
    c[ImGuiCol_ButtonHovered]      = kBlueMid;
    c[ImGuiCol_ButtonActive]       = kBlue;
    // header (collapsibles, selectables, …)
    c[ImGuiCol_Header]             = kBlueFaint;
    c[ImGuiCol_HeaderHovered]      = kBlueDim;
    c[ImGuiCol_HeaderActive]       = kBlue;
    // separator / resize
    c[ImGuiCol_Separator]          = kBorder;
    c[ImGuiCol_SeparatorHovered]   = kBlueMid;
    c[ImGuiCol_SeparatorActive]    = kBlue;
    c[ImGuiCol_ResizeGrip]         = {0,0,0,0};
    c[ImGuiCol_ResizeGripHovered]  = kBlueDim;
    c[ImGuiCol_ResizeGripActive]   = kBlue;
    // tabs
    c[ImGuiCol_Tab]                = kBaseDark;
    c[ImGuiCol_TabHovered]         = kBlueMid;
    c[ImGuiCol_TabActive]          = kBlueDim;
    c[ImGuiCol_TabUnfocused]       = kBaseDark;
    c[ImGuiCol_TabUnfocusedActive] = kBaseLight;
    // docking
    c[ImGuiCol_DockingPreview]     = {kBlue.x,kBlue.y,kBlue.z,0.55f};
    c[ImGuiCol_DockingEmptyBg]     = kBase;
    // plots
    c[ImGuiCol_PlotLines]          = kBlue;
    c[ImGuiCol_PlotLinesHovered]   = kOrange;
    c[ImGuiCol_PlotHistogram]      = kOrangeDim;
    c[ImGuiCol_PlotHistogramHovered]= kOrange;
    // tables
    c[ImGuiCol_TableHeaderBg]      = kBaseLight;
    c[ImGuiCol_TableBorderLight]   = kBorder;
    c[ImGuiCol_TableBorderStrong]  = kBorder;
    c[ImGuiCol_TableRowBg]         = {0,0,0,0};
    c[ImGuiCol_TableRowBgAlt]      = {1,1,1,0.03f};
    // misc
    c[ImGuiCol_NavHighlight]       = kOrange;
    c[ImGuiCol_NavWindowingHighlight] = kOrange;

    s.ScaleAllSizes(uiScale);
}

// ── Settings window ───────────────────────────────────────────────────────────
void GuiManager::drawSettingsWindow() {
    if (viewport3DPanel_)
        viewport3DPanel_->drawSettings();
}

void GuiManager::saveSettings() {
    if (viewport3DPanel_) {
        viewport3DPanel_->settings().save("settings.cfg");
        GVLOG_INFO("Settings saved to settings.cfg");
    }
}

} // namespace gtsam_viz
