#include "GuiManager.h"
#include "../globals.h"
#include "panels/LogPanel.h"
#include "../examples/LiveBackendExample.h"
#include <imgui.h>
#include <imgui_internal.h>
#include <implot.h>

namespace gtsam_viz {

GuiManager::GuiManager() : state_() {}

bool GuiManager::init(Renderer3D& renderer, GVizServer& server) {
    server_ = &server;
    applyDarkTheme();

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

    ipcServer_.start();
    GVLOG_INFO("GTSAMViz ready. Waiting for backend data…");
    return true;
}

void GuiManager::draw() {
    drawMainMenuBar();
    drawDockspace();

    // Poll IPC server (~60 Hz) — receives frames from the SLAM backend process
    if (server_ && !bridgePanel_->isPaused())
        server_->poll(state_);

    // ── 3D Viewport (main / central window) ───────────────────────────────────
    ImGui::SetNextWindowSize({900,700}, ImGuiCond_FirstUseEver);
    if (ImGui::Begin("3D Viewport##panel", nullptr,
                     ImGuiWindowFlags_NoScrollbar |
                     ImGuiWindowFlags_NoScrollWithMouse)) {
        viewport3DPanel_->draw();
    }
    ImGui::End();

    // ── Stats panel (right) ───────────────────────────────────────────────────
    ImGui::SetNextWindowSize({320,600}, ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Stats##panel")) {
        optimizerPanel_->draw();
    }
    ImGui::End();

    // ── Bridge panel (right, tabbed with Stats) ───────────────────────────────
    ImGui::SetNextWindowSize({320,300}, ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Live Bridge##panel")) {
        bridgePanel_->draw();
    }
    ImGui::End();

    // ── Log (bottom strip) ────────────────────────────────────────────────────
    ImGui::SetNextWindowSize({900,160}, ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Log##panel")) {
        logPanel_->draw();
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
    const char* connStr = ipcServer_.isConnected() ? "● BACKEND" : "○ no backend";
    ImVec4 connCol = ipcServer_.isConnected()
        ? ImVec4{0.2f,0.95f,0.4f,1.f} : ImVec4{0.5f,0.5f,0.55f,1.f};
    char buf[120];
    snprintf(buf, sizeof(buf), "Vars: %zu  Factors: %zu  Error: %.5f",
             state_.variables().size(), state_.factors().size(), err);
    float connW = ImGui::CalcTextSize(connStr).x + 24;
    float tw    = ImGui::CalcTextSize(buf).x + connW + 16;
    ImGui::SetCursorPosX(ImGui::GetIO().DisplaySize.x - tw);
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

        // Split: center (75%) | right sidebar (25%)
        ImGuiID center, right;
        ImGui::DockBuilderSplitNode(dockspaceId_, ImGuiDir_Right, 0.25f, &right, &center);

        // Center: 3D viewport (top, 85%) + Log (bottom, 15%)
        ImGuiID viewport, logStrip;
        ImGui::DockBuilderSplitNode(center, ImGuiDir_Down, 0.15f, &logStrip, &viewport);

        // Right sidebar: Bridge (top ~35%) + Stats (bottom ~65%)
        ImGuiID bridgeNode, statsNode;
        ImGui::DockBuilderSplitNode(right, ImGuiDir_Down, 0.65f, &statsNode, &bridgeNode);

        ImGui::DockBuilderDockWindow("3D Viewport##panel",  viewport);
        ImGui::DockBuilderDockWindow("Log##panel",          logStrip);
        ImGui::DockBuilderDockWindow("Live Bridge##panel",  bridgeNode);
        ImGui::DockBuilderDockWindow("Stats##panel",        statsNode);
        ImGui::DockBuilderFinish(dockspaceId_);
    }

    ImGui::End();
}

// ── About ─────────────────────────────────────────────────────────────────────
void GuiManager::drawAboutWindow() {
    ImGui::SetNextWindowSize({400,220}, ImGuiCond_Always);
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(),
                            ImGuiCond_Always, {0.5f,0.5f});
    if (!ImGui::Begin("About GTSAMViz", &showAbout_,
                       ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse))
    { ImGui::End(); return; }

    ImGui::TextColored({0.4f,0.85f,1.f,1.f}, "GTSAMViz  v1.0.0");
    ImGui::Separator(); ImGui::Spacing();
    ImGui::TextWrapped(
        "Live 3D visualizer for GTSAM factor graphs.\n\n"
        "Controls:\n"
        "  LMB drag  – Orbit\n"
        "  MMB drag  – Pan\n"
        "  RMB drag  – Zoom (dolly)\n"
        "  Scroll    – Zoom\n"
        "  WASD / Arrows – Pan on ground plane");
    ImGui::Spacing();
    if (ImGui::Button("Close")) showAbout_ = false;
    ImGui::End();
}

// ── Theme ─────────────────────────────────────────────────────────────────────
void GuiManager::applyDarkTheme() {
    ImGuiStyle& s = ImGui::GetStyle();
    s.WindowRounding = 4.f; s.FrameRounding = 3.f;
    s.ScrollbarRounding = 4.f; s.GrabRounding = 3.f; s.TabRounding = 3.f;
    s.WindowBorderSize = 1.f; s.FrameBorderSize = 0.f;
    s.ItemSpacing = {6,4}; s.FramePadding = {6,3};

    ImVec4* c = s.Colors;
    c[ImGuiCol_WindowBg]           = {0.09f,0.10f,0.12f,1.f};
    c[ImGuiCol_ChildBg]            = {0.10f,0.11f,0.14f,1.f};
    c[ImGuiCol_PopupBg]            = {0.10f,0.11f,0.14f,0.97f};
    c[ImGuiCol_Border]             = {0.22f,0.24f,0.30f,1.f};
    c[ImGuiCol_Header]             = {0.20f,0.40f,0.72f,0.5f};
    c[ImGuiCol_HeaderHovered]      = {0.26f,0.52f,0.90f,0.7f};
    c[ImGuiCol_HeaderActive]       = {0.26f,0.52f,0.90f,1.f};
    c[ImGuiCol_Button]             = {0.20f,0.36f,0.60f,0.8f};
    c[ImGuiCol_ButtonHovered]      = {0.30f,0.50f,0.82f,1.f};
    c[ImGuiCol_ButtonActive]       = {0.14f,0.28f,0.55f,1.f};
    c[ImGuiCol_FrameBg]            = {0.15f,0.17f,0.22f,1.f};
    c[ImGuiCol_FrameBgHovered]     = {0.20f,0.24f,0.32f,1.f};
    c[ImGuiCol_FrameBgActive]      = {0.10f,0.16f,0.28f,1.f};
    c[ImGuiCol_TitleBg]            = {0.08f,0.09f,0.12f,1.f};
    c[ImGuiCol_TitleBgActive]      = {0.12f,0.22f,0.40f,1.f};
    c[ImGuiCol_MenuBarBg]          = {0.07f,0.08f,0.10f,1.f};
    c[ImGuiCol_ScrollbarBg]        = {0.07f,0.08f,0.10f,1.f};
    c[ImGuiCol_ScrollbarGrab]      = {0.24f,0.28f,0.36f,1.f};
    c[ImGuiCol_CheckMark]          = {0.36f,0.72f,1.00f,1.f};
    c[ImGuiCol_SliderGrab]         = {0.36f,0.60f,0.90f,1.f};
    c[ImGuiCol_SliderGrabActive]   = {0.50f,0.75f,1.00f,1.f};
    c[ImGuiCol_Tab]                = {0.12f,0.20f,0.34f,0.9f};
    c[ImGuiCol_TabHovered]         = {0.28f,0.50f,0.82f,1.f};
    c[ImGuiCol_TabActive]          = {0.20f,0.38f,0.68f,1.f};
    c[ImGuiCol_TabUnfocused]       = {0.09f,0.13f,0.20f,0.9f};
    c[ImGuiCol_TabUnfocusedActive] = {0.14f,0.24f,0.42f,1.f};
    c[ImGuiCol_DockingPreview]     = {0.36f,0.60f,0.90f,0.7f};
    c[ImGuiCol_Separator]          = {0.20f,0.24f,0.32f,1.f};
    c[ImGuiCol_Text]               = {0.88f,0.90f,0.95f,1.f};
    c[ImGuiCol_TextDisabled]       = {0.40f,0.45f,0.55f,1.f};
    c[ImGuiCol_PlotHistogram]      = {0.36f,0.72f,1.00f,1.f};
    c[ImGuiCol_PlotLines]          = {0.36f,0.72f,1.00f,1.f};
    c[ImGuiCol_TableHeaderBg]      = {0.12f,0.18f,0.28f,1.f};
    c[ImGuiCol_TableBorderLight]   = {0.18f,0.22f,0.30f,1.f};
    c[ImGuiCol_TableRowBg]         = {0.00f,0.00f,0.00f,0.f};
    c[ImGuiCol_TableRowBgAlt]      = {1.00f,1.00f,1.00f,0.03f};
}

} // namespace gtsam_viz
