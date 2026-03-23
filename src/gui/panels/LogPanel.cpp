#include "LogPanel.h"
#include <imgui.h>
#include <chrono>
#include <cstring>
#include <algorithm>

namespace gtsam_viz {

AppLogger& AppLogger::get() {
    static AppLogger instance;
    return instance;
}

void AppLogger::log(LogLevel lvl, const std::string& msg) {
    using namespace std::chrono;
    double t = duration<double>(steady_clock::now().time_since_epoch()).count();
    std::lock_guard<std::mutex> g(mtx_);
    if (entries_.size() >= MAX_ENTRIES) entries_.erase(entries_.begin());
    entries_.push_back({lvl, msg, t});
}

void LogPanel::draw() {
    // Filter bar
    ImGui::SetNextItemWidth(200);
    ImGui::InputText("Filter##log", filter_, sizeof(filter_));
    ImGui::SameLine();

    const char* levels[] = {"Debug","Info","Warning","Error"};
    ImGui::SetNextItemWidth(90);
    int lvl = (int)minLevel_;
    if (ImGui::Combo("Min Level", &lvl, levels, 4))
        minLevel_ = (LogLevel)lvl;

    ImGui::SameLine();
    ImGui::Checkbox("Auto Scroll", &autoScroll_);
    ImGui::SameLine();
    if (ImGui::Button("Clear")) AppLogger::get().clear();

    ImGui::Separator();

    // Log body
    ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.08f,0.09f,0.11f,1.f));
    ImGui::BeginChild("##logscroll", {-1,-1}, false,
                      ImGuiWindowFlags_HorizontalScrollbar);

    auto& entries = AppLogger::get().entries();
    std::string filterStr(filter_);

    static const ImVec4 colors[] = {
        {0.5f,0.55f,0.65f,1.f},   // Debug  – grey
        {0.7f,0.9f,0.7f,1.f},     // Info   – green
        {1.0f,0.85f,0.3f,1.f},    // Warning– yellow
        {1.0f,0.4f,0.35f,1.f},    // Error  – red
    };
    static const char* prefixes[] = {"[DBG]","[INF]","[WRN]","[ERR]"};

    for (auto& e : entries) {
        if (e.level < minLevel_) continue;
        if (!filterStr.empty() &&
            e.msg.find(filterStr) == std::string::npos) continue;

        ImGui::TextColored(colors[(int)e.level], "%s", prefixes[(int)e.level]);
        ImGui::SameLine();
        ImGui::TextUnformatted(e.msg.c_str());
    }

    if (autoScroll_ && ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 4.f)
        ImGui::SetScrollHereY(1.f);

    ImGui::EndChild();
    ImGui::PopStyleColor();
}

} // namespace gtsam_viz
