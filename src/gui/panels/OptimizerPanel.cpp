#include "OptimizerPanel.h"
#include "LogPanel.h"
#include <imgui.h>
#include <vector>
#include <algorithm>

namespace gtsam_viz {

OptimizerPanel::OptimizerPanel(FactorGraphState& state) : state_(state) {}

void OptimizerPanel::draw() {
    drawSummary();
    ImGui::Separator();
    drawFactorErrorTable();
    ImGui::Separator();
    drawMarginalsSection();
}

// ── Summary line ──────────────────────────────────────────────────────────────
void OptimizerPanel::drawSummary() {
    auto& vars = state_.variables();
    auto& facs = state_.factors();

    ImGui::TextColored({0.5f,0.85f,1.f,1.f}, "Graph Summary");
    ImGui::Spacing();

    ImGui::Text("Variables : %zu", vars.size());
    ImGui::Text("Factors   : %zu", facs.size());

    if (!vars.empty() && !facs.empty()) {
        double err = state_.totalError();
        ImGui::Spacing();
        ImGui::Text("Total error : ");
        ImGui::SameLine();
        // Color: green < 0.1, yellow < 1.0, red otherwise
        ImVec4 col = err < 0.1 ? ImVec4{0.3f,1.f,0.4f,1.f}
                   : err < 1.0 ? ImVec4{1.f,0.85f,0.2f,1.f}
                               : ImVec4{1.f,0.35f,0.3f,1.f};
        ImGui::TextColored(col, "%.6f", err);
    } else {
        ImGui::TextDisabled("No graph data.");
    }
}

// ── Factor error table ────────────────────────────────────────────────────────
void OptimizerPanel::drawFactorErrorTable() {
    ImGui::TextColored({0.5f,0.85f,1.f,1.f}, "Factor Errors");
    ImGui::Spacing();

    auto& factors = state_.factors();
    if (factors.empty()) {
        ImGui::TextDisabled("No factors.");
        return;
    }

    ImVec2 sz = {-1, std::min(220.f, float(factors.size())*22.f + 34.f)};
    if (ImGui::BeginTable("##ferr", 3,
            ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
            ImGuiTableFlags_ScrollY | ImGuiTableFlags_Sortable |
            ImGuiTableFlags_SizingFixedFit, sz)) {

        ImGui::TableSetupScrollFreeze(0, 1);
        ImGui::TableSetupColumn("#",     ImGuiTableColumnFlags_DefaultSort,            36.f);
        ImGui::TableSetupColumn("Type",  ImGuiTableColumnFlags_None,                  90.f);
        ImGui::TableSetupColumn("Error", ImGuiTableColumnFlags_PreferSortDescending,   80.f);
        ImGui::TableHeadersRow();

        // Sort
        static int  sortCol  = 2;
        static bool sortDesc = true;
        if (ImGuiTableSortSpecs* ss = ImGui::TableGetSortSpecs()) {
            if (ss->SpecsDirty && ss->SpecsCount > 0) {
                sortCol  = ss->Specs[0].ColumnIndex;
                sortDesc = ss->Specs[0].SortDirection == ImGuiSortDirection_Descending;
                ss->SpecsDirty = false;
            }
        }

        std::vector<int> order(factors.size());
        for (int i=0;i<(int)order.size();++i) order[i]=i;
        std::sort(order.begin(), order.end(), [&](int a, int b) {
            if (sortCol==2) return sortDesc ? factors[a].error > factors[b].error
                                            : factors[a].error < factors[b].error;
            return sortDesc ? a > b : a < b;
        });

        double maxE = 1e-9;
        for (auto& f : factors) maxE = std::max(maxE, f.error);

        for (int oi : order) {
            auto& fn = factors[oi];
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0); ImGui::Text("%zu", fn.index);
            ImGui::TableSetColumnIndex(1); ImGui::TextUnformatted(fn.label.c_str());
            ImGui::TableSetColumnIndex(2);
            float t = (float)(fn.error / (maxE+1e-9));
            ImGui::PushStyleColor(ImGuiCol_PlotHistogram,
                ImVec4(t, 1.f-t*0.7f, 0.2f*(1-t), 0.85f));
            char buf[32]; snprintf(buf,sizeof(buf),"%.5f",fn.error);
            ImGui::ProgressBar(t, {-1,0}, buf);
            ImGui::PopStyleColor();
        }
        ImGui::EndTable();
    }
}

// ── Marginals ─────────────────────────────────────────────────────────────────
void OptimizerPanel::drawMarginalsSection() {
    ImGui::TextColored({0.5f,0.85f,1.f,1.f}, "Marginals");
    ImGui::Spacing();

    if (ImGui::Button("Compute Marginals")) {
        bool ok = state_.computeMarginals();
        if (!ok) GVLOG_WARN("Marginals computation failed (graph singular?).");
        else     GVLOG_INFO("Marginals computed.");
    }
    ImGui::SameLine();
    ImGui::TextDisabled("(requires optimized graph)");

    // Show per-variable standard deviations if marginals are available
    auto& vars = state_.variables();
    bool any = false;
    for (auto& vn : vars) {
        auto cov = state_.marginalCovariance(vn.key);
        if (!cov) continue;
        any = true;
        const gtsam::Matrix& C = *cov;
        int off = (C.rows()==6) ? 3 : 0;
        double sx = std::sqrt(std::max(0.0, C(off+0,off+0)));
        double sy = std::sqrt(std::max(0.0, C(off+1,off+1)));
        double sz = (C.rows()>=off+3) ? std::sqrt(std::max(0.0,C(off+2,off+2))) : 0.0;
        ImGui::TextDisabled("%-6s  σx=%.4f  σy=%.4f  σz=%.4f",
                            vn.label.c_str(), sx, sy, sz);
    }
    if (!any && !vars.empty())
        ImGui::TextDisabled("No marginals yet. Click 'Compute Marginals'.");
}

} // namespace gtsam_viz
