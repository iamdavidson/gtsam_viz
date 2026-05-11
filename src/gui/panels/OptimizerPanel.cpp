#include "OptimizerPanel.h"
#include "LogPanel.h"
#include <imgui.h>
#include <vector>
#include <algorithm>
#include <cmath>

namespace gtsam_viz {

static const char* factorTypeName(FactorType t) {
    switch (t) {
    case FactorType::Prior:      return "Prior";
    case FactorType::Between:    return "Between";
    case FactorType::Projection: return "Projection";
    case FactorType::BearingRange: return "BearingRange";
    default:                     return "Custom";
    }
}

static int factorTypeFilterValue(int comboIndex) {
    switch (comboIndex) {
    case 1: return (int)FactorType::Prior;
    case 2: return (int)FactorType::Between;
    case 3: return (int)FactorType::Projection;
    case 4: return (int)FactorType::BearingRange;
    case 5: return (int)FactorType::Custom;
    default: return -1;
    }
}

static int factorTypeFilterIndex(int filterValue) {
    if (filterValue == (int)FactorType::Prior) return 1;
    if (filterValue == (int)FactorType::Between) return 2;
    if (filterValue == (int)FactorType::Projection) return 3;
    if (filterValue == (int)FactorType::BearingRange) return 4;
    if (filterValue == (int)FactorType::Custom) return 5;
    return 0;
}

OptimizerPanel::OptimizerPanel(FactorGraphState& state,
                               std::optional<size_t>* selectedFactor)
    : state_(state), selectedFactor_(selectedFactor) {}

void OptimizerPanel::draw() {
    residualStats_ = computeResidualStats(state_.factors());
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

    ImGui::TextColored({0.5f,0.85f,1.f,1.f}, "Residuals");
    ImGui::Spacing();

    ImGui::Text("Variables : %zu", vars.size());
    ImGui::Text("Factors   : %zu", facs.size());
    ImGui::Text("Valid residuals : %d", residualStats_.validCount);

    if (residualStats_.validCount > 0) {
        ImGui::Text("Median / P95 / Max");
        ImGui::Text("%.5f / %.5f / %.5f",
                    residualStats_.median, residualStats_.p95, residualStats_.max);
        ImGui::TextDisabled("Scale: %.5f", residualStats_.scale);
    } else {
        ImGui::TextDisabled("No valid non-unary residuals.");
    }

    if (residualStats_.staleCount > 0) {
        ImGui::Spacing();
        ImGui::TextColored({1.f,0.70f,0.25f,1.f},
            "%d residual colors may be stale", residualStats_.staleCount);
        ImGui::TextDisabled("Use publishValuesWithErrors(...) after optimization.");
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

    ImGui::Checkbox("Only high", &showHighOnly_);
    ImGui::SameLine();
    ImGui::SetNextItemWidth(130);
    const char* filterItems[] = {"All", "Prior", "Between", "Projection", "BearingRange", "Custom"};
    int filterUi = factorTypeFilterIndex(typeFilter_);
    if (ImGui::Combo("Type", &filterUi, filterItems, IM_ARRAYSIZE(filterItems)))
        typeFilter_ = factorTypeFilterValue(filterUi);

    ImVec2 sz = {-1, std::min(280.f, float(factors.size())*24.f + 42.f)};
    if (ImGui::BeginTable("##ferr", 5,
            ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
            ImGuiTableFlags_ScrollY | ImGuiTableFlags_Sortable |
            ImGuiTableFlags_SizingFixedFit, sz)) {

        ImGui::TableSetupScrollFreeze(0, 1);
        ImGui::TableSetupColumn("#",      ImGuiTableColumnFlags_DefaultSort,           36.f);
        ImGui::TableSetupColumn("Type",   ImGuiTableColumnFlags_None,                 86.f);
        ImGui::TableSetupColumn("Error",  ImGuiTableColumnFlags_PreferSortDescending, 88.f);
        ImGui::TableSetupColumn("Src",    ImGuiTableColumnFlags_None,                 48.f);
        ImGui::TableSetupColumn("Keys",   ImGuiTableColumnFlags_None,                 44.f);
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

        for (int oi : order) {
            auto& fn = factors[oi];
            if (typeFilter_ >= 0 && (int)fn.type != typeFilter_) continue;
            if (showHighOnly_ && (!fn.errorValid || fn.error < residualStats_.p95)) continue;

            ImGui::TableNextRow();
            bool selected = selectedFactor_ && selectedFactor_->has_value()
                         && **selectedFactor_ == fn.index;
            ImGui::TableSetColumnIndex(0);
            char id[32]; snprintf(id, sizeof(id), "%zu", fn.index);
            if (ImGui::Selectable(id, selected,
                    ImGuiSelectableFlags_SpanAllColumns |
                    ImGuiSelectableFlags_AllowOverlap)) {
                if (selectedFactor_) *selectedFactor_ = fn.index;
            }

            ImGui::TableSetColumnIndex(1);
            ImGui::TextUnformatted(factorTypeName(fn.type));
            ImGui::TableSetColumnIndex(2);
            glm::vec4 c = factorResidualColor(fn, residualStats_.scale);
            ImGui::PushStyleColor(ImGuiCol_PlotHistogram, ImVec4(c.r,c.g,c.b,c.a));
            float t = fn.errorValid
                ? (float)std::clamp(fn.error / (residualStats_.scale + 1e-9), 0.0, 1.0)
                : 0.f;
            char buf[32];
            snprintf(buf, sizeof(buf), fn.errorValid ? "%.5f" : "invalid", fn.error);
            ImGui::ProgressBar(t, {-1,0}, buf);
            ImGui::PopStyleColor();

            ImGui::TableSetColumnIndex(3);
            ImGui::TextUnformatted(factorErrorSourceLabel(fn.errorSource));
            ImGui::TableSetColumnIndex(4);
            ImGui::Text("%zu", fn.keys.size());
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
