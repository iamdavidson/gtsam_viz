#pragma once
#include <imgui.h>
#include "../../graph/FactorGraphState.h"
#include "../../graph/ResidualColorScale.h"
#include <optional>

namespace gtsam_viz {

/// Residual diagnostics panel: factor errors + marginals.
/// Optimization is done by the SLAM backend — no controls here.
class OptimizerPanel {
public:
    explicit OptimizerPanel(FactorGraphState& state,
                            std::optional<size_t>* selectedFactor = nullptr);
    void draw();

private:
    void drawSummary();
    void drawFactorErrorTable();
    void drawMarginalsSection();

    FactorGraphState& state_;
    std::optional<size_t>* selectedFactor_ = nullptr;
    ResidualStats residualStats_;
    bool showHighOnly_ = false;
    int typeFilter_ = -1;
};

} // namespace gtsam_viz
