#pragma once
#include <imgui.h>
#include "../../graph/FactorGraphState.h"

namespace gtsam_viz {

/// Read-only stats panel: factor errors + marginals.
/// Optimization is done by the SLAM backend — no controls here.
class OptimizerPanel {
public:
    explicit OptimizerPanel(FactorGraphState& state);
    void draw();

private:
    void drawSummary();
    void drawFactorErrorTable();
    void drawMarginalsSection();

    FactorGraphState& state_;
};

} // namespace gtsam_viz
