#pragma once
#include <imgui.h>
#include "../../graph/FactorGraphState.h"
#include <optional>

namespace gtsam_viz {

class InspectorPanel {
public:
    explicit InspectorPanel(FactorGraphState& state);
    void draw(std::optional<gtsam::Key>& selVar,
              std::optional<size_t>&     selFactor);

private:
    void drawVariableInspector(gtsam::Key key);
    void drawFactorInspector(size_t idx);
    void drawCovariance(gtsam::Key key);
    void drawPose2Values(gtsam::Key key);
    void drawPose3Values(gtsam::Key key);
    void drawMatrix(const gtsam::Matrix& M, const char* label);

    FactorGraphState& state_;
    bool marginalsComputed_ = false;
};

} // namespace gtsam_viz
