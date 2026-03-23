#pragma once
#include "../graph/FactorGraphState.h"
#include <string>
#include <vector>

namespace gtsam_viz {

struct DemoInfo {
    std::string name;
    std::string description;
};

class DemoScenarios {
public:
    static std::vector<DemoInfo> list();

    // Load demo into state; returns false on failure
    static bool load(int index, FactorGraphState& state);

private:
    // ── Individual demos ───────────────────────────────────────────────────────
    static void loadPlanarSLAM(FactorGraphState& state);
    static void loadPose3Chain(FactorGraphState& state);
    static void loadLandmarkSLAM2D(FactorGraphState& state);
    static void loadNoisyOdometry(FactorGraphState& state);
    static void loadLoopClosure(FactorGraphState& state);
};

} // namespace gtsam_viz
