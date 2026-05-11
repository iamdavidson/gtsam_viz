#include "../graph/FactorGraphState.h"
#include "../graph/ResidualColorScale.h"

#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <cassert>
#include <cmath>

using namespace gtsam_viz;
using gtsam::symbol_shorthand::X;

static void testResidualScale() {
    FactorNode unary;
    unary.index = 0;
    unary.keys = {X(0)};
    unary.error = 100.0;
    unary.errorValid = true;
    unary.errorFresh = true;

    FactorNode low;
    low.index = 1;
    low.keys = {X(0), X(1)};
    low.error = 0.0;
    low.errorValid = true;
    low.errorFresh = true;

    FactorNode high = low;
    high.index = 2;
    high.error = 10.0;

    std::vector<FactorNode> factors{unary, low, high};
    ResidualStats stats = computeResidualStats(factors);
    assert(stats.validCount == 2);
    assert(std::abs(stats.max - 10.0) < 1e-9);

    glm::vec4 lowCol = residualColor(low.error, stats.scale);
    glm::vec4 highCol = residualColor(high.error, stats.scale);
    assert(lowCol.g > lowCol.r);
    assert(highCol.r > highCol.g);
}

static void testBatchValueUpdate() {
    auto noise = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
    gtsam::NonlinearFactorGraph graph;
    graph.add(gtsam::BetweenFactor<gtsam::Pose2>(
        X(0), X(1), gtsam::Pose2(1.0, 0.0, 0.0), noise));

    gtsam::Values values;
    values.insert(X(0), gtsam::Pose2(0.0, 0.0, 0.0));
    values.insert(X(1), gtsam::Pose2(2.0, 0.0, 0.0));

    FactorGraphState state;
    state.setGraph(graph, values);
    double before = state.factors().front().error;

    gtsam::Values updated;
    updated.insert(X(1), gtsam::Pose2(1.0, 0.0, 0.0));
    state.updateValues(updated, false);

    double after = state.factors().front().error;
    auto initial = state.initialValues().at<gtsam::Pose2>(X(1));
    assert(before > after);
    assert(std::abs(initial.x() - 2.0) < 1e-9);
}

static void testVisualErrorRefresh() {
    FactorGraphState state;
    FactorNode fn;
    fn.index = 0;
    fn.type = FactorType::Between;
    fn.keys = {X(0), X(1)};
    fn.error = 2.0;
    fn.errorValid = true;
    fn.errorFresh = true;
    fn.errorSource = FactorErrorSource::Ipc;
    state.appendFactorVisual(fn);

    state.markFactorErrorsStale();
    assert(!state.factors().front().errorFresh);

    FactorNode fresh = fn;
    fresh.error = 0.25;
    fresh.errorSource = FactorErrorSource::Ipc;
    state.refreshVisualFactorErrors({fresh});
    assert(state.factors().size() == 1);
    assert(state.factors().front().errorFresh);
    assert(std::abs(state.factors().front().error - 0.25) < 1e-9);
}

static void testBearingRangeDetection() {
    gtsam::Vector3 sigmas;
    sigmas << 0.1, 0.1, 0.2;
    auto noise = gtsam::noiseModel::Diagonal::Sigmas(sigmas);

    gtsam::Key l0 = gtsam::Symbol('l', 0).key();
    gtsam::Pose3 pose = gtsam::Pose3::Identity();
    gtsam::Point3 landmark(2.0, 0.0, 0.0);

    gtsam::NonlinearFactorGraph graph;
    graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Point3>>(
        X(0), l0, pose.bearing(landmark), pose.range(landmark), noise);

    gtsam::Values values;
    values.insert(X(0), pose);
    values.insert(l0, landmark);

    FactorGraphState state;
    state.setGraph(graph, values);
    assert(state.factors().front().type == FactorType::BearingRange);

    bool sawPoint3 = false;
    for (const auto& v : state.variables())
        sawPoint3 = sawPoint3 || (v.key == l0 && v.type == VariableType::Point3);
    assert(sawPoint3);
}

int main() {
    testResidualScale();
    testBatchValueUpdate();
    testVisualErrorRefresh();
    testBearingRangeDetection();
    return 0;
}
