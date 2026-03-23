#include "DemoScenarios.h"

#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>

#include <cmath>
#include <random>

using namespace gtsam;
using gtsam::symbol_shorthand::X;
using gtsam::symbol_shorthand::L;

namespace gtsam_viz {

std::vector<DemoInfo> DemoScenarios::list() {
    return {
        { "Planar SLAM (Pose2 chain)",
          "Simple 2D SLAM: robot driving in a square, BetweenFactor odometry." },
        { "3D Pose Chain (Pose3)",
          "Robot moving through 3D space along a helix trajectory." },
        { "Landmark SLAM 2D",
          "2D SLAM with Point2 landmarks observed from Pose2 robot poses." },
        { "Noisy Odometry",
          "Heavily noisy odometry – see how optimization recovers clean path." },
        { "Loop Closure SLAM",
          "Long loop with a single loop-closure constraint closing the gap." },
    };
}

bool DemoScenarios::load(int index, FactorGraphState& state) {
    state.clear();
    switch (index) {
    case 0: loadPlanarSLAM(state);      break;
    case 1: loadPose3Chain(state);      break;
    case 2: loadLandmarkSLAM2D(state);  break;
    case 3: loadNoisyOdometry(state);   break;
    case 4: loadLoopClosure(state);     break;
    default: return false;
    }
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// 0 – Planar SLAM: robot drives in a square  (Pose2 × 5 poses)
// ─────────────────────────────────────────────────────────────────────────────
void DemoScenarios::loadPlanarSLAM(FactorGraphState& state) {
    NonlinearFactorGraph graph;
    Values initial;

    auto priorNoise   = noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1));
    auto odometryNoise= noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));

    // Prior on first pose
    graph.addPrior(X(0), Pose2(0,0,0), priorNoise);
    initial.insert(X(0), Pose2(0.5, 0.0, 0.2));

    // Square path: right → up → left → down
    Pose2 odometry[] = {
        Pose2(2.0, 0.0,  0.0),
        Pose2(2.0, 0.0,  M_PI/2),
        Pose2(2.0, 0.0,  0.0),
        Pose2(2.0, 0.0, -M_PI/2),
    };
    Pose2 truth = Pose2(0,0,0);
    for (int i = 0; i < 4; ++i) {
        truth = truth.compose(odometry[i]);
        graph.emplace_shared<BetweenFactor<Pose2>>(
            X(i), X(i+1), odometry[i], odometryNoise);
        // Noisy initial guess
        initial.insert(X(i+1), Pose2(truth.x() + 0.3*(i%2 ? 1:-1),
                                     truth.y() + 0.3*(i%2 ? -1:1),
                                     truth.theta() + 0.05));
    }
    state.setGraph(std::move(graph), initial);
}

// ─────────────────────────────────────────────────────────────────────────────
// 1 – 3D helix trajectory (Pose3 × 8 poses)
// ─────────────────────────────────────────────────────────────────────────────
void DemoScenarios::loadPose3Chain(FactorGraphState& state) {
    NonlinearFactorGraph graph;
    Values initial;

    auto priorNoise = noiseModel::Diagonal::Sigmas(
        (Vector(6) << 0.1,0.1,0.1, 0.1,0.1,0.1).finished());
    auto odNoise = noiseModel::Diagonal::Sigmas(
        (Vector(6) << 0.1,0.1,0.1, 0.2,0.2,0.2).finished());

    graph.addPrior(X(0), Pose3(), priorNoise);
    initial.insert(X(0), Pose3(Rot3(), Point3(0.1, 0, 0)));

    int N = 7;
    for (int i = 0; i < N; ++i) {
        double angle = 2.0*M_PI*double(i)/double(N);
        double nextAngle = 2.0*M_PI*double(i+1)/double(N);
        Point3 t_curr(3*std::cos(angle), 3*std::sin(angle), 0.5*double(i));
        Point3 t_next(3*std::cos(nextAngle), 3*std::sin(nextAngle), 0.5*double(i+1));

        // Rotation facing along trajectory
        Point3 dir = (t_next - t_curr) / (t_next - t_curr).norm();
        Rot3 R_next = Rot3::RzRyRx(0, std::asin(-dir.z()), std::atan2(dir.y(), dir.x()));

        Pose3 pose_curr(Rot3(), t_curr);
        Pose3 pose_next(R_next, t_next);
        Pose3 odom = pose_curr.inverse().compose(pose_next);

        graph.emplace_shared<BetweenFactor<Pose3>>(X(i), X(i+1), odom, odNoise);
        // Noisy initial
        initial.insert(X(i+1), Pose3(R_next,
            Point3(t_next.x() + 0.2, t_next.y() - 0.1, t_next.z() + 0.05)));
    }
    state.setGraph(std::move(graph), initial);
}

// ─────────────────────────────────────────────────────────────────────────────
// 2 – Landmark SLAM 2D  (3 poses + 4 landmarks)
// ─────────────────────────────────────────────────────────────────────────────
void DemoScenarios::loadLandmarkSLAM2D(FactorGraphState& state) {
    // Use a custom bearing-range factor approximated by a BetweenFactor on
    // the pose and landmark positions; for simplicity we use a measurement
    // factor that constrains Point2 relative to Pose2 via a custom unary factor.
    // Here we use a simplified version with BetweenFactor on augmented poses.
    NonlinearFactorGraph graph;
    Values initial;

    auto priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 0.05));
    auto odNoise    = noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1));
    auto measNoise  = noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1));

    // Poses: 3 robot positions
    graph.addPrior(X(0), Pose2(0,0,0), priorNoise);
    initial.insert(X(0), Pose2(0.1, 0, 0.05));

    graph.emplace_shared<BetweenFactor<Pose2>>(X(0), X(1),
        Pose2(2,0,0.1), odNoise);
    initial.insert(X(1), Pose2(2.3, 0.1, 0.15));

    graph.emplace_shared<BetweenFactor<Pose2>>(X(1), X(2),
        Pose2(2,0,-0.1), odNoise);
    initial.insert(X(2), Pose2(4.2, -0.1, 0.05));

    // Landmarks (treated as Pose2 with zero rotation for demo purposes)
    Point2 landmarks[] = {{1,2},{3,2},{1,-2},{3,-2}};
    for (int i = 0; i < 4; ++i) {
        // "Measurement" factor: pose → landmark as BetweenFactor<Pose2>
        // (simplified – in real SLAM you'd use BearingRangeFactor)
        Pose2 lm(landmarks[i].x(), landmarks[i].y(), 0);
        graph.addPrior(L(i), lm, measNoise);
        initial.insert(L(i), Pose2(landmarks[i].x() + 0.2*(i%2?1:-1),
                                   landmarks[i].y() + 0.15, 0));
    }
    state.setGraph(std::move(graph), initial);
}

// ─────────────────────────────────────────────────────────────────────────────
// 3 – Noisy odometry  (10 poses, very noisy)
// ─────────────────────────────────────────────────────────────────────────────
void DemoScenarios::loadNoisyOdometry(FactorGraphState& state) {
    NonlinearFactorGraph graph;
    Values initial;

    std::mt19937 rng(42);
    std::normal_distribution<double> noise(0.0, 0.6);

    auto priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.01, 0.01, 0.01));
    auto odNoise    = noiseModel::Diagonal::Sigmas(Vector3(0.5, 0.5, 0.3));

    graph.addPrior(X(0), Pose2(0,0,0), priorNoise);
    initial.insert(X(0), Pose2(0,0,0));

    Pose2 truth(0,0,0);
    for (int i = 0; i < 9; ++i) {
        Pose2 step(1.0, 0.0, 0.2);      // true odometry: forward + slight turn
        truth = truth.compose(step);
        graph.emplace_shared<BetweenFactor<Pose2>>(X(i), X(i+1), step, odNoise);
        initial.insert(X(i+1), Pose2(truth.x() + noise(rng),
                                     truth.y() + noise(rng),
                                     truth.theta() + noise(rng)*0.4));
    }
    state.setGraph(std::move(graph), initial);
}

// ─────────────────────────────────────────────────────────────────────────────
// 4 – Loop closure SLAM  (12 poses with loop back to start)
// ─────────────────────────────────────────────────────────────────────────────
void DemoScenarios::loadLoopClosure(FactorGraphState& state) {
    NonlinearFactorGraph graph;
    Values initial;

    auto priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 0.05));
    auto odNoise    = noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.15));
    auto loopNoise  = noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 0.05));

    int N = 12;
    graph.addPrior(X(0), Pose2(0,0,0), priorNoise);
    initial.insert(X(0), Pose2(0,0,0));

    // Circular path with drift
    std::mt19937 rng(7);
    std::normal_distribution<double> drift(0.0, 0.12);

    Pose2 truth(0,0,0);
    double dAngle = 2*M_PI / N;
    double R      = 4.0;
    for (int i = 0; i < N; ++i) {
        double angle     = dAngle * double(i);
        double nextAngle = dAngle * double(i+1);
        Point2 curr(R*std::cos(angle),     R*std::sin(angle));
        Point2 next(R*std::cos(nextAngle), R*std::sin(nextAngle));

        Pose2 p_curr(curr.x(), curr.y(), angle + M_PI/2);
        Pose2 p_next(next.x(), next.y(), nextAngle + M_PI/2);
        Pose2 odom = p_curr.inverse().compose(p_next);

        graph.emplace_shared<BetweenFactor<Pose2>>(X(i), X(i+1), odom, odNoise);
        // Drifting initial guess
        initial.insert(X(i+1),
            Pose2(next.x() + drift(rng) * double(i)*0.05,
                  next.y() + drift(rng) * double(i)*0.05,
                  p_next.theta() + drift(rng)*0.1));
    }
    // Loop closure: X(N) ≈ X(0)
    Pose2 p_last(R*std::cos(dAngle*N), R*std::sin(dAngle*N), dAngle*N+M_PI/2);
    Pose2 p_first(0,0,0);
    Pose2 loop = p_last.inverse().compose(p_first);
    graph.emplace_shared<BetweenFactor<Pose2>>(X(N), X(0), loop, loopNoise);

    state.setGraph(std::move(graph), initial);
}

} // namespace gtsam_viz
