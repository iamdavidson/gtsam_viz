/**
 * usage_example.cpp
 * ─────────────────
 * Demonstrates the GTSAMViz v2 client API:
 *   - Graph publish with covariance ellipsoids
 *   - Point cloud streaming
 *   - 3-D geometry primitives via SceneBuilder
 *
 * Workflow:
 *   Terminal 1:  ./gtsam_viz          # start the GUI
 *   Terminal 2:  ./my_slam            # this program
 *
 * No shared libraries required — communication via Unix socket.
 */

#include "gviz_client.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/inference/Symbol.h>

#include <Eigen/Core>

#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include <vector>

using namespace gtsam;
using gtsam::symbol_shorthand::X;  // Pose3 variables: X(0), X(1), ...

// ─── helpers ──────────────────────────────────────────────────────────────────

static Pose3 odometry(int step) {
    // Small forward motion + slow yaw — traces a helix
    double yaw   = 0.15;
    double pitch = 0.02;
    return Pose3(Rot3::RzRyRx(yaw, pitch, 0.0), Point3(0.5, 0.0, 0.05));
}

// ─── Example 1: graph with covariance ellipsoids ──────────────────────────────

void example_graph(GVizClient& viz) {
    std::cout << "[example_graph] Building graph …\n";

    auto noise3  = noiseModel::Diagonal::Sigmas(Vector6::Constant(0.05));
    auto priorNm = noiseModel::Diagonal::Sigmas(Vector6::Constant(0.001));

    NonlinearFactorGraph graph;
    Values               values;

    graph.add(PriorFactor<Pose3>(X(0), Pose3::Identity(), priorNm));
    values.insert(X(0), Pose3::Identity());

    Pose3 current = Pose3::Identity();
    for (int i = 1; i <= 12; ++i) {
        Pose3 odom = odometry(i);
        current    = current * odom;
        graph.add(BetweenFactor<Pose3>(X(i-1), X(i), odom, noise3));
        values.insert(X(i), current);
    }
    // Loop closure
    graph.add(BetweenFactor<Pose3>(X(11), X(0),
                                   values.at<Pose3>(X(0)).between(values.at<Pose3>(X(11))),
                                   noise3));

    // Optimize
    LevenbergMarquardtParams p; p.setVerbosityLM("SILENT");
    Values result = LevenbergMarquardtOptimizer(graph, values, p).optimize();

    // Compute marginal covariances for ellipsoid display
    GVizClient::CovarianceMap covs;
    try {
        Marginals marginals(graph, result);
        for (int i = 0; i <= 12; ++i) {
            // For Pose3: full 6×6 marginal — take XYZ sub-block [3..5, 3..5]
            covs[X(i)] = marginals.marginalCovariance(X(i)).block<3,3>(3,3);
        }
    } catch (...) {
        std::cerr << "[example_graph] Marginals failed — skipping ellipsoids\n";
    }

    viz.publish(graph, result, "loop-closure SLAM", &covs);
    std::cout << "[example_graph] Done — " << result.size() << " poses published.\n";
}

// ─── Example 2: streaming point cloud ─────────────────────────────────────────

void example_point_cloud(GVizClient& viz) {
    std::cout << "[example_cloud] Streaming point cloud …\n";

    const int N = 2000;
    for (int frame = 0; frame < 5; ++frame) {
        std::vector<Eigen::Vector3f> pts;
        std::vector<Eigen::Vector4f> colors;
        pts.reserve(N); colors.reserve(N);

        for (int i = 0; i < N; ++i) {
            float t  = (float)i / N * 2.f * M_PI;
            float r  = 2.f + 0.5f * std::sin(5.f * t + (float)frame * 0.5f);
            float z  = (float)i / N * 3.f - 1.5f;
            pts.push_back({ r * std::cos(t), r * std::sin(t), z });

            // Color by height: blue → red
            float h = (z + 1.5f) / 3.f;
            colors.push_back({ h, 0.3f, 1.f - h, 1.f });
        }

        viz.publishPointCloud(pts, colors,
                              "scan frame " + std::to_string(frame), 4.f);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }
    std::cout << "[example_cloud] Done.\n";
}

// ─── Example 3: geometry primitives ───────────────────────────────────────────

void example_primitives(GVizClient& viz) {
    std::cout << "[example_prims] Sending geometry primitives …\n";

    viz.scene()
        // Coordinate axes at origin
        .coordFrame(Eigen::Vector3f::Zero(),
                    Eigen::Matrix3f::Identity(), 1.5f)

        // A row of spheres with increasing radius
        .sphere({-3, 0, 0}, 0.20f, {1, 0, 0, 1})
        .sphere({-2, 0, 0}, 0.30f, {1, 0.5f, 0, 1})
        .sphere({-1, 0, 0}, 0.40f, {1, 1, 0, 1})

        // Boxes
        .box({1, 0, 0}, {0.4f, 0.2f, 0.6f}, {0, 0.8f, 0.2f, 1})
        .box({2, 0, 0}, {0.3f, 0.5f, 0.3f}, {0, 0.4f, 1, 1},
             Eigen::AngleAxisf(0.4f, Eigen::Vector3f::UnitZ()).toRotationMatrix())

        // Arrows (direction indicators)
        .arrow({0, -2, 0}, {1, -2, 0}, {1, 0.2f, 0.2f, 1})
        .arrow({0, -2, 0}, {0, -1, 0}, {0.2f, 1, 0.2f, 1})
        .arrow({0, -2, 0}, {0, -2, 1}, {0.2f, 0.2f, 1, 1})

        // Cone and cylinder
        .cone({0, 2, 1}, {0, 2, 0}, 0.4f, {1, 0.5f, 0, 0.9f})
        .cylinder({0, 3, 0}, {0, 0, 1}, 0.25f, 0.8f, {0.6f, 0, 1, 0.9f})

        // A diagonal line
        .line({-4, -4, 0}, {4, 4, 2}, {1, 1, 0.5f, 1}, 2.f)

        .send(viz, "geometry demo");

    std::cout << "[example_prims] Done.\n";
}

// ─── main ─────────────────────────────────────────────────────────────────────

int main() {
    GVizClient viz;
    if (!viz.connect()) {
        std::cerr << "[main] Could not connect to GTSAMViz — is the GUI running?\n";
        std::cerr << "       Start it with:  ./gtsam_viz\n";
        return 1;
    }
    std::cout << "[main] Connected to GTSAMViz.\n";

    viz.clear();

    // 1. Optimized factor graph with covariance ellipsoids
    example_graph(viz);
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // 2. Streaming point cloud (5 frames)
    example_point_cloud(viz);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // 3. 3-D geometry primitives
    example_primitives(viz);

    std::cout << "[main] All examples done.  GTSAMViz remains open.\n";
    return 0;
}
