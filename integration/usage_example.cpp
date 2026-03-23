/**
 * usage_example.cpp
 * ─────────────────
 * Zeigt wie graph.hpp in einem SLAM-Backend verwendet wird.
 *
 * Workflow:
 *   Terminal 1:  ./gtsam_viz          # GUI-Prozess starten
 *   Terminal 2:  ./my_slam            # dieses Programm
 *
 * Keine gemeinsamen Libraries nötig — Kommunikation über Unix-Socket.
 */

#include "graph.hpp"
#include <thread>
#include <atomic>
#include <chrono>
#include <iostream>

using namespace factor_graph;
using namespace gtsam;

// ─── Hintergrund-Optimierer ───────────────────────────────────────────────────
void optimizer_thread(Graph& g, std::atomic<bool>& stop) {
    LevenbergMarquardtParams p;
    p.maxIterations = 20;
    p.setVerbosityLM("SILENT");

    while (!stop) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        NonlinearFactorGraph gs; Values vs;
        g.get_snap(gs, vs);
        if (gs.empty() || vs.empty()) continue;

        try {
            Values result = LevenbergMarquardtOptimizer(gs, vs, p).optimize();
            uint64_t tail = g.currentKey() > 0 ? g.currentKey()-1 : 0;
            g.merge_optimzation(result, tail); // → Values-Update in GUI
        } catch (const std::exception& e) {
            std::cerr << "[Optimizer] " << e.what() << "\n";
        }
    }
}

// ─── SLAM-Thread ─────────────────────────────────────────────────────────────
void slam_thread(std::atomic<bool>& stop) {

    // Viz-Konfiguration — einzige Änderung gegenüber der alten graph.hpp
    VizConfig viz;
    viz.enabled           = true;
    viz.publishEveryN     = 1;
    viz.publishOnMerge    = true;
    viz.publishOnLoopClose = true;
    viz.backendName       = "My SLAM";
    // viz.socketPath     = "/tmp/gtsam_viz.sock";  // default

    Graph g(viz);
    g.add_pose_prior(Pose3::Identity());

    std::atomic<bool> opt_stop{false};
    std::thread opt_th(optimizer_thread, std::ref(g), std::ref(opt_stop));

    int step = 0;
    while (!stop) {
        Pose3 odom(
            Rot3::RzRyRx(0, 0, 0.05 * (step % 2 ? 1 : -1)),
            Point3(0.5, 0.0, 0.0)
        );
        g.add_pose_between(odom);

        if (step > 0 && step % 30 == 0) {
            g.add_loop_closure(g.currentKey()-1, 0, Pose3::Identity());
            std::cout << "[SLAM] Loop closure @ step " << step << "\n";
        }

        ++step;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    opt_stop = true;
    opt_th.join();
}

int main() {
    std::cout << "[main] SLAM-Thread starten (10 s Demo)\n";

    std::atomic<bool> stop{false};
    std::thread slam(slam_thread, std::ref(stop));

    std::this_thread::sleep_for(std::chrono::seconds(10));
    stop = true;
    slam.join();

    std::cout << "[main] Fertig.\n";
    return 0;
}
