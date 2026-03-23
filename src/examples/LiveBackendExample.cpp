/**
 * LiveBackendExample.cpp
 * ─────────────────────────────────────────────────────────────────────────────
 * Demonstriert die Live-Bridge: Ein SLAM-Backend-Thread published
 * kontinuierlich wachsende Graphen, während die GUI live aktualisiert.
 *
 * Um diesen Thread zu starten: in main.cpp oder Application::init() aufrufen:
 *   auto thread = LiveBackendExample::startThread();
 *   // GUI-Loop ...
 *   thread.join();
 */
#include "LiveBackendExample.h"
#include "../bridge/GraphBridge.h"
#include "../gui/panels/LogPanel.h"

#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/inference/Symbol.h>

#include <chrono>
#include <cmath>
#include <random>
#include <sstream>
#include <thread>
#include <atomic>

using namespace gtsam;
using gtsam::symbol_shorthand::X;

namespace gtsam_viz {

// ─── Szenario A: wachsende Pose2-Kette (inkrementeller Append-Modus) ─────────
static void runGrowingChain(std::atomic<bool>& stop, SLAMPublisher& pub,
                            double stepHz) {
    auto noiseOd   = noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 0.05));
    auto noisePrior= noiseModel::Diagonal::Sigmas(Vector3(0.01,0.01,0.01));

    std::mt19937 rng(99);
    std::normal_distribution<double> noise(0.0, 0.08);

    NonlinearFactorGraph graph;
    Values               values;

    // Startpose als Prior
    Pose2 truth(0,0,0);
    graph.addPrior(X(0), truth, noisePrior);
    values.insert(X(0), Pose2(noise(rng), noise(rng), noise(rng)*0.2));
    pub.publishReplace(graph, values, "init");

    int step = 1;
    auto interval = std::chrono::duration<double>(1.0 / stepHz);

    while (!stop) {
        auto t0 = std::chrono::steady_clock::now();

        // Erzeuge Odometrie-Schritt (Kreisbahn)
        double angle = 0.25;   // rad/step
        Pose2  odom(0.5, 0.0, angle);
        truth = truth.compose(odom);

        // Noisy Initialschätzung
        Pose2 guess(truth.x() + noise(rng),
                    truth.y() + noise(rng),
                    truth.theta() + noise(rng) * 0.15);

        graph.emplace_shared<BetweenFactor<Pose2>>(
            X(step-1), X(step), odom, noiseOd);
        values.insert(X(step), guess);

        // Loop-Closure nach einer vollen Runde (~25 Schritte)
        if (step > 0 && step % 25 == 0) {
            auto loopNoise = noiseModel::Diagonal::Sigmas(Vector3(0.05,0.05,0.02));
            graph.emplace_shared<BetweenFactor<Pose2>>(
                X(step), X(0),
                truth.inverse().compose(Pose2(0,0,0)),
                loopNoise);
            pub.publishReplace(graph, values,
                "loop closure @ step " + std::to_string(step));
        } else {
            pub.publishReplace(graph, values, "step " + std::to_string(step));
        }

        ++step;
        std::this_thread::sleep_until(t0 + interval);
    }
}

// ─── Szenario B: iSAM2 live ─────────────────────────────────────────────────
static void runISAM2(std::atomic<bool>& stop, SLAMPublisher& pub,
                     double stepHz) {
    ISAM2Params params;
    params.relinearizeThreshold = 0.01;
    params.relinearizeSkip      = 1;
    ISAM2 isam(params);

    auto noiseOd    = noiseModel::Diagonal::Sigmas(Vector3(0.15, 0.15, 0.08));
    auto noisePrior = noiseModel::Diagonal::Sigmas(Vector3(0.01, 0.01, 0.01));

    std::mt19937 rng(42);
    std::normal_distribution<double> noise(0.0, 0.12);

    Pose2 truth(0,0,0);
    int   step = 0;
    auto  interval = std::chrono::duration<double>(1.0 / stepHz);

    while (!stop) {
        auto t0 = std::chrono::steady_clock::now();

        NonlinearFactorGraph newFactors;
        Values               newValues;

        if (step == 0) {
            newFactors.addPrior(X(0), truth, noisePrior);
            newValues.insert(X(0), Pose2(noise(rng)*0.1, noise(rng)*0.1, 0));
        } else {
            double a     = 0.3 * std::sin(double(step) * 0.1);
            Pose2 odom(0.4 + noise(rng)*0.05,
                       noise(rng)*0.03,
                       a + noise(rng)*0.05);
            truth = truth.compose(odom);

            Pose2 guess(truth.x() + noise(rng),
                        truth.y() + noise(rng),
                        truth.theta() + noise(rng) * 0.1);

            newFactors.emplace_shared<BetweenFactor<Pose2>>(
                X(step-1), X(step), odom, noiseOd);
            newValues.insert(X(step), guess);
        }

        try {
            isam.update(newFactors, newValues);
            isam.update(); // extra Relinearisierung
            Values estimate = isam.calculateEstimate();

            // Vollständigen Graphen alle 5 Schritte publishen
            if (step % 5 == 0) {
                pub.publishReplace(isam.getFactorsUnsafe(), estimate,
                    "iSAM2 step=" + std::to_string(step));
            } else {
                pub.publishValuesUpdate(estimate,
                    "iSAM2 values step=" + std::to_string(step));
            }
        } catch (...) {}

        ++step;
        std::this_thread::sleep_until(t0 + interval);
    }
}

// ─── LiveBackendExample ───────────────────────────────────────────────────────

std::thread LiveBackendExample::startGrowingChain(double hz) {
    return std::thread([hz]() {
        SLAMPublisher pub;
        pub.connect();
        GVLOG_INFO("[LiveDemo] Growing chain backend gestartet (" +
                   std::to_string((int)hz) + " Hz)");
        std::atomic<bool> stop{false};

        // Läuft bis Programm-Ende (thread ist detachable oder joinable)
        runGrowingChain(stop, pub, hz);
        pub.disconnect();
    });
}

std::thread LiveBackendExample::startISAM2(double hz) {
    return std::thread([hz]() {
        SLAMPublisher pub;
        pub.connect();
        GVLOG_INFO("[LiveDemo] iSAM2 backend gestartet (" +
                   std::to_string((int)hz) + " Hz)");
        std::atomic<bool> stop{false};
        runISAM2(stop, pub, hz);
        pub.disconnect();
    });
}

} // namespace gtsam_viz
