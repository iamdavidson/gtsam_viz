#pragma once
/**
 * graph.hpp  –  SLAM-Backend mit GTSAMViz Out-of-Process Visualisierung
 * =======================================================================
 *
 * EINBINDEN
 * ─────────
 * Kopiere in dein Projekt:
 *   gtsam_viz/src/ipc/GVizProtocol.h
 *   gtsam_viz/src/ipc/GVizClient.h
 *   integration/graph.hpp
 *
 * Kein CMake-Link gegen gtsam_viz_lib nötig — nur GTSAM + POSIX.
 *
 * BENUTZUNG
 * ─────────
 *   // Optional: Viz-Config anpassen
 *   factor_graph::VizConfig viz;
 *   viz.publishEveryN = 5;
 *   viz.backendName   = "LiDAR SLAM";
 *
 *   factor_graph::Graph g(viz);
 *   g.add_pose_prior(gtsam::Pose3::Identity());
 *
 *   // In deinem Sensor-Loop:
 *   g.add_pose_between(odom);
 *   g.add_loop_closure(from, to, rel_pose);
 *
 *   // Im Hintergrund-Optimierer:
 *   g.merge_optimzation(result, tail_key);
 *
 * WORKFLOW
 * ────────
 *   Terminal 1:  ./gtsam_viz          # GUI-Prozess starten
 *   Terminal 2:  ./my_slam            # Backend verbindet automatisch
 *
 * OHNE GUI (zero overhead)
 * ─────────────────────────
 *   target_compile_definitions(my_slam PRIVATE GTSAMVIZ_NO_IPC=1)
 *   → alle viz_*-Aufrufe werden zu No-Ops.
 */

#include <string>
#include <vector>
#include <mutex>
#include <cstdint>
#include <optional>

#include <yaml-cpp/yaml.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include "logger.hpp"

// ── IPC Client (opt-out mit GTSAMVIZ_NO_IPC=1) ───────────────────────────────
#ifndef GTSAMVIZ_NO_IPC
  #include "GVizClient.h"   // GVizProtocol.h + GVizClient.h neben graph.hpp
  #define GVIZ_AVAILABLE 1
#else
  #define GVIZ_AVAILABLE 0
#endif

namespace factor_graph {

// ─────────────────────────────────────────────────────────────────────────────
// VizConfig  –  steuert Publish-Verhalten
// ─────────────────────────────────────────────────────────────────────────────
struct VizConfig {
    /// false → alle Bridge-Aufrufe sind No-Ops
    bool   enabled            = true;

    /// Nur jeden N-ten add_pose_between publishen (1 = jeden Schritt)
    int    publishEveryN      = 1;

    /// Nach merge_optimzation() optimierte Values pushen
    bool   publishOnMerge     = true;

    /// Nach add_loop_closure() sofort Replace-Snapshot senden
    bool   publishOnLoopClose = true;

    /// Anzeigename im GUI Bridge-Panel
    std::string backendName   = "SLAM Backend";

    /// Socket-Pfad des gtsam_viz-Prozesses
    std::string socketPath    = "/tmp/gtsam_viz.sock";
};

// ─────────────────────────────────────────────────────────────────────────────
// GraphParams  –  Rauschmodelle (aus settings.yaml)
// ─────────────────────────────────────────────────────────────────────────────
struct GraphParams {
    double a{};
    std::vector<double> odometry_sigmas{ 0.1, 0.1, 0.1, 0.3, 0.3, 0.05 };
    std::vector<double> loop_sigmas    { 0.2, 0.2, 0.2, 0.3, 0.3, 0.01 };
    std::vector<double> prior_sigmas   { 0.01, 0.01, 0.01, 0.1, 0.1, 0.1 };
};

// ─────────────────────────────────────────────────────────────────────────────
// Graph
// ─────────────────────────────────────────────────────────────────────────────
class Graph {
private:

    GraphParams params;
    Logger      graph_log;
    VizConfig   viz_cfg_;

    gtsam::noiseModel::Diagonal::shared_ptr
        odometry_noise, loop_closure_noise, prior_noise;

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values               init_guess;
    std::mutex                  mtx;

    uint64_t     curr_pose_key = 0;
    int          step_counter_ = 0;
    gtsam::Pose3 bias;

#if GVIZ_AVAILABLE
    GVizClient client_;
#endif

    // ── Interne Publish-Helfer ────────────────────────────────────────────────

    void viz_publish_replace(const std::string& label = "") {
#if GVIZ_AVAILABLE
        if (!viz_cfg_.enabled) return;
        gtsam::NonlinearFactorGraph g_snap;
        gtsam::Values               v_snap;
        { std::lock_guard<std::mutex> lk(mtx); g_snap = graph; v_snap = init_guess; }
        client_.publish(g_snap, v_snap,
                        viz_cfg_.backendName + " | " + label
                        + " | poses=" + std::to_string(curr_pose_key));
#else
        (void)label;
#endif
    }

    void viz_publish_values(const std::string& label = "") {
#if GVIZ_AVAILABLE
        if (!viz_cfg_.enabled) return;
        gtsam::Values v_snap;
        { std::lock_guard<std::mutex> lk(mtx); v_snap = init_guess; }
        client_.publishValuesOnly(v_snap,
                        viz_cfg_.backendName + " | " + label);
#else
        (void)label;
#endif
    }

public:

    // ── Konstruktor ───────────────────────────────────────────────────────────
    /**
     * @param viz  Visualisierungskonfiguration (optional).
     *
     * Beispiele:
     *   Graph g;                                       // defaults
     *   Graph g({ .enabled = false });                 // Viz aus
     *   Graph g({ .publishEveryN = 5,
     *             .backendName  = "LiDAR SLAM" });
     */
    explicit Graph(VizConfig viz_cfg = {})
        : graph_log("GRAPH", LogColor::BRIGHT_CYAN)
        , params(load_params())
        , viz_cfg_(std::move(viz_cfg))
        , curr_pose_key(0)
    {
        bias = gtsam::Pose3::Identity();

        gtsam::Vector6 odom_s, loop_s, prior_s;
        for (int i = 0; i < 6; ++i) {
            odom_s(i)  = params.odometry_sigmas[i];
            loop_s(i)  = params.loop_sigmas[i];
            prior_s(i) = params.prior_sigmas[i];
        }
        odometry_noise     = gtsam::noiseModel::Diagonal::Sigmas(odom_s);
        loop_closure_noise = gtsam::noiseModel::Diagonal::Sigmas(loop_s);
        prior_noise        = gtsam::noiseModel::Diagonal::Sigmas(prior_s);

#if GVIZ_AVAILABLE
        if (viz_cfg_.enabled) {
            bool ok = client_.connect(viz_cfg_.socketPath.c_str());
            if (ok)
                graph_log.info("GTSAMViz verbunden (" + viz_cfg_.socketPath + ")");
            else
                graph_log.info("GTSAMViz nicht erreichbar – publish() ist No-Op bis GUI startet");
        }
#endif
    }

    ~Graph() {
#if GVIZ_AVAILABLE
        if (viz_cfg_.enabled) client_.disconnect();
#endif
    }

    // ── Parameter laden ───────────────────────────────────────────────────────
    GraphParams load_params() {
        YAML::Node settings = YAML::LoadFile("./config/settings.yaml");
        GraphParams loaded;
        loaded.odometry_sigmas = settings["graph"]["odometry_sigmas"].as<std::vector<double>>();
        loaded.loop_sigmas     = settings["graph"]["loop_sigmas"].as<std::vector<double>>();
        loaded.prior_sigmas    = settings["graph"]["prior_sigmas"].as<std::vector<double>>();
        return loaded;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // PUBLIC API  (identisch zur alten graph.hpp — drop-in replacement)
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Startpose als Prior-Faktor einfügen.
     * Sendet sofort einen Replace-Snapshot in die GUI.
     */
    uint64_t add_pose_prior(const gtsam::Pose3& pose,
                            const gtsam::noiseModel::Base::shared_ptr noise) {
        {
            std::lock_guard<std::mutex> lock(mtx);
            init_guess.insert(x_(curr_pose_key), pose);
            graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
                x_(curr_pose_key), pose, noise);
        }
        viz_publish_replace("prior key=" + std::to_string(curr_pose_key));
        return curr_pose_key++;
    }

    /// Überladung mit Default-Prior-Rauschen
    uint64_t add_pose_prior(const gtsam::Pose3& pose) {
        return add_pose_prior(pose, prior_noise);
    }

    /**
     * Neue Pose per Odometrie-BetweenFaktor einfügen.
     * Publishes gemäß VizConfig::publishEveryN.
     * Bei Verbindungsverlust: automatischer Reconnect beim nächsten Aufruf.
     *
     * @param pose   Odometrie-Relativpose
     */
    uint64_t add_pose_between(const gtsam::Pose3& pose,
                              const gtsam::noiseModel::Base::shared_ptr noise) {
        {
            std::lock_guard<std::mutex> lock(mtx);
            init_guess.insert(x_(curr_pose_key), bias * pose);
            const gtsam::Pose3 prev =
                bias.inverse() * init_guess.at<gtsam::Pose3>(x_(curr_pose_key-1));
            graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
                x_(curr_pose_key-1), x_(curr_pose_key), prev.between(pose), noise);
        }

        ++step_counter_;
        if (step_counter_ % viz_cfg_.publishEveryN == 0) {
            // Alle 10×publishEveryN Schritte: vollständiger Replace (neue Kanten sichtbar)
            // Sonst: nur Values-Update (günstiger, keine Topologie-Übertragung)
            bool do_replace = (step_counter_ % (viz_cfg_.publishEveryN * 10) == 0);
            if (do_replace)
                viz_publish_replace("odom key=" + std::to_string(curr_pose_key));
            else
                viz_publish_values("odom key=" + std::to_string(curr_pose_key));
        }

        return curr_pose_key++;
    }

    /// Überladung mit Default-Odometrie-Rauschen
    uint64_t add_pose_between(const gtsam::Pose3& pose) {
        return add_pose_between(pose, odometry_noise);
    }

    /**
     * Loop-Closure-Faktor einfügen.
     * Sendet immer einen vollständigen Replace-Snapshot damit die neue
     * Kante sofort in der GUI sichtbar ist.
     *
     * @param from_key      Key der aktuellen Pose
     * @param to_key        Key der Ziel-Pose (Loop-Schluss)
     * @param relative_pose Sensorgemessene Relativpose from → to
     */
    uint64_t add_loop_closure(uint64_t from_key, uint64_t to_key,
                              const gtsam::Pose3& relative_pose,
                              const gtsam::noiseModel::Base::shared_ptr noise = nullptr) {
        auto n = noise ? noise : loop_closure_noise;
        {
            std::lock_guard<std::mutex> lock(mtx);
            graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
                x_(from_key), x_(to_key), relative_pose, n);
        }
        if (viz_cfg_.publishOnLoopClose)
            viz_publish_replace("loop " + std::to_string(from_key)
                                + "→" + std::to_string(to_key));
        return curr_pose_key;
    }

    /**
     * Aktuellen Schätzwert einer Pose abfragen.
     * @return std::nullopt wenn Key nicht existiert.
     */
    std::optional<gtsam::Pose3> get_pose(uint64_t key) {
        std::lock_guard<std::mutex> lock(mtx);
        if (!init_guess.exists(x_(key))) return std::nullopt;
        return init_guess.at<gtsam::Pose3>(x_(key));
    }

    /**
     * Thread-sicherer Snapshot für Hintergrund-Optimierer.
     */
    void get_snap(gtsam::NonlinearFactorGraph& g, gtsam::Values& v) {
        std::lock_guard<std::mutex> lock(mtx);
        g = graph;
        v = init_guess;
    }

    /**
     * Optimiertes Ergebnis zurückschreiben + sofort in GUI pushen.
     * Sendet nur Values (keine Topologie-Übertragung → sehr günstig).
     *
     * @param v        Optimiertes Values-Objekt
     * @param tail_key Key der letzten optimierten Pose
     */
    void merge_optimzation(gtsam::Values& v, uint64_t tail_key) {
        {
            std::lock_guard<std::mutex> lock(mtx);
            init_guess.update(v);
            auto tail_opt = v.at<gtsam::Pose3>(x_(tail_key));
            auto tail_raw = bias.inverse() * init_guess.at<gtsam::Pose3>(x_(tail_key));
            bias = tail_opt * tail_raw.inverse();
        }
        if (viz_cfg_.publishOnMerge)
            viz_publish_values("merged tail=" + std::to_string(tail_key));
    }

    /// Laufzeit-Zugriff auf VizConfig (z.B. publishEveryN ändern)
    VizConfig& vizConfig() { return viz_cfg_; }

    /// Anzahl bisher eingefügter Poses
    uint64_t currentKey() const { return curr_pose_key; }

private:
    gtsam::Key x_(uint64_t key) const {
        return gtsam::Symbol('x', key).key();
    }
};

} // namespace factor_graph
