#pragma once
/**
 * GraphBridge – Live-Schnittstelle zwischen SLAM-Backend und GTSAMViz-GUI
 * =========================================================================
 *
 * DESIGN-ÜBERSICHT
 * ─────────────────
 * Das Backend läuft in einem eigenen Thread (z.B. ROS-Callback, iSAM2-Loop,
 * reiner SLAM-Thread). Die GUI läuft im Main-Thread (OpenGL).
 *
 * Kommunikation über GraphBridge (lock-free double-buffer + condition_variable):
 *
 *   Backend-Thread                GUI-Thread (60 Hz)
 *   ───────────────                ──────────────────
 *   bridge.publish(graph, values)  bridge.pollUpdate(state)
 *   bridge.publishValues(values)   → merged into FactorGraphState
 *   bridge.addFactor(f)
 *   bridge.setMetadata(...)
 *
 * Drei Publish-Modi:
 *   REPLACE  – kompletter Austausch (Snapshot jedes Keyframe)
 *   APPEND   – nur neue Faktoren / Variablen hinzufügen (inkrementell)
 *   VALUES   – nur Values updaten (z.B. nach Optimierung), Graph bleibt
 *
 * Zusätzlich: optionaler UNIX-Domain-Socket / Named-Pipe-Server
 * (GTSAMVIZ_ENABLE_IPC=ON) für Out-of-Process-Backends (Python, ROS-Node).
 */

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace gtsam_viz {

// ─── Publish-Modi ─────────────────────────────────────────────────────────────
enum class PublishMode {
    Replace,   ///< Kompletter Austausch des Graphen
    Append,    ///< Neue Faktoren / Variablen anhängen
    ValuesOnly ///< Nur Values aktualisieren, Graphstruktur bleibt
};

// ─── Snapshot (interner Double-Buffer-Eintrag) ─────────────────────────────
struct GraphSnapshot {
    PublishMode               mode       = PublishMode::Replace;
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values             values;
    std::string               label;        ///< Optionales Freitext-Label
    double                    timestampSec  = 0.0;
    uint64_t                  sequenceId    = 0;
};

// ─── Statistiken (für GUI-Statusleiste) ───────────────────────────────────
struct BridgeStats {
    uint64_t publishCount     = 0;
    uint64_t dropCount        = 0;   ///< Overwritten before GUI consumed
    double   lastPublishTime  = 0.0;
    double   avgPublishHz     = 0.0;
};

// ─── GraphBridge ──────────────────────────────────────────────────────────────
class GraphBridge {
public:
    // ── Singleton (bequem für In-Process-Nutzung) ─────────────────────────
    static GraphBridge& instance();

    // ── Backend API (thread-safe, aufrufbar aus beliebigem Thread) ─────────

    /**
     * Kompletten Graphen + Values publishen.
     * @param mode  Replace / Append / ValuesOnly
     * @param label Freitext (z.B. "keyframe 42", "loop closed")
     */
    void publish(const gtsam::NonlinearFactorGraph& graph,
                 const gtsam::Values& values,
                 PublishMode mode  = PublishMode::Replace,
                 const std::string& label = "");

    /** Nur Values publishen (Graphstruktur unverändert). */
    void publishValues(const gtsam::Values& values,
                       const std::string& label = "");

    /** Einzelnen Faktor anhängen (Append-Mode). */
    void appendFactor(gtsam::NonlinearFactor::shared_ptr factor);

    /** Einzelne Variable (Key + Value) anhängen. */
    void appendVariable(gtsam::Key key, const gtsam::Value& value);

    /** Graphen leeren (z.B. bei Reset im Backend). */
    void clear();

    // ── GUI API (nur aus dem Main/GUI-Thread aufrufen!) ────────────────────

    /**
     * Prüft ob ein neues Snapshot vorliegt und applied es auf den State.
     * @return true wenn etwas aktualisiert wurde.
     */
    bool poll(class FactorGraphState& state);

    /** Callback registrieren: wird im GUI-Thread nach jedem Update aufgerufen. */
    void onUpdate(std::function<void(const GraphSnapshot&)> cb);

    /** Verbindungsstatus: ist ein Backend aktiv? */
    bool isBackendConnected() const { return backendActive_.load(); }
    void setBackendConnected(bool v) { backendActive_.store(v); }

    const BridgeStats& stats() const { return stats_; }

    // ── Konfiguration ─────────────────────────────────────────────────────
    /** Max. Queue-Tiefe (überschüssige Snapshots werden verworfen). */
    void setMaxQueueDepth(size_t n) { maxQueueDepth_ = n; }

    /** Mindestabstand zwischen GUI-Updates (throttling). */
    void setMinUpdateIntervalMs(double ms) { minUpdateIntervalMs_ = ms; }

private:
    GraphBridge();

    double nowSec() const;
    void updateStats(double t);

    // ─ Thread-Schutz
    mutable std::mutex           mtx_;
    std::condition_variable      cv_;

    // ─ Double-Buffer: pending_ ist das zuletzt gepublishte Snapshot
    std::optional<GraphSnapshot> pending_;
    bool                         hasNew_      = false;

    // ─ Für Append-Modus: akkumulierter Graph
    gtsam::NonlinearFactorGraph  accumulated_;
    gtsam::Values                accumulatedVals_;

    uint64_t                     seqCounter_  = 0;
    std::atomic<bool>            backendActive_{false};

    // ─ Konfiguration
    size_t   maxQueueDepth_       = 4;
    double   minUpdateIntervalMs_ = 16.0;  // ~60 Hz
    double   lastPollTime_        = 0.0;

    // ─ Callbacks
    std::vector<std::function<void(const GraphSnapshot&)>> updateCallbacks_;

    // ─ Statistiken
    BridgeStats stats_;
    double      lastPublishTimeStat_ = 0.0;
};

// ─────────────────────────────────────────────────────────────────────────────
// BridgePanel – GUI-Panel für Verbindungsstatus und Steuerung
// ─────────────────────────────────────────────────────────────────────────────
class BridgePanel {
public:
    explicit BridgePanel(GraphBridge& bridge);
    void draw();

private:
    GraphBridge& bridge_;
    char  labelFilter_[64] = {};
    bool  pauseUpdates_    = false;
    bool  autoOptimize_    = false;
    int   autoOptMaxIter_  = 10;

    // History for sparkline
    static constexpr int HZ_HISTORY = 60;
    float hzHistory_[HZ_HISTORY] = {};
    int   hzIdx_ = 0;

    class FactorGraphState* statePtr_ = nullptr;
public:
    void linkState(class FactorGraphState* s) { statePtr_ = s; }
    bool isPaused() const { return pauseUpdates_; }
};

} // namespace gtsam_viz


// ─────────────────────────────────────────────────────────────────────────────
// BACKEND HELPER – schlanke Header-only API für dein SLAM-Backend
// Einfach diesen Header inkludieren, keine weiteren Abhängigkeiten nötig.
// ─────────────────────────────────────────────────────────────────────────────
namespace gtsam_viz {

/**
 * SLAMPublisher – komfortable Wrapper-Klasse für dein Backend.
 *
 * Typischer Einsatz in deinem SLAM-Backend:
 *
 *   #include "bridge/GraphBridge.h"
 *
 *   gtsam_viz::SLAMPublisher pub;
 *   pub.connect();  // markiert Backend als aktiv
 *
 *   // Nach jedem Keyframe:
 *   pub.publishReplace(myGraph, myValues, "keyframe " + std::to_string(kf));
 *
 *   // Inkrementell nach iSAM2-Update:
 *   pub.publishValuesUpdate(isam.calculateEstimate());
 *
 *   // Am Ende:
 *   pub.disconnect();
 */
class SLAMPublisher {
public:
    SLAMPublisher()  = default;
    ~SLAMPublisher() { disconnect(); }

    void connect()    { GraphBridge::instance().setBackendConnected(true);  }
    void disconnect() { GraphBridge::instance().setBackendConnected(false); }

    /** Kompletter Snapshot – ersetzt den gesamten Graphen in der GUI. */
    void publishReplace(const gtsam::NonlinearFactorGraph& g,
                        const gtsam::Values& v,
                        const std::string& label = "") {
        GraphBridge::instance().publish(g, v, PublishMode::Replace, label);
    }

    /** Nur aktuellen Values-Stand updaten (z.B. nach Optimierungsschritt). */
    void publishValuesUpdate(const gtsam::Values& v,
                             const std::string& label = "") {
        GraphBridge::instance().publishValues(v, label);
    }

    /** Inkrementell neuen Faktor hinzufügen. */
    void appendFactor(gtsam::NonlinearFactor::shared_ptr f) {
        GraphBridge::instance().appendFactor(f);
    }

    /** Inkrementell neue Variable hinzufügen. */
    void appendVariable(gtsam::Key k, const gtsam::Value& v) {
        GraphBridge::instance().appendVariable(k, v);
    }

    void clear() { GraphBridge::instance().clear(); }
};

} // namespace gtsam_viz
