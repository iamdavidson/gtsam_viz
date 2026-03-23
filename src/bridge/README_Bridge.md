# GTSAMViz Live Bridge – Schnittstellen-Dokumentation

## Übersicht

```
┌─────────────────────────────┐        ┌────────────────────────────────┐
│   SLAM-Backend (dein Code)  │        │    GTSAMViz GUI (Main Thread)  │
│   (beliebiger Thread)       │        │                                │
│                             │        │  ┌──────────────────────────┐  │
│  SLAMPublisher pub;         │        │  │  BridgePanel             │  │
│  pub.connect();             │        │  │  • Verbindungsstatus     │  │
│                             │  Lock- │  │  • Publish-Rate (Hz)     │  │
│  pub.publishReplace(        │ ──────▶│  │  • Pause / Auto-Optimize │  │
│    graph, values, "kf42");  │ free   │  └──────────────────────────┘  │
│                             │ Bridge │                                │
│  pub.publishValuesUpdate(   │        │  GraphBridge::instance()       │
│    isam.calculateEstimate());│       │    .poll(factorGraphState)     │
│                             │        │    → 60 Hz, Main Thread        │
└─────────────────────────────┘        └────────────────────────────────┘
```

---

## In-Process API (selber Prozess, anderer Thread)

### Minimales Beispiel

```cpp
#include "bridge/GraphBridge.h"

// Im SLAM-Thread:
gtsam_viz::SLAMPublisher pub;
pub.connect();   // GUI zeigt "Backend VERBUNDEN"

// Nach jedem Keyframe:
gtsam::NonlinearFactorGraph graph = mySlam.getGraph();
gtsam::Values               vals  = mySlam.getValues();
pub.publishReplace(graph, vals, "keyframe " + std::to_string(kfId));

// Nach iSAM2-Update (nur Values, Graphstruktur unverändert):
pub.publishValuesUpdate(isam.calculateEstimate(), "isam2 update");

// Inkrementell einzelne Faktoren/Variablen senden:
pub.appendFactor(newOdometryFactor);
pub.appendVariable(X(newId), initialPose);

// Am Ende / bei Reset:
pub.clear();
pub.disconnect();
```

### Publish-Modi im Detail

| Modus | Methode | Wann verwenden |
|-------|---------|----------------|
| `Replace` | `publishReplace(g, v)` | Kompletter Snapshot nach Keyframe, Loop-Closure |
| `Append` | `appendFactor(f)` / `appendVariable(k,v)` | Inkrementell, z.B. live während Fahrt |
| `ValuesOnly` | `publishValuesUpdate(v)` | Optimizer läuft, Graphstruktur stabil |

### Direkter Bridge-Zugriff

```cpp
// Ohne SLAMPublisher: direkt auf Bridge zugreifen
auto& bridge = gtsam_viz::GraphBridge::instance();

bridge.publish(graph, values, gtsam_viz::PublishMode::Replace, "label");
bridge.publishValues(newValues);
bridge.appendFactor(factor);
bridge.clear();

// Update-Callback registrieren (wird im GUI-Thread aufgerufen):
bridge.onUpdate([](const gtsam_viz::GraphSnapshot& snap) {
    std::cout << "GUI updated: seq=" << snap.sequenceId << "\n";
});
```

---

## Typische SLAM-Integration

### iSAM2-Loop

```cpp
#include <gtsam/nonlinear/ISAM2.h>
#include "bridge/GraphBridge.h"

void slamThread(std::atomic<bool>& running) {
    gtsam_viz::SLAMPublisher pub;
    pub.connect();

    gtsam::ISAM2 isam;
    gtsam::NonlinearFactorGraph newFactors;
    gtsam::Values               newValues;

    while (running) {
        // ... Messungen verarbeiten, Faktoren/Values befüllen ...

        gtsam::ISAM2Result result = isam.update(newFactors, newValues);
        gtsam::Values estimate    = isam.calculateEstimate();

        // Nur Values live aktualisieren (effizient, kein Graph-Kopie):
        pub.publishValuesUpdate(estimate, "isam2 step");

        // Alle N Keyframes kompletten Graph senden:
        if (keyframeCount % 10 == 0) {
            pub.publishReplace(isam.getFactorsUnsafe(), estimate,
                               "keyframe " + std::to_string(keyframeCount));
        }

        newFactors.resize(0);
        newValues.clear();
    }
    pub.disconnect();
}
```

### Klassischer Batch-SLAM (LM)

```cpp
void batchSlamThread(std::atomic<bool>& running) {
    gtsam_viz::SLAMPublisher pub;
    pub.connect();

    while (running) {
        // Graph aufbauen:
        gtsam::NonlinearFactorGraph g;
        gtsam::Values               v0;
        buildGraph(g, v0);   // deine Funktion

        // Initiale Schätzung in GUI zeigen:
        pub.publishReplace(g, v0, "initial estimate");
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // Optimieren und jeden Schritt publishen:
        gtsam::LevenbergMarquardtParams params;
        gtsam::LevenbergMarquardtOptimizer opt(g, v0, params);
        for (int i = 0; i < 50 && running; ++i) {
            gtsam::Values step = opt.optimizeSafely();
            pub.publishValuesUpdate(step,
                "LM iter " + std::to_string(i) +
                " err=" + std::to_string(g.error(step)));
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }
    pub.disconnect();
}
```

### ROS2 Integration

```cpp
// In deinem ROS2-Node:
#include "bridge/GraphBridge.h"

class SLAMNode : public rclcpp::Node {
    gtsam_viz::SLAMPublisher pub_;

    void onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Faktor aus Odometrie bauen...
        pub_.appendFactor(betweenFactor);
        pub_.appendVariable(newKey, initialPose);
    }

    void onLoopClosure() {
        // Nach Loop-Closure: kompletten Graph senden
        pub_.publishReplace(graph_, values_, "loop closure!");
    }
};
```

---

## Konfiguration

```cpp
auto& bridge = gtsam_viz::GraphBridge::instance();

// Maximale Queue-Tiefe (Standard: 4)
// Wenn das Backend schneller published als die GUI konsumiert,
// werden ältere Snapshots verworfen (drop counter steigt)
bridge.setMaxQueueDepth(2);

// Minimaler Abstand zwischen GUI-Updates (Standard: 16ms = ~60Hz)
// Erhöhen für weniger häufige Updates (CPU-Entlastung GUI-Thread)
bridge.setMinUpdateIntervalMs(33.0);  // ~30 Hz
```

---

## Thread-Sicherheit

| Operation | Thread | Sicher? |
|-----------|--------|---------|
| `publish*()` / `append*()` | Beliebig | ✓ (Mutex) |
| `poll()` | **Nur Main/GUI-Thread** | ✓ |
| `onUpdate()` callback | GUI-Thread | ✓ |
| `GraphBridge::instance()` | Beliebig | ✓ (static) |

---

## GUI-Panel "Live Bridge"

Das Panel zeigt:
- **Verbindungsstatus** (grün/grau)
- **Publish-Rate** als Sparkline (Hz)
- **Sequenz-Counter** & **Drop-Counter**
- Toggle: **Updates pausieren** (GUI friert ein, Backend läuft weiter)
- Toggle: **Auto-Optimierung** (nach jedem Bridge-Update N LM-Schritte)
- **Bridge leeren**-Button
