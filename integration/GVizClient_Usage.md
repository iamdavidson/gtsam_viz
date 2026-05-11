# GVizClient in einem externen Projekt nutzen

Diese Anleitung beschreibt die Socket-Schnittstelle von GTSAMViz. Dein SLAM-,
Robotik- oder Optimierungsprozess laeuft dabei als eigenes Programm und sendet
Graph-, Pose-, Punktwolken- und Primitive-Daten an die laufende GTSAMViz-GUI.

## 1. GTSAMViz starten

Baue und starte zuerst die Visualisierung:

```bash
./build.sh release
cd build_release
./src/gtsam_viz
```

Die GUI lauscht standardmaessig auf:

```text
/tmp/gtsam_viz.sock
```

## 2. Welche Dateien kopieren?

Empfohlen ist der Single-File-Client:

```text
integration/gviz_client.h
```

Kopiere diese Datei in dein Projekt, zum Beispiel:

```text
my_slam_project/
  include/
    gtsam_viz/
      gviz_client.h
  src/
    main.cpp
```

Alternativ kannst du die Split-Variante verwenden:

```text
src/ipc/GVizProtocol.h
src/ipc/GVizClient.h
```

Dann muessen beide Dateien zusammen in denselben Include-Ordner kopiert werden.
Fuer neue externe Projekte ist `integration/gviz_client.h` einfacher, weil das
Protokoll dort bereits eingebettet ist.

## 3. Abhaengigkeiten

Der Client ist header-only. Du musst keine GTSAMViz-Library linken.

Dein Projekt braucht:

- GTSAM
- Eigen, kommt normalerweise ueber GTSAM
- POSIX-Sockets, also Linux oder macOS
- C++17

## 4. Koordinatensystem

Sende Daten im normalen ROS2/RViz2- beziehungsweise GTSAM-Koordinatensystem:

```text
X = Forward = rot
Y = Left    = gruen
Z = Up      = blau
```

GTSAMViz verwendet fuer die 3D-Anzeige dieselbe Achsenkonvention. Du sollst also
keine Achsen vor dem Senden tauschen. Ein Identity-Koordinatenrahmen wird in der
GUI wie in RViz2 dargestellt: rote X-Achse nach vorne, gruene Y-Achse nach links
und blaue Z-Achse nach oben.

## 5. CMake-Beispiel

```cmake
find_package(GTSAM REQUIRED)

add_executable(my_slam src/main.cpp)

target_compile_features(my_slam PRIVATE cxx_std_17)

target_include_directories(my_slam PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(my_slam PRIVATE
    gtsam
)
```

Wenn `gviz_client.h` unter `include/gtsam_viz/` liegt, bindest du ihn so ein:

```cpp
#include <gtsam_viz/gviz_client.h>
```

## 6. Minimalbeispiel

```cpp
#include <gtsam_viz/gviz_client.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

int main() {
    using gtsam::symbol_shorthand::X;

    GVizClient viz;
    if (!viz.connect()) {
        return 1; // GUI laeuft vermutlich nicht
    }

    auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas(
        gtsam::Vector6::Constant(0.01));
    auto odom_noise = gtsam::noiseModel::Diagonal::Sigmas(
        gtsam::Vector6::Constant(0.1));

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values values;

    gtsam::Pose3 p0 = gtsam::Pose3::Identity();
    gtsam::Pose3 odom(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.1),
                      gtsam::Point3(1.0, 0.0, 0.0));
    gtsam::Pose3 p1 = p0 * odom;

    graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(0), p0, prior_noise));
    graph.add(gtsam::BetweenFactor<gtsam::Pose3>(X(0), X(1), odom, odom_noise));

    values.insert(X(0), p0);
    values.insert(X(1), p1);

    viz.publish(graph, values, "initial graph");
    viz.disconnect();
}
```

## 7. Wichtige Methoden

```cpp
viz.connect();                         // verbindet mit /tmp/gtsam_viz.sock
viz.connect("/custom/path.sock");      // eigener Socket-Pfad
viz.publish(graph, values, "label");   // kompletter Graph-Snapshot
viz.publishValuesOnly(values, "opt");  // nur Posen/Values aktualisieren
viz.append(new_graph, new_values);     // inkrementelles Update
viz.appendEdge(key_a, key_b);          // einzelne visuelle Kante
viz.clear();                           // GUI-Zustand loeschen
viz.disconnect();                      // Socket schliessen
```

`publish()` ersetzt den aktuellen Graphen in der GUI. Nutze das fuer Keyframes,
Loop-Closure-Events oder regelmaessige Snapshots.

`publishValuesOnly()` ist guenstiger und aktualisiert nur die Positionen der
bekannten Variablen. Nutze das nach Optimierungsschritten.

## 8. Kovarianzen anzeigen

Wenn du Marginals berechnest, kannst du pro Variable eine 3x3-Positionskovarianz
mitschicken. GTSAMViz kann daraus Kovarianzellipsoide zeichnen.

```cpp
GVizClient::CovarianceMap covs;
gtsam::Marginals marginals(graph, result);

for (auto key : result.keys()) {
    auto marginal = marginals.marginalCovariance(key);
    covs[key] = marginal.block<3,3>(3,3); // Pose3: XYZ-Block
}

viz.publish(graph, result, "optimized with covariance", &covs);
```

In der GUI muss die Anzeige fuer Kovarianzen aktiviert sein.

## 9. Punktwolken senden

```cpp
std::vector<Eigen::Vector3f> points;
points.push_back({0.f, 0.f, 0.f});
points.push_back({1.f, 0.f, 0.f});

viz.publishPointCloud(points, "scan", 4.f, Eigen::Vector4f(1, 1, 1, 1));
```

Mit Farben pro Punkt:

```cpp
std::vector<Eigen::Vector4f> colors;
colors.push_back({1.f, 0.f, 0.f, 1.f});
colors.push_back({0.f, 1.f, 0.f, 1.f});

viz.publishPointCloud(points, colors, "colored scan", 4.f);
```

## 10. Primitive senden

```cpp
viz.scene()
    .coordFrame(Eigen::Vector3f::Zero(), Eigen::Matrix3f::Identity(), 1.0f)
    .line({0,0,0}, {1,0,0}, {1,1,0,1}, 2.f)
    .arrow({0,0,0}, {0,1,0}, {0,1,0,1})
    .sphere({2,0,0}, 0.25f, {1,0,0,1})
    .box({3,0,0}, {0.3f,0.3f,0.3f}, {0,0,1,1})
    .send(viz, "debug primitives");
```

Primitive ersetzen die aktuelle Primitive-Layer in der GUI.

## 11. Verhalten wenn die GUI nicht laeuft

`connect()` gibt `false` zurueck, wenn GTSAMViz nicht erreichbar ist. Publish-
Aufrufe versuchen spaeter automatisch erneut zu verbinden, sobald ein Socket-
Pfad bekannt ist. Dadurch kannst du Visualisierung optional halten und dein
Backend auch ohne GUI laufen lassen.

```cpp
GVizClient viz;
viz.connect(); // false ist okay, wenn Visualisierung optional ist

// Im Laufzeit-Code trotzdem publishen:
viz.publishValuesOnly(values, "tick");
```

## 12. Typischer Workflow

1. GTSAMViz starten.
2. Externes Projekt starten.
3. Beim ersten Keyframe `publish(graph, values, label)` senden.
4. Nach Optimierungen `publishValuesOnly(values, label)` senden.
5. Bei neuer Topologie, etwa Loop Closure, wieder `publish(graph, values, label)` senden.
6. Optional Punktwolken, Kovarianzen oder Primitive dazusenden.
