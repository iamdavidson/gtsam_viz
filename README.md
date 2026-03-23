# *(vibe-coded)* GTSAMViz – Factor Graph Debugger & Visualizer

A fully-featured **C++17/CMake** desktop application for **tracking, debugging
and visualizing GTSAM factor graphs** in real time, built on:

| Library | Role |
|---------|------|
| **GTSAM** | Factor graph estimation back-end |
| **ImGui** (docking) | Immediate-mode GUI |
| **ImPlot** | Scientific plots (error history) |
| **GLFW** | Window & OpenGL context |
| **glad** | OpenGL 4.3 loader |
| **GLM** | Mathematics |

---

## Build Instructions

### Prerequisites

| Requirement | Version |
|-------------|---------|
| CMake       | ≥ 3.20  |
| C++ compiler| GCC ≥ 11 / Clang ≥ 14 / MSVC 2022 |
| GTSAM       | ≥ 4.2 (installed system-wide or via CMAKE_PREFIX_PATH) |
| OpenGL      | 4.3 capable GPU & driver |
| Git         | For FetchContent |

GLFW, glad, ImGui, ImPlot and GLM are **fetched automatically** at configure
time – no manual installation needed.

### Install GTSAM (Ubuntu / Debian example)

```bash
sudo apt install libboost-all-dev libeigen3-dev libtbb-dev

git clone https://github.com/borglab/gtsam.git
cd gtsam && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
         -DGTSAM_BUILD_TESTS=OFF \
         -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF
make -j$(nproc)
sudo make install
```

### Configure & Build GTSAMViz

```bash
git clone 
cd gtsam_viz && ./build.sh release
```

### CMake Options

| Option | Default | Description |
|--------|---------|-------------|
| `GTSAMVIZ_BUILD_EXAMPLES` | ON  | Build demo scenarios |
| `GTSAMVIZ_ENABLE_ASAN`    | OFF | AddressSanitizer |
| `GTSAMVIZ_ENABLE_TSAN`    | OFF | ThreadSanitizer |

---
## Usage 
### Separate Process (IPC via a socket)

Copy this files into your project.

```bash
GVizProtocol.h
GVizClient.h
```

Example structure 

```bash
your_proj/
    src/
    include/
        gtsam_viz/
            GVizProtocol.h
            GVizClient.h
```

In CMake 

```CMake
target_include_directories(your_exec PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include/gtsam_viz
)
```

In code

```cpp
#include "GVizClient.h"

GVizClient viz;

int main() {
    viz.connect("/tmp/gtsam_viz.sock");

    //...
    viz.publish(graph, values, "frame 42");x
    //...
    viz.publishValuesOnly(values, "optimized");
    //...

    viz.disconnect();
    return 0;
}
```

### Including as library (Shared Memory)

Copy this repository completely into the /include directory. \
Then in code 

```cpp
#include "gtsam_viz/graph.hpp"

int main() {
    factor_graph::VizConfig viz;
    viz.enabled = true;
    viz.publishEveryN = 1;
    viz.backendName = "My SLAM";

    factor_graph::Graph g(viz);

    g.add_pose_prior(gtsam::Pose3::Identity());
    // g.add_pose_between(...)
    // g.add_loop_closure(...)
}
```

---




### 3-D Viewport
| Action | Input |
|--------|-------|
| Orbit | Left-drag |
| Pan | Middle-drag / Right-drag |
| Zoom | Scroll wheel |

### Keyboard
| Key | Action |
|-----|--------|
| `F5` | Run full optimization |
| `F6` | Single optimizer step |
| `F7` | Reset to initial values |
| `Ctrl+D` | Open demo loader |
| `Ctrl+N` | Clear graph |
| `Esc` | Quit |

