#include "Application.h"
#include "globals.h"
#include <iostream>

int main(int /*argc*/, char** /*argv*/) {
    gtsam_viz::AppConfig cfg;
    cfg.width   = 1600;
    cfg.height  = 950;
    cfg.title   = "GTSAMViz  –  Factor Graph Debugger & Visualizer";
    cfg.vsync   = true;
    cfg.msaa4   = false;

    gtsam_viz::Application app(cfg);

    if (!app.init()) {
        std::cerr << "[GTSAMViz] Startup failed.\n";
        return -1;
    }

    return app.run();
}
