#pragma once
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include "gui/GuiManager.h"
#include "renderer/Renderer3D.h"
#include "ipc/GVizServer.h"
#include <string>

namespace gtsam_viz {

struct AppConfig {
    int         width   = 1600;
    int         height  = 950;
    std::string title   = "GTSAMViz – Factor Graph Debugger";
    bool        vsync   = true;
    bool        msaa4   = false;
    float       uiScale = 0.f;  // 0 = auto-detect from monitor/content scale
};

class Application {
public:
    explicit Application(AppConfig cfg = {});
    ~Application();

    bool init();
    int  run();

private:
    void beginFrame();
    void endFrame();
    void processEvents();
    float computeUiScale() const;

    static void glfwErrorCallback(int err, const char* desc);
    static void framebufferSizeCallback(GLFWwindow* w, int width, int height);

    AppConfig   cfg_;
    GLFWwindow* window_  = nullptr;
    Renderer3D  renderer_;
    GuiManager  gui_;
    GVizServer  ipcServer_;
    float       uiScale_ = 1.f;
    bool        running_ = false;
};

} // namespace gtsam_viz
