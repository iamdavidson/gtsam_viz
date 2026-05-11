#include "Application.h"
#include "globals.h"
#include "gui/panels/LogPanel.h"

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <implot.h>

#include <iostream>
#include <stdexcept>
#include <algorithm>

namespace gtsam_viz {

Application::Application(AppConfig cfg) : cfg_(std::move(cfg)) {}

Application::~Application() {
    ipcServer_.stop();
    renderer_.shutdown();
    if (ImPlot::GetCurrentContext())  ImPlot::DestroyContext();
    if (ImGui::GetCurrentContext()) {
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
    }
    if (window_) glfwDestroyWindow(window_);
    glfwTerminate();
}

bool Application::init() {
    // ── GLFW ──────────────────────────────────────────────────────────────────
    glfwSetErrorCallback(glfwErrorCallback);
    if (!glfwInit()) {
        std::cerr << "[GTSAMViz] Failed to initialize GLFW\n";
        return false;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif
    if (cfg_.msaa4) glfwWindowHint(GLFW_SAMPLES, 4);

    window_ = glfwCreateWindow(cfg_.width, cfg_.height, cfg_.title.c_str(),
                                nullptr, nullptr);
    if (!window_) {
        std::cerr << "[GTSAMViz] Failed to create GLFW window\n";
        glfwTerminate();
        return false;
    }

    glfwMakeContextCurrent(window_);
    glfwSwapInterval(cfg_.vsync ? 1 : 0);
    glfwSetFramebufferSizeCallback(window_, framebufferSizeCallback);
    glfwSetWindowUserPointer(window_, this);

    // ── glad ──────────────────────────────────────────────────────────────────
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "[GTSAMViz] Failed to initialize glad\n";
        return false;
    }
    std::cout << "[GTSAMViz] OpenGL " << glGetString(GL_VERSION) << "\n";
    std::cout << "[GTSAMViz] Renderer: " << glGetString(GL_RENDERER) << "\n";

    // ── ImGui ─────────────────────────────────────────────────────────────────
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();

    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;

    uiScale_ = computeUiScale();
    ImFontConfig fontCfg;
    fontCfg.SizePixels = 16.f * uiScale_;
    fontCfg.OversampleH = 2;
    fontCfg.OversampleV = 2;
    io.FontDefault = io.Fonts->AddFontDefault(&fontCfg);
    io.FontGlobalScale = 1.f;

    ImGui_ImplGlfw_InitForOpenGL(window_, true);
    ImGui_ImplOpenGL3_Init("#version 430");

    // ── Renderer ──────────────────────────────────────────────────────────────
    if (!renderer_.init()) {
        std::cerr << "[GTSAMViz] Renderer3D initialization failed\n";
        return false;
    }

    // ── GUI ───────────────────────────────────────────────────────────────────
    if (!gui_.init(renderer_, ipcServer_, uiScale_)) {
        std::cerr << "[GTSAMViz] GuiManager initialization failed\n";
        return false;
    }

    // ── IPC Server ────────────────────────────────────────────────────────────
    if (!ipcServer_.start()) {
        std::cerr << "[GTSAMViz] Warning: IPC server could not start (socket in use?)\n";
        // Non-fatal — GUI still works without a backend
    }

    GVLOG_INFO("Application started.");
    running_ = true;
    return true;
}

int Application::run() {
    if (!running_) return -1;

    while (!glfwWindowShouldClose(window_) && !g_requestQuit) {
        processEvents();
        beginFrame();
        gui_.draw();
        endFrame();
    }
    return 0;
}

void Application::beginFrame() {
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
}

void Application::endFrame() {
    ImGui::Render();

    int w, h;
    glfwGetFramebufferSize(window_, &w, &h);
    glViewport(0, 0, w, h);
    glClearColor(0.06f, 0.07f, 0.09f, 1.f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    // Multi-viewport support
    ImGuiIO& io = ImGui::GetIO();
    if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
        GLFWwindow* backup = glfwGetCurrentContext();
        ImGui::UpdatePlatformWindows();
        ImGui::RenderPlatformWindowsDefault();
        glfwMakeContextCurrent(backup);
    }

    glfwSwapBuffers(window_);
}

void Application::processEvents() {
    glfwPollEvents();

    // Global keyboard shortcuts
    ImGuiIO& io = ImGui::GetIO();
    if (!io.WantCaptureKeyboard) {
        if (glfwGetKey(window_, GLFW_KEY_ESCAPE) == GLFW_PRESS)
            glfwSetWindowShouldClose(window_, GLFW_TRUE);
    }
}

float Application::computeUiScale() const {
    if (cfg_.uiScale > 0.f)
        return std::clamp(cfg_.uiScale, 0.75f, 2.5f);

    float xscale = 1.f, yscale = 1.f;
    if (window_)
        glfwGetWindowContentScale(window_, &xscale, &yscale);

    float monitorScale = 1.f;
    GLFWmonitor* monitor = window_ ? glfwGetWindowMonitor(window_) : nullptr;
    if (!monitor)
        monitor = glfwGetPrimaryMonitor();
    if (monitor) {
        if (const GLFWvidmode* mode = glfwGetVideoMode(monitor)) {
            float sx = static_cast<float>(mode->width) / 1920.f;
            float sy = static_cast<float>(mode->height) / 1080.f;
            monitorScale = std::min(sx, sy);
        }
    }

    return std::clamp(std::max({xscale, yscale, monitorScale}), 1.f, 1.75f);
}

void Application::glfwErrorCallback(int err, const char* desc) {
    std::cerr << "[GLFW] Error " << err << ": " << desc << "\n";
}

void Application::framebufferSizeCallback(GLFWwindow*, int w, int h) {
    glViewport(0, 0, w, h);
}

} // namespace gtsam_viz
