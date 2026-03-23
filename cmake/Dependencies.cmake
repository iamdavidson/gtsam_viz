include(FetchContent)

# ── GLFW ──────────────────────────────────────────────────────────────────────
FetchContent_Declare(glfw
  GIT_REPOSITORY https://github.com/glfw/glfw.git
  GIT_TAG        3.4
  GIT_SHALLOW    TRUE
)
set(GLFW_BUILD_DOCS     OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS    OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL        OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(glfw)

# ── glad ──────────────────────────────────────────────────────────────────────
FetchContent_Declare(glad
  GIT_REPOSITORY https://github.com/Dav1dde/glad.git
  GIT_TAG        v0.1.36
  GIT_SHALLOW    TRUE
)
set(GLAD_PROFILE    "core"  CACHE STRING "" FORCE)
set(GLAD_API        "gl=4.3" CACHE STRING "" FORCE)
set(GLAD_GENERATOR  "c"     CACHE STRING "" FORCE)
set(GLAD_EXTENSIONS ""      CACHE STRING "" FORCE)
FetchContent_MakeAvailable(glad)

# ── ImGui (docking branch) ────────────────────────────────────────────────────
FetchContent_Declare(imgui
  GIT_REPOSITORY https://github.com/ocornut/imgui.git
  GIT_TAG        docking
  GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(imgui)

# Build ImGui as a static library (no CMakeLists.txt upstream)
add_library(imgui STATIC
  ${imgui_SOURCE_DIR}/imgui.cpp
  ${imgui_SOURCE_DIR}/imgui_draw.cpp
  ${imgui_SOURCE_DIR}/imgui_tables.cpp
  ${imgui_SOURCE_DIR}/imgui_widgets.cpp
  ${imgui_SOURCE_DIR}/imgui_demo.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)
target_include_directories(imgui PUBLIC
  ${imgui_SOURCE_DIR}
  ${imgui_SOURCE_DIR}/backends
)
target_link_libraries(imgui PUBLIC glfw glad)
target_compile_definitions(imgui PUBLIC IMGUI_IMPL_OPENGL_LOADER_GLAD)

# ── ImPlot ────────────────────────────────────────────────────────────────────
FetchContent_Declare(implot
  GIT_REPOSITORY https://github.com/epezent/implot.git
  GIT_TAG        master
  GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(implot)

add_library(implot STATIC
  ${implot_SOURCE_DIR}/implot.cpp
  ${implot_SOURCE_DIR}/implot_items.cpp
  ${implot_SOURCE_DIR}/implot_demo.cpp
)
target_include_directories(implot PUBLIC ${implot_SOURCE_DIR})
target_link_libraries(implot PUBLIC imgui)

# ── glm ───────────────────────────────────────────────────────────────────────
FetchContent_Declare(glm
  GIT_REPOSITORY https://github.com/g-truc/glm.git
  GIT_TAG        1.0.1
  GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(glm)

# ── OpenGL ────────────────────────────────────────────────────────────────────
find_package(OpenGL REQUIRED)

# ── Convenience interface target ──────────────────────────────────────────────
add_library(gtsam_viz_deps INTERFACE)
target_link_libraries(gtsam_viz_deps INTERFACE
  OpenGL::GL
  glfw
  glad
  imgui
  implot
  glm::glm
  gtsam
)
