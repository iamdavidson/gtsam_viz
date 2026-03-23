set(LIB_SOURCES
    Application.cpp
    globals.cpp
    gui/GuiManager.cpp
    gui/panels/OptimizerPanel.cpp
    gui/panels/LogPanel.cpp
    gui/panels/Viewport3DPanel.cpp
    graph/FactorGraphState.cpp
    graph/GraphLayout.cpp
    renderer/Renderer3D.cpp
    bridge/GraphBridge.cpp
    ipc/GVizServer.cpp
    examples/DemoScenarios.cpp
    examples/LiveBackendExample.cpp
)

add_library(gtsam_viz_lib STATIC ${LIB_SOURCES})

target_include_directories(gtsam_viz_lib
    PUBLIC  ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(gtsam_viz_lib
    PUBLIC  gtsam_viz_deps
)
target_compile_options(gtsam_viz_lib PRIVATE
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
)

add_executable(gtsam_viz main.cpp)
target_link_libraries(gtsam_viz PRIVATE gtsam_viz_lib)

install(TARGETS gtsam_viz_lib ARCHIVE DESTINATION lib)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
    DESTINATION include/gtsam_viz
    FILES_MATCHING PATTERN "*.h"
)
