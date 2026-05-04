cmake_minimum_required(VERSION 3.20)
project(GTSAMViz VERSION 1.0.0 LANGUAGES CXX)

# ── C++ Standard ──────────────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ── Build type ────────────────────────────────────────────────────────────────
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# ── Options ───────────────────────────────────────────────────────────────────
option(GTSAMVIZ_BUILD_STANDALONE "Build the gtsam_viz executable"  ON)
option(GTSAMVIZ_ENABLE_ASAN      "Enable AddressSanitizer"         OFF)
option(GTSAMVIZ_ENABLE_TSAN      "Enable ThreadSanitizer"          OFF)

# ── Sanitizers ────────────────────────────────────────────────────────────────
if(GTSAMVIZ_ENABLE_ASAN)
  add_compile_options(-fsanitize=address -fno-omit-frame-pointer)
  add_link_options(-fsanitize=address)
endif()
if(GTSAMVIZ_ENABLE_TSAN)
  add_compile_options(-fsanitize=thread)
  add_link_options(-fsanitize=thread)
endif()

# ── Dependencies ──────────────────────────────────────────────────────────────
include(cmake/Dependencies.cmake)

# ── GTSAM ─────────────────────────────────────────────────────────────────────
find_package(GTSAM REQUIRED)
message(STATUS "GTSAM found: ${GTSAM_VERSION}")

# ── Sources / targets ─────────────────────────────────────────────────────────
add_subdirectory(src)

# ── Consumer-facing alias ─────────────────────────────────────────────────────
# External projects can link against GTSAMViz::lib regardless of how they
# found this package (add_subdirectory vs find_package).
add_library(GTSAMViz::lib ALIAS gtsam_viz_lib)

# ── Integration example ───────────────────────────────────────────────────────
option(GTSAMVIZ_BUILD_EXAMPLE "Build integration/usage_example" OFF)
if(GTSAMVIZ_BUILD_EXAMPLE)
    add_executable(gviz_usage_example integration/usage_example.cpp)
    target_include_directories(gviz_usage_example PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/integration)
    target_link_libraries(gviz_usage_example PRIVATE gtsam)
    set_target_properties(gviz_usage_example PROPERTIES CXX_STANDARD 17)
endif()
