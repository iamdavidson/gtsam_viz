#pragma once

namespace gtsam_viz {

/// Set to true from any thread to request a clean shutdown of the main loop.
/// Defined once in Application.cpp; declared here so both Application.cpp
/// and GuiManager.cpp can reference it inside the gtsam_viz namespace
/// without triggering the "undefined reference to gtsam_viz::g_requestQuit"
/// linker error that occurs when the variable is defined at global (::) scope
/// while extern-declarations appear inside a namespace block.
extern bool g_requestQuit;

} // namespace gtsam_viz
