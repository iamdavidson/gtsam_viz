#include "globals.h"

namespace gtsam_viz {

/// One-and-only definition lives here.
/// Declared extern in globals.h → visible to Application.cpp, GuiManager.cpp,
/// and any other TU that includes globals.h without causing ODR violations.
bool g_requestQuit = false;

} // namespace gtsam_viz
