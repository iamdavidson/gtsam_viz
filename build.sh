#!/usr/bin/env bash
# ──────────────────────────────────────────────────────────────────────────────
# GTSAMViz build helper
# Usage: ./build.sh [debug|release|asan]
# ──────────────────────────────────────────────────────────────────────────────
set -euo pipefail

MODE="${1:-release}"
BUILD_DIR="build_${MODE}"

CMAKE_ARGS=(-DCMAKE_EXPORT_COMPILE_COMMANDS=ON)

case "$MODE" in
  debug)
    CMAKE_ARGS+=(-DCMAKE_BUILD_TYPE=Debug)
    ;;
  release)
    CMAKE_ARGS+=(-DCMAKE_BUILD_TYPE=Release)
    ;;
  asan)
    CMAKE_ARGS+=(-DCMAKE_BUILD_TYPE=RelWithDebInfo -DGTSAMVIZ_ENABLE_ASAN=ON)
    ;;
  *)
    echo "Unknown mode '$MODE'. Use: debug | release | asan"
    exit 1
    ;;
esac

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  GTSAMViz build  [${MODE}]"
echo "  Build dir: ${BUILD_DIR}"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

mkdir -p "${SCRIPT_DIR}/${BUILD_DIR}"
cd "${SCRIPT_DIR}/${BUILD_DIR}"

cmake "${SCRIPT_DIR}" "${CMAKE_ARGS[@]}"
cmake --build . --parallel 1

# Create symlink to compile_commands.json for clangd
ln -sf "${BUILD_DIR}/compile_commands.json" "${SCRIPT_DIR}/compile_commands.json" 2>/dev/null || true

echo ""
echo "✓ Build successful!"
echo "  Binary: ${BUILD_DIR}/src/gtsam_viz"
echo ""
echo "  Run: cd ${BUILD_DIR} && ./src/gtsam_viz"
