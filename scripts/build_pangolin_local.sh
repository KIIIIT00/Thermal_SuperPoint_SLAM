#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PANGOLIN_DIR="${PANGOLIN_DIR:-$ROOT_DIR/thirdparty/thermal-kd-superpoint/third_party/Pangolin}"
INSTALL_DIR="${INSTALL_DIR:-$PANGOLIN_DIR/install}"
BUILD_DIR="${BUILD_DIR:-$PANGOLIN_DIR/build}"
BUILD_TYPE="${BUILD_TYPE:-Release}"
JOBS="${JOBS:-$(nproc)}"

cmake -S "$PANGOLIN_DIR" -B "$BUILD_DIR" \
  -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
  -DCMAKE_INSTALL_PREFIX="$INSTALL_DIR" \
  -DBUILD_EXAMPLES=OFF \
  -DBUILD_TESTS=OFF \
  -DBUILD_TOOLS=OFF \
  -DPangolin_BUILD_EXAMPLES=OFF \
  -DPangolin_BUILD_TESTS=OFF \
  -DPangolin_BUILD_TOOLS=OFF
cmake --build "$BUILD_DIR" -j"$JOBS"
cmake --install "$BUILD_DIR"

cat <<MSG

Pangolin installed to:
  $INSTALL_DIR

Build SuperPoint_SLAM with:
  Pangolin_DIR="$INSTALL_DIR/lib/cmake/Pangolin" CMAKE_PREFIX_PATH="$INSTALL_DIR" scripts/build_superpoint_slam_ubuntu2404.sh
MSG
