#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SLAM_DIR="$ROOT_DIR/thirdparty/SuperPoint_SLAM"
BUILD_TYPE="${BUILD_TYPE:-Release}"
JOBS="${JOBS:-$(nproc)}"

cmake -S "$SLAM_DIR/Thirdparty/DBoW3" -B "$SLAM_DIR/Thirdparty/DBoW3/build" \
  -DCMAKE_BUILD_TYPE="$BUILD_TYPE"
cmake --build "$SLAM_DIR/Thirdparty/DBoW3/build" -j"$JOBS"

cmake -S "$SLAM_DIR/Thirdparty/g2o" -B "$SLAM_DIR/Thirdparty/g2o/build" \
  -DCMAKE_BUILD_TYPE="$BUILD_TYPE"
cmake --build "$SLAM_DIR/Thirdparty/g2o/build" -j"$JOBS"

cmake -S "$SLAM_DIR" -B "$SLAM_DIR/build" \
  -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
  ${Pangolin_DIR:+-DPangolin_DIR="$Pangolin_DIR"} \
  ${CMAKE_PREFIX_PATH:+-DCMAKE_PREFIX_PATH="$CMAKE_PREFIX_PATH"}
cmake --build "$SLAM_DIR/build" -j"$JOBS"
