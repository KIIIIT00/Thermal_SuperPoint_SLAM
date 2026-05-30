#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SLAM_DIR="$ROOT_DIR/thirdparty/SuperPoint_SLAM"
JOBS="${JOBS:-$(nproc)}"

ASAN_FLAGS="-O1 -g -fno-omit-frame-pointer -fsanitize=address,undefined"

cmake -S "$SLAM_DIR/Thirdparty/DBoW3" -B "$SLAM_DIR/Thirdparty/DBoW3/build_asan"   -DCMAKE_BUILD_TYPE=RelWithDebInfo   -DCMAKE_C_FLAGS_RELWITHDEBINFO="$ASAN_FLAGS"   -DCMAKE_CXX_FLAGS_RELWITHDEBINFO="$ASAN_FLAGS"
cmake --build "$SLAM_DIR/Thirdparty/DBoW3/build_asan" -j"$JOBS"

cmake -S "$SLAM_DIR/Thirdparty/g2o" -B "$SLAM_DIR/Thirdparty/g2o/build_asan"   -DCMAKE_BUILD_TYPE=RelWithDebInfo   -DCMAKE_C_FLAGS_RELWITHDEBINFO="$ASAN_FLAGS"   -DCMAKE_CXX_FLAGS_RELWITHDEBINFO="$ASAN_FLAGS"
cmake --build "$SLAM_DIR/Thirdparty/g2o/build_asan" -j"$JOBS"

cmake -S "$SLAM_DIR" -B "$SLAM_DIR/build_asan"   -DCMAKE_BUILD_TYPE=RelWithDebInfo   -DCMAKE_C_FLAGS_RELWITHDEBINFO="$ASAN_FLAGS"   -DCMAKE_CXX_FLAGS_RELWITHDEBINFO="$ASAN_FLAGS"   ${Pangolin_DIR:+-DPangolin_DIR="$Pangolin_DIR"}   ${CMAKE_PREFIX_PATH:+-DCMAKE_PREFIX_PATH="$CMAKE_PREFIX_PATH"}
cmake --build "$SLAM_DIR/build_asan" -j"$JOBS"

echo "ASan build completed: $SLAM_DIR/build_asan"
