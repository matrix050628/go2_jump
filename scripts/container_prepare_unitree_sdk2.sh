#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="${ROOT_DIR:-/workspace}"
SDK_SRC_DIR="${ROOT_DIR}/third_party/unitree_sdk2"
SDK_INSTALL_PREFIX="${ROOT_DIR}/third_party/unitree_robotics"
SDK_CONFIG_FILE="${SDK_INSTALL_PREFIX}/lib/cmake/unitree_sdk2/unitree_sdk2Config.cmake"

if [ ! -d "${SDK_SRC_DIR}" ]; then
  echo "Missing Unitree SDK2 source tree: ${SDK_SRC_DIR}" >&2
  return 1 2>/dev/null || exit 1
fi

if [ ! -f "${SDK_CONFIG_FILE}" ]; then
  echo "Installing Unitree SDK2 into ${SDK_INSTALL_PREFIX}"
  rm -rf /tmp/unitree_sdk2_build
  cmake -S "${SDK_SRC_DIR}" -B /tmp/unitree_sdk2_build \
    -DBUILD_EXAMPLES=OFF \
    -DCMAKE_INSTALL_PREFIX="${SDK_INSTALL_PREFIX}"
  cmake --build /tmp/unitree_sdk2_build -j"$(nproc)"
  cmake --install /tmp/unitree_sdk2_build
fi

export CMAKE_PREFIX_PATH="${SDK_INSTALL_PREFIX}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
export LD_LIBRARY_PATH="${SDK_INSTALL_PREFIX}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
