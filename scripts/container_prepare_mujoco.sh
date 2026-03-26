#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="${ROOT_DIR:-/workspace}"
MUJOCO_TARBALL="${ROOT_DIR}/third_party/mujoco-3.3.6-linux-x86_64.tar.gz"
MUJOCO_EXTRACT_DIR="${ROOT_DIR}/third_party/mujoco-3.3.6"
MUJOCO_LINK_DIR="${ROOT_DIR}/src/unitree_mujoco/simulate/mujoco"

if [ ! -f "${MUJOCO_TARBALL}" ]; then
  echo "Missing MuJoCo tarball: ${MUJOCO_TARBALL}" >&2
  return 1 2>/dev/null || exit 1
fi

if [ ! -d "${MUJOCO_EXTRACT_DIR}" ]; then
  echo "Extracting MuJoCo into ${MUJOCO_EXTRACT_DIR}"
  tar -xf "${MUJOCO_TARBALL}" -C "${ROOT_DIR}/third_party"
fi

ln -snf "${MUJOCO_EXTRACT_DIR}" "${MUJOCO_LINK_DIR}"
export LD_LIBRARY_PATH="${MUJOCO_EXTRACT_DIR}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
