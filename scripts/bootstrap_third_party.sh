#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MUJOCO_TARBALL="${ROOT_DIR}/third_party/mujoco-3.3.6-linux-x86_64.tar.gz"
UNITREE_SDK_SRC_DIR="${ROOT_DIR}/third_party/unitree_sdk2"

if [ ! -f "${MUJOCO_TARBALL}" ]; then
  echo "Missing ${MUJOCO_TARBALL}" >&2
  exit 1
fi

if [ ! -d "${UNITREE_SDK_SRC_DIR}" ]; then
  echo "Missing ${UNITREE_SDK_SRC_DIR}" >&2
  exit 1
fi

echo "Third-party cache looks ready:"
echo "  MuJoCo tarball: ${MUJOCO_TARBALL}"
echo "  Unitree SDK2 source: ${UNITREE_SDK_SRC_DIR}"
