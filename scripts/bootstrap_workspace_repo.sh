#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PATCH_FILE="${ROOT_DIR}/patches/unitree_mujoco/0001-go2-mpc-bridge-base.patch"

git -C "${ROOT_DIR}" submodule sync --recursive
git -C "${ROOT_DIR}" submodule update --init --recursive

if [ -f "${PATCH_FILE}" ]; then
  if git -C "${ROOT_DIR}/src/unitree_mujoco" apply --check "${PATCH_FILE}" >/dev/null 2>&1; then
    git -C "${ROOT_DIR}/src/unitree_mujoco" apply "${PATCH_FILE}"
    echo "Applied Unitree MuJoCo compatibility patch."
  elif git -C "${ROOT_DIR}/src/unitree_mujoco" apply --reverse --check "${PATCH_FILE}" >/dev/null 2>&1; then
    echo "Unitree MuJoCo compatibility patch already present."
  else
    echo "Unitree MuJoCo patch could not be applied cleanly. Check local submodule state." >&2
  fi
fi
