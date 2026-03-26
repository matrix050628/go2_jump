#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE_TAG="${GO2_JUMP_IMAGE:-go2-jump-humble:latest}"
TARGET_DISTANCE_M="${1:-0.25}"
SOLVER_BACKEND="${GO2_JUMP_SOLVER_BACKEND:-reference_preview}"
ENABLE_LOWCMD_OUTPUT="${GO2_JUMP_ENABLE_LOWCMD_OUTPUT:-false}"
AUTO_START="${GO2_JUMP_AUTO_START:-true}"
CONTAINER_NAME="${GO2_JUMP_STACK_CONTAINER:-go2-jump-mpc-$(date +%s)}"

cleanup() {
  docker rm -f "${CONTAINER_NAME}" >/dev/null 2>&1 || true
}

trap cleanup EXIT INT TERM

docker run --rm --name "${CONTAINER_NAME}" --net host \
  --user "$(id -u):$(id -g)" \
  -e HOME=/tmp \
  -e ROS_LOG_DIR=/tmp/roslog \
  -v "${ROOT_DIR}:/workspace" \
  -w /workspace \
  "${IMAGE_TAG}" \
  bash -lc "source /workspace/scripts/container_source_env.sh && exec stdbuf -oL -eL ros2 launch go2_jump_bringup sim_jump_mpc.launch.py target_distance_m:=${TARGET_DISTANCE_M} solver_backend:=${SOLVER_BACKEND} enable_lowcmd_output:=${ENABLE_LOWCMD_OUTPUT} auto_start:=${AUTO_START}"
