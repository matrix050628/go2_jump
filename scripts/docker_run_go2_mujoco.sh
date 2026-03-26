#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE_TAG="${GO2_JUMP_IMAGE:-go2-jump-humble:latest}"
CONTAINER_NAME="${GO2_JUMP_SIM_CONTAINER:-go2-jump-mujoco-$(date +%s)}"

DOCKER_ARGS=(
  --rm
  --net host
  --privileged
  -v "${ROOT_DIR}:/workspace"
  -w /workspace
)

if [ -n "${DISPLAY:-}" ]; then
  DOCKER_ARGS+=(-e "DISPLAY=${DISPLAY}" -v /tmp/.X11-unix:/tmp/.X11-unix:rw)
  if [ -n "${XAUTHORITY:-}" ] && [ -f "${XAUTHORITY}" ]; then
    DOCKER_ARGS+=(-e XAUTHORITY=/tmp/.docker.xauth -v "${XAUTHORITY}:/tmp/.docker.xauth:ro")
  fi
  LAUNCH_CMD='source /workspace/scripts/container_source_env.sh && source /workspace/scripts/container_prepare_unitree_sdk2.sh && source /workspace/scripts/container_prepare_mujoco.sh && cd /workspace/src/unitree_mujoco/simulate/build && exec ./unitree_mujoco -r go2 -s scene.xml'
else
  DOCKER_ARGS+=(-e UNITREE_MUJOCO_HEADLESS=1)
  LAUNCH_CMD='source /workspace/scripts/container_source_env.sh; source /workspace/scripts/container_prepare_unitree_sdk2.sh; source /workspace/scripts/container_prepare_mujoco.sh; unset XAUTHORITY; cd /workspace/src/unitree_mujoco/simulate/build; XVFB_DISPLAY="${UNITREE_MUJOCO_XVFB_DISPLAY:-:199}"; Xvfb "${XVFB_DISPLAY}" -screen 0 1280x720x24 -nolisten tcp -ac >/tmp/unitree_mujoco_xvfb.log 2>&1 & xvfb_pid=$!; trap "kill ${xvfb_pid} 2>/dev/null || true" EXIT; export DISPLAY="${XVFB_DISPLAY}"; sleep 1; exec ./unitree_mujoco -r go2 -s scene.xml'
fi

cleanup() {
  docker rm -f "${CONTAINER_NAME}" >/dev/null 2>&1 || true
}

trap cleanup EXIT INT TERM

docker run --name "${CONTAINER_NAME}" "${DOCKER_ARGS[@]}" "${IMAGE_TAG}" bash -lc "${LAUNCH_CMD}"
