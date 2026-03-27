#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE_TAG="${GO2_JUMP_IMAGE:-go2-jump-humble:latest}"

"${ROOT_DIR}/scripts/bootstrap_workspace_repo.sh"
"${ROOT_DIR}/scripts/bootstrap_third_party.sh"

docker run --rm --net host \
  --user "$(id -u):$(id -g)" \
  -e HOME=/tmp \
  -e ROS_LOG_DIR=/tmp/roslog \
  -v "${ROOT_DIR}:/workspace" \
  -w /workspace \
  "${IMAGE_TAG}" \
  bash -lc '
    set -euo pipefail

    set +u
    source /opt/ros/humble/setup.bash
    set -u
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export COLCON_LOG_PATH=/tmp/colcon_logs
    export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"lo\" priority=\"default\" multicast=\"default\" /></Interfaces></General></Domain></CycloneDDS>"

    cd /workspace/src/unitree_ros2/cyclonedds_ws/src
    [ -d rmw_cyclonedds ] || git clone --depth 1 -b humble https://github.com/ros2/rmw_cyclonedds.git
    [ -d cyclonedds ] || git clone --depth 1 -b releases/0.10.x https://github.com/eclipse-cyclonedds/cyclonedds.git

    cd /workspace/src/unitree_ros2/cyclonedds_ws
    export LD_LIBRARY_PATH=/opt/ros/humble/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
    colcon build --symlink-install --packages-select cyclonedds
    set +u
    source /workspace/src/unitree_ros2/cyclonedds_ws/install/setup.bash
    set -u
    colcon build --symlink-install --packages-select unitree_go unitree_hg unitree_api

    cd /workspace
    source /workspace/scripts/container_source_env.sh
    colcon build --symlink-install \
      --packages-ignore unitree_api unitree_go unitree_hg \
      --packages-select go2_jump_msgs go2_jump_core go2_jump_planner go2_jump_mpc go2_jump_bringup

    source /workspace/scripts/container_prepare_unitree_sdk2.sh
    source /workspace/scripts/container_prepare_mujoco.sh

    cd /workspace/src/unitree_mujoco/simulate
    rm -rf build
    mkdir -p build
    cd build
    cmake ..
    make -j"$(nproc)"
  '
