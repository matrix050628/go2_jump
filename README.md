# Go2 Jump Workspace

[中文文档](README.zh-CN.md)

This repository is a development workspace for one specific control problem:

build a distance-conditioned forward jump controller for Unitree Go2 on the
`LowCmd / LowState` interface, with `unitree_mujoco` as the primary simulator
and a MuJoCo-native whole-body MPC line as the low-level execution path.

The current mainline is:

`JumpTask -> JumpIntent -> WholeBodyMpc -> LowCmd`

The repository is organized for controller development, not for sport-mode
replay. The jump has to be produced by our own planner and low-level control
stack.

## What Is In Scope

- primary simulator: `unitree_mujoco`
- transport layer: `unitree_ros2`
- task-level goal: target forward jump distance
- explicit high-level interface: `JumpIntent`
- low-level execution interface: `LowCmd`
- active backend: `mujoco_native_mpc`

## Repository Layout

- `src/unitree_ros2`
  Upstream ROS 2 transport dependency from Unitree.
- `src/unitree_mujoco`
  Upstream MuJoCo simulator used as the main validation platform.
- `src/go2_jump_msgs`
  ROS 2 interfaces for `JumpTask`, `JumpIntent`, and controller diagnostics.
- `src/go2_jump_core`
  Distance-conditioned task construction and reference utilities.
- `src/go2_jump_planner`
  High-level planner that converts `JumpTask` into explicit `JumpIntent`.
- `src/go2_jump_mpc`
  Contact estimation, phase management, reference building, and MuJoCo-native MPC.
- `src/go2_jump_bringup`
  Launch files and shared parameter sets.
- `scripts/`
  Docker build, run, trial, and batch helpers.
- `reports/`
  Trial reports and aggregated batch results.

## Current State

What works today:

- headless `unitree_mujoco` runs reliably inside the Docker workflow
- `/lowstate`, `/sportmodestate`, and `/lowcmd` are on the real simulation path
- `JumpTask -> JumpIntent -> WholeBodyMpc` is wired end to end
- the planner can be enabled or disabled from launch and from the helper scripts
- trial reports now record both the solver backend and the active planner intent

What is still under active development:

- distance accuracy is still below target
- run-to-run variance is still large
- the current explicit planner is still heuristic; RL is the intended next step
- the current “whole-body MPC” backend is a MuJoCo rollout controller, not yet a full centroidal-QP stack

## Quick Start

### 1. Build

```bash
cd /home/hayan/go2_jump_ws
./scripts/bootstrap_workspace_repo.sh
./scripts/bootstrap_third_party.sh
./scripts/docker_build_image.sh
./scripts/docker_build_workspace.sh
```

### 2. Run the simulator with GUI

```bash
./scripts/docker_run_go2_mujoco.sh
```

If the host display is configured correctly, this is the easiest way to inspect
the jump visually.

### 3. Launch the jump stack

Default development path:

```bash
GO2_JUMP_SOLVER_BACKEND=mujoco_native_mpc \
GO2_JUMP_ENABLE_LOWCMD_OUTPUT=true \
./scripts/docker_launch_jump_mpc.sh 0.25
```

Disable the explicit planner and let the low-level stack run directly from
`JumpTask`:

```bash
GO2_JUMP_SOLVER_BACKEND=mujoco_native_mpc \
GO2_JUMP_ENABLE_LOWCMD_OUTPUT=true \
GO2_JUMP_ENABLE_INTENT_PLANNER=false \
./scripts/docker_launch_jump_mpc.sh 0.25
```

### 4. Run one instrumented trial

```bash
./scripts/docker_run_single_jump_trial.sh 0.25
```

The script writes a report under:

`reports/trials/<timestamp>_d<distance>_<solver>_<planner>/`

Each report contains:

- `summary.json`
- `stack.log`
- `sim.log`
- `recorder.log`

### 5. Run a small A/B batch

Compare `no_intent` against `heuristic_explicit`:

```bash
GO2_JUMP_BATCH_REPEATS=1 \
GO2_JUMP_BATCH_DISTANCES="0.25 0.30" \
GO2_JUMP_BATCH_INTENT_MODES="disabled enabled" \
./scripts/docker_run_jump_batch.sh
```

The aggregate report is written to:

`reports/batches/<timestamp>/aggregate.json`

## Reading Trial Reports

The trial summary is meant to answer four questions quickly:

1. Which low-level backend ran
2. Which planner produced the active intent
3. How much forward motion happened during flight
4. Whether the phase sequence and contact sequence stayed clean

The most useful fields today are:

- `backends.solver_backend`
- `backends.planner_backend`
- `planner.target_takeoff_velocity_x_mps`
- `planner.target_takeoff_velocity_z_mps`
- `motion.airborne_distance_m`
- `motion.post_touchdown_distance_m`
- `phase.has_flight_relapse_after_landing`
- `command_effort.push_joint_limit_utilization`

## Recommended Reading Order

1. Read [Algorithm](algorithm.md) for the control split.
2. Read [Architecture](docs/architecture.md) for package responsibilities.
3. Read [Research Program](docs/research_program.md) for the next implementation stages.

## Development Notes

- Docker is the reference runtime.
- `mujoco_native_mpc` is the active low-level backend.
- `heuristic_explicit` is a placeholder high-level planner, not the final planner.
- The intended long-term split is:
  RL high-level planner outputs `JumpIntent`, low-level MPC executes it through the same `LowCmd` path.
