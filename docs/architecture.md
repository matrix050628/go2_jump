# Architecture

[中文版本](architecture.zh-CN.md)

## 1. Main Runtime Path

The active runtime path is:

`JumpTask -> JumpIntent -> WholeBodyMpc -> LowCmd`

This split is the backbone of the repository.

- `JumpTask` is the task-level request
- `JumpIntent` is the explicit high-level jump plan
- `WholeBodyMpc` is the low-level execution layer
- `LowCmd` is the final actuator command path

## 2. Package Responsibilities

### `go2_jump_msgs`

Defines the ROS 2 interfaces used by the stack.

Important messages:

- `JumpTask.msg`
- `JumpIntent.msg`
- `JumpControllerState.msg`

### `go2_jump_core`

Contains task-level utilities that are independent from the solver backend.

Responsibilities:

- build `JumpTaskSpec` from distance-conditioned requests
- normalize and sample phase references
- provide reusable reference logic for planner and controller

### `go2_jump_planner`

High-level planner package.

Responsibilities:

- subscribe to `/go2_jump/task`
- convert the task into an explicit `JumpIntent`
- publish `/go2_jump/intent`

The current backend is heuristic. The long-term role of this package is to host
the RL planner without changing the downstream controller interface.

### `go2_jump_mpc`

Low-level execution package.

Responsibilities:

- subscribe to `JumpTask`
- subscribe to `JumpIntent`
- read robot observation from `LowState` and `SportModeState`
- estimate contact
- manage jump phase transitions
- build executable references
- run the MuJoCo-native backend
- publish `LowCmd`
- publish `/go2_jump/controller_state`

### `go2_jump_bringup`

Launch and parameter package.

Responsibilities:

- shared parameters
- stack launch files
- planner on/off toggles for A/B experiments

## 3. Runtime Flow

1. `go2_jump_core` publishes a `JumpTask`.
2. `go2_jump_planner` converts that task into a `JumpIntent`.
3. `go2_jump_mpc` receives both the task and the intent.
4. The controller aligns the nominal plan with measured contact and measured robot state.
5. The MuJoCo-native MPC backend searches over short-horizon actions.
6. The best first-step action is converted into `LowCmd`.
7. MuJoCo executes the command and publishes the next observation.

## 4. Design Rules

The repository follows four architectural rules.

### Rule 1: keep the planner output explicit

The planner must publish interpretable quantities such as takeoff velocity,
timing, and landing preparation terms.

### Rule 2: keep the low-level execution path stable

The final execution interface remains `LowCmd / LowState`.

That makes it possible to improve the planner without rewriting the transport
layer or the actuator command path.

### Rule 3: keep contact handling inside the low level

The high-level planner should not assume perfect takeoff or touchdown timing.

Measured contact is handled by the low-level controller because that is where
the real execution state is available.

### Rule 4: keep planner and solver comparable

The same low-level controller should support:

- no high-level intent
- heuristic explicit intent
- future RL explicit intent

That is required for fair ablation.

## 5. Current Replacement Points

The main replacement point for future work is:

- replace `heuristic_explicit` inside `go2_jump_planner`

The main low-level upgrade point is:

- strengthen `mujoco_native_mpc` inside `go2_jump_mpc`

The goal is to improve both layers while keeping the boundary between them
stable.
