# Architecture

## Goal

The repository is now organized around a single mainline:

`distance-conditioned jump task -> whole-body MPC -> LowCmd`

This split is meant to keep the research stack modular enough for ablations and
paper-grade comparisons.

## Packages

### `go2_jump_msgs`

ROS 2 interfaces used to move task-level intent through the stack.

The first interface is `JumpTask.msg`, which carries a distance-conditioned jump
objective together with the derived takeoff and timing targets.

### `go2_jump_core`

Pure task-generation logic.

Responsibilities:

- map target distance into takeoff velocity targets
- estimate nominal flight timing
- define high-level phase targets for crouch, push, flight, landing, and settle
- publish the active `JumpTask`

This package is intentionally lightweight. It should remain usable by both the
future MPC controller and comparison baselines.

### `go2_jump_mpc`

Mainline controller package.

Responsibilities:

- receive `JumpTask`
- build a receding-horizon preview
- estimate contact state from low-level observation
- override nominal phase timing when measured contact disagrees with the plan
- manage MPC configuration and backend selection
- publish `LowCmd`
- publish `/go2_jump/controller_state` diagnostics useful for solver bring-up

The package is designed so the preview/reference backend can be replaced by a
MuJoCo-native solver backend without changing the outer ROS 2 interfaces.

### `go2_jump_bringup`

Launch files and shared parameters for the new stack.

The bringup package is intentionally thin. The controller logic lives in
`go2_jump_mpc`, not in launch-time Python.

## Runtime Flow

1. `go2_jump_core` builds a `JumpTask` from the requested distance.
2. `go2_jump_mpc` receives the task and the current robot state.
3. The MPC layer estimates contact state, applies contact-aware phase logic,
   then builds a horizon-level preview and solves for the current control
   command.
4. The controller publishes `LowCmd`.
5. MuJoCo executes the command and feeds the next optimization step.

## Design Principle

The task layer should be stable while the solver layer evolves.

That allows:

- switching between preview, centroidal-QP, and MuJoCo-native backends
- comparing methods fairly on the same task interface
- preserving experiment reproducibility as the controller matures
