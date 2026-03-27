# Algorithm

[中文版本](algorithm.zh-CN.md)

This document describes the current control split in engineering terms.

The active runtime path is:

`JumpTask -> JumpIntent -> WholeBodyMpc -> LowCmd`

This is intentional. The high level should decide *what jump to execute*. The
low level should decide *how to realize that jump on the current contact state
and robot state*.

## 1. Problem Statement

The external task is simple:

- input: target forward jump distance
- output: a clean forward jump where most of the translation happens during flight

The controller is not allowed to solve this by replaying a built-in sport-mode
jump. It has to generate its own low-level command sequence.

The two technical constraints that dominate the design are:

- jump timing is contact-sensitive
- the final command must still be issued as low-level joint commands

That is why the stack is split into an explicit planner interface and a
separate low-level execution layer.

## 2. What `JumpTask` Means

`JumpTask` is the user-facing goal.

It carries:

- target distance
- nominal takeoff angle
- nominal takeoff speed or takeoff velocity components
- takeoff and landing body pitch targets
- nominal phase durations

`JumpTask` is deliberately compact. It says what the jump request is, but it
does not yet specify a full executable kinodynamic plan.

## 3. What `JumpIntent` Means

`JumpIntent` is the explicit output of the high-level planner.

It carries the quantities that the low-level controller can execute directly:

- target takeoff `vx` and `vz`
- takeoff and landing pitch targets
- crouch / push / flight / landing / settle durations
- body height offsets for different phases
- leg retraction and landing brace factors
- front/rear foot placement bias terms
- planned apex height
- planner backend name

`JumpIntent` is the contract boundary for the future RL planner.

That is the key design choice of this repository:

the RL layer should output an interpretable, physically meaningful jump intent,
not raw joint torques and not an opaque motion clip.

## 4. Planner: What It Computes

Current implementation:

- package: `go2_jump_planner`
- backend: `heuristic_explicit`

Current input:

- `JumpTask`

Current output:

- one explicit `JumpIntent`

### 4.1 What the current planner actually does

Today the planner is still heuristic. It does not solve a learned policy or a
full trajectory optimization problem yet.

Its job is:

1. read the nominal task
2. build a consistent explicit jump intent
3. publish that intent so the low-level stack works against a stable interface

The current planner already serves an important purpose:

- it keeps the high-level interface explicit
- it allows A/B comparison between `no_intent` and `explicit_intent`
- it gives us a clean replacement point for the future RL planner

### 4.2 What the RL planner is supposed to replace

The future RL planner should replace the *policy that generates `JumpIntent`*,
not the entire stack.

The intended RL output remains:

- takeoff velocity targets
- timing targets
- attitude targets
- leg retraction / landing preparation terms
- optional foot bias terms

In other words, the RL planner should produce a constrained kinodynamic jump
plan, not a raw motor command stream.

## 5. Controller: What It Computes

Current implementation:

- package: `go2_jump_mpc`

The controller runs every control tick and combines:

- the active `JumpTask`
- the active `JumpIntent` if available
- the current robot observation

Its output is the next executable low-level command.

### 5.1 Observation processing

The controller reads:

- joint state from `/lowstate`
- IMU orientation and angular velocity from `/lowstate`
- body pose and body velocity from `/sportmodestate`
- contact proxy from foot-force related fields

Contact estimation is not a cosmetic feature. It is central to the whole jump.

Without contact-aware logic, the controller cannot reliably answer:

- has takeoff already happened
- is the robot still pushing
- has touchdown already started
- is landing complete enough to enter settle

### 5.2 Phase management

The controller does not trust the nominal clock alone.

For each control tick it:

1. samples the nominal reference phase from the active task / intent
2. checks measured contact and body vertical velocity
3. applies phase overrides when the nominal phase and measured contact disagree

This is how the controller prevents obvious mistiming such as:

- entering flight too early while stance contact is still present
- entering landing too early while the robot is still airborne
- missing touchdown because the nominal phase clock is late

### 5.3 Reference construction

After the active phase is decided, the controller builds:

- desired body pitch
- desired body height offset
- desired forward and vertical velocity targets
- a joint-space reference pose
- phase-dependent gains
- feedforward torques

At the code level, this is where the “whole-body control” role currently lives.

This repository does **not** have a separate textbook QP-WBC module yet. The
current controller combines reference generation, phase-aware logic, and MPC
search inside one low-level execution package.

## 6. MuJoCo-native WBC / MPC: What It Computes

Current implementation:

- backend name: `mujoco_native_mpc`

This backend is a MuJoCo rollout MPC.

It is not yet a full centroidal-QP solver, but it already performs the core MPC
operation:

evaluate short-horizon consequences before sending the next low-level command.

### 6.1 State used by the backend

The backend reconstructs a MuJoCo state from the current observation:

- body pose
- body velocity
- joint positions
- joint velocities
- contact condition

### 6.2 Candidate actions

The backend samples a small set of structured action modifiers around the
current phase reference, including:

- push extension strength
- forward drive bias
- lift bias
- body pitch bias
- flight tuck amount
- landing brace amount
- front/rear push redistribution

Each candidate is rolled forward in the MuJoCo model over a short horizon.

### 6.3 Cost terms

The rollout score currently reasons about:

- forward velocity tracking
- vertical velocity tracking
- pitch and roll stability
- phase-consistent contact count
- short-horizon forward progress
- control effort

This is the main low-level optimization in the current project.

### 6.4 What the backend returns

After evaluating all candidates, the backend:

1. keeps the best rollout
2. extracts the first control action only
3. converts that action into joint references and feedforward terms
4. publishes `LowCmd`

This loop repeats every control cycle.

## 7. How Planner, Controller, and WBC/MPC Fit Together

The three layers have different jobs:

- planner: choose an explicit, interpretable jump intent
- controller: align that intent with measured contact and current robot state
- WBC/MPC backend: choose the low-level action that best executes the current intent over a short horizon

That division is the reason we can improve the planner and the low-level
controller separately.

Examples:

- replace `heuristic_explicit` with RL without changing `LowCmd`
- improve low-level MPC without changing the `JumpIntent` contract
- run `no_intent` and `explicit_intent` A/B experiments on the same low-level stack

## 8. Current Limitations

The current stack is real and useful, but it is not the final controller yet.

The main open problems are:

- distance tracking is still inconsistent
- airborne translation is still too small on harder distances
- touchdown quality and post-landing phase cleanliness still vary
- the current planner is still heuristic
- the low-level backend is not yet a full contact-force-optimized centroidal controller

## 9. Near-Term Direction

The next major step is:

`RL planner -> explicit JumpIntent -> contact-aware MuJoCo-native MPC -> LowCmd`

That keeps the system interpretable while letting the high level learn better
jump intent selection than the current heuristic planner.
