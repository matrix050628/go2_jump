# Research Program

## Research Hypothesis

A MuJoCo-native whole-body MPC controller, coupled with contact-aware landing,
can produce cleaner airborne-dominant forward jumps on Go2 than template-based
joint-space controllers.

## Phase 1

Reset the repository around the new architecture.

Deliverables:

- task-level jump specification
- MPC package boundary
- reproducible dependency setup
- clean launch path for the new mainline
- verified `/lowstate -> JumpTask -> controller_state -> /lowcmd` chain

## Phase 2

Build the first closed-loop MPC backend.

Minimum requirements:

- consume `JumpTask`
- build a receding-horizon reference
- produce low-level outputs at the same external ROS 2 interface
- expose solver diagnostics and phase transitions

## Phase 3

Upgrade from preview/reference mode to a MuJoCo-native solver backend.

Near-term targets:

- horizon rollout using MuJoCo dynamics
- task cost on takeoff velocity, body pitch, landing stability, and settle state
- explicit torque and state regularization
- touchdown-aware horizon adaptation

## Phase 4

Paper-grade experiments.

Required comparisons:

- whole-body MPC mainline
- centroidal / QP fallback baseline
- strong published baselines where reproduction is practical

Required metrics:

- success rate
- airborne forward displacement
- post-landing compensation distance
- touchdown pitch and touchdown pitch rate
- peak torque and estimated energy proxy

## Publication Direction

This repository is being rebuilt so the final work can support a paper story
centered on:

- distance-conditioned forward jumping
- airborne-dominant motion quality
- clean and reactive touchdown
- low-level control on Go2
- MuJoCo-native whole-body optimization
