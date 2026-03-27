# Research Program

[中文版本](research_program.zh-CN.md)

## 1. Working Hypothesis

The most promising line for this project is no longer a pure template controller
and no longer a pure end-to-end policy.

The working hypothesis is:

an explicit high-level jump planner, combined with a contact-aware MuJoCo-native
whole-body MPC low level, can deliver a controller that is more interpretable,
more robust, and easier to transfer than a monolithic policy.

The intended long-term split is:

`RL planner -> explicit JumpIntent -> contact-aware MuJoCo-native MPC -> LowCmd`

## 2. Stage A: Interface Hardening

Goal:

make the control stack observable and replaceable before pushing performance.

Required deliverables:

- stable `JumpTask` interface
- stable `JumpIntent` interface
- controller diagnostics that record the active intent and backend
- reproducible trial and batch workflows
- clean A/B switch between `no_intent` and `explicit_intent`

This stage is what keeps later RL and MPC work from turning into uncontrolled
trial-and-error.

## 3. Stage B: Explicit Planner Baseline

Goal:

build a solid explicit planner baseline before introducing RL.

Near-term work:

- clean up the heuristic explicit planner
- ensure task reconstruction and intent reconstruction are consistent
- measure how much improvement or regression comes from the planner itself
- tune the planner only after low-level execution variance is understood

The point of this stage is not to keep the heuristic planner forever. The point
is to create a stable baseline and a clean interface contract.

## 4. Stage C: RL High-Level Planner

Goal:

replace the heuristic planner with a policy that still outputs interpretable
kinodynamic intent.

Expected RL outputs:

- takeoff velocity targets
- timing targets
- body attitude targets
- leg retraction and landing preparation terms
- optional foot bias terms

Planned constraints:

- joint range feasibility
- takeoff/landing timing feasibility
- friction-aware intent bounds
- body pitch and touchdown stability targets

The RL layer should improve *intent selection*, not bypass the explicit control
structure.

## 5. Stage D: Stronger Low-Level MPC

Goal:

upgrade the current MuJoCo rollout controller into a stronger low-level
executor.

Near-term work:

- better contact-state consistency between prediction and execution
- better takeoff and touchdown timing handling
- stronger front/rear load redistribution
- better short-horizon cost design for airborne progress and landing quality

Possible medium-term upgrades:

- explicit centroidal momentum terms
- stronger contact-force optimization
- tighter whole-body force-to-joint consistency

## 6. Stage E: Evaluation Protocol

The project needs a fixed evaluation protocol before major tuning.

Core metrics:

- success rate
- total forward distance
- airborne forward distance
- post-touchdown compensation distance
- touchdown pitch and pitch rate
- peak torque and torque limit utilization
- phase cleanliness and contact consistency

Core comparisons:

- no high-level intent
- heuristic explicit planner
- RL explicit planner
- stronger published baselines when reproduction is practical

## 7. Publication Angle

If this line works well, the paper story is not “Go2 can jump.”

The stronger story is:

- distance-conditioned forward jumping on Go2
- most of the translation achieved during flight
- explicit and interpretable high-level jump intent
- contact-aware low-level whole-body MPC execution
- a practical hybrid alternative to both hand-built templates and opaque end-to-end policies
