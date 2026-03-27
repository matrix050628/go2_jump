# Go2 Jump 工作区

[English](README.md)

这个仓库只围绕一件事展开：

在 `LowCmd / LowState` 接口上，为宇树 Go2 搭建一个“按目标距离起跳”的前跳控制器，并且以 `unitree_mujoco` 作为主验证平台，以 MuJoCo-native whole-body MPC 作为低层执行主线。

当前主线结构是：

`JumpTask -> JumpIntent -> WholeBodyMpc -> LowCmd`

这里不走 sport mode 动作复用路线。跳跃动作必须由我们自己的规划器和低层控制链生成。

## 当前范围

- 主仿真平台：`unitree_mujoco`
- 通信链路：`unitree_ros2`
- 任务目标：按指定前跳距离执行前跳
- 高层显式接口：`JumpIntent`
- 低层执行接口：`LowCmd`
- 当前主 backend：`mujoco_native_mpc`

## 仓库结构

- `src/unitree_ros2`
  Unitree 官方 ROS 2 通信依赖。
- `src/unitree_mujoco`
  Unitree 官方 MuJoCo 仿真，用作主验证平台。
- `src/go2_jump_msgs`
  `JumpTask`、`JumpIntent` 和控制器诊断消息。
- `src/go2_jump_core`
  距离到任务的转换逻辑，以及参考轨迹采样工具。
- `src/go2_jump_planner`
  高层规划器，把 `JumpTask` 转成显式 `JumpIntent`。
- `src/go2_jump_mpc`
  接触估计、相位管理、参考构造和 MuJoCo-native MPC。
- `src/go2_jump_bringup`
  launch 文件和共享参数。
- `scripts/`
  Docker 构建、运行、单次试验、批量试验脚本。
- `reports/`
  单次试验结果和批量统计结果。

## 当前进展

已经打通的部分：

- `unitree_mujoco` 可以通过 Docker 稳定启动
- `/lowstate`、`/sportmodestate`、`/lowcmd` 都在真实仿真链路上
- `JumpTask -> JumpIntent -> WholeBodyMpc` 已经端到端接通
- planner 可以通过 launch 和脚本显式开启或关闭
- 单次 trial 报告已经能记录 solver backend 和 active intent

还在持续推进的部分：

- 跳跃距离精度还不够
- 结果的批次一致性还不够好
- 当前高层 planner 仍然是启发式版本，RL 是下一阶段主线
- 当前 low-level backend 是 MuJoCo rollout MPC，还不是完整的 centroidal + QP/WBC 体系

## 快速开始

### 1. 构建

```bash
cd /home/hayan/go2_jump_ws
./scripts/bootstrap_workspace_repo.sh
./scripts/bootstrap_third_party.sh
./scripts/docker_build_image.sh
./scripts/docker_build_workspace.sh
```

### 2. 启动带画面的 MuJoCo 仿真

```bash
./scripts/docker_run_go2_mujoco.sh
```

如果宿主机 `DISPLAY` 配置正常，这是最直接的看动作方式。

### 3. 启动跳跃控制栈

默认开发主线：

```bash
GO2_JUMP_SOLVER_BACKEND=mujoco_native_mpc \
GO2_JUMP_ENABLE_LOWCMD_OUTPUT=true \
./scripts/docker_launch_jump_mpc.sh 0.25
```

关闭显式 planner，让 low-level 直接从 `JumpTask` 工作：

```bash
GO2_JUMP_SOLVER_BACKEND=mujoco_native_mpc \
GO2_JUMP_ENABLE_LOWCMD_OUTPUT=true \
GO2_JUMP_ENABLE_INTENT_PLANNER=false \
./scripts/docker_launch_jump_mpc.sh 0.25
```

### 4. 运行一次带记录的试验

```bash
./scripts/docker_run_single_jump_trial.sh 0.25
```

输出目录：

`reports/trials/<timestamp>_d<distance>_<solver>_<planner>/`

每次 trial 会保存：

- `summary.json`
- `stack.log`
- `sim.log`
- `recorder.log`

### 5. 运行小规模 A/B 对比

对比 `no_intent` 和 `heuristic_explicit`：

```bash
GO2_JUMP_BATCH_REPEATS=1 \
GO2_JUMP_BATCH_DISTANCES="0.25 0.30" \
GO2_JUMP_BATCH_INTENT_MODES="disabled enabled" \
./scripts/docker_run_jump_batch.sh
```

批量统计结果输出到：

`reports/batches/<timestamp>/aggregate.json`

## 如何看 Trial 报告

当前 summary 主要回答四个问题：

1. 这次运行的是哪个 low-level backend
2. active intent 是由哪个 planner 生成的
3. 前移主要发生在腾空阶段还是落地后补位
4. 相位切换和接触切换是否干净

最常用的字段有：

- `backends.solver_backend`
- `backends.planner_backend`
- `planner.target_takeoff_velocity_x_mps`
- `planner.target_takeoff_velocity_z_mps`
- `motion.airborne_distance_m`
- `motion.post_touchdown_distance_m`
- `phase.has_flight_relapse_after_landing`
- `command_effort.push_joint_limit_utilization`

## 推荐阅读顺序

1. 先看 [Algorithm](algorithm.zh-CN.md)，理解控制栈怎么分层。
2. 再看 [Architecture](docs/architecture.zh-CN.md)，理解各个包负责什么。
3. 最后看 [Research Program](docs/research_program.zh-CN.md)，了解后续迭代方向。

## 开发说明

- Docker 是当前标准运行环境。
- `mujoco_native_mpc` 是当前低层主线。
- `heuristic_explicit` 只是高层显式 planner 的占位版本，不是最终版本。
- 长期目标是：
  高层 RL 输出 `JumpIntent`，低层 MPC 通过同一条 `LowCmd` 链执行。
