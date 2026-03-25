# 调参流程

[English](calibration_workflow.md)

## 文档目的

本文说明如何以可重复的方式对前跳控制栈做调参。

当前调参流程分成两个主要阶段：

1. 先把 `takeoff_speed_scale` 标定到目标距离附近
2. 再在不失控的前提下提升真正的空中前移能力

## 开始前检查

先确认工作区可以构建，并且仿真器确实会响应低层指令。

```bash
cd /home/hayan/go2_jump_ws
./scripts/docker_build_workspace.sh
./scripts/docker_verify_rt_lowcmd.sh
```

## 推荐调参顺序

### 第一步：跑一遍基线试验

```bash
./scripts/docker_run_single_jump_trial.sh 0.25
```

这一步的目标是拿到当前默认配置的参考报告。

### 第二步：先确定当前使用的参数档位

如果你想系统地比较控制器行为，建议在开始 sweep 之前先固定一个命名参数档位。

保守档：

```bash
GO2_JUMP_PROFILE=conservative_airborne \
./scripts/docker_run_single_jump_trial.sh 0.25
```

激进空中探索档：

```bash
GO2_JUMP_PROFILE=aggressive_airborne \
./scripts/docker_run_single_jump_trial.sh 0.25
```

### 第三步：标定起跳速度曲线

```bash
./scripts/sweep_takeoff_speed_scale.sh 0.20,0.25,0.30 1.00,1.03,1.06 1
```

当主要问题是“目标距离跟不上或超得太多”时，优先做这一步。

脚本会生成：

- `reports/calibration/` 下的原始 CSV
- `reports/calibration/` 下的汇总文件

### 第四步：提升空中前移能力

```bash
./scripts/sweep_airborne_push_pitch.sh 0.25 0.88,0.92,0.96 1.08,1.12,1.16 -8.0,-5.0,-2.0 -2.0,0.0 1
```

这一步应该在起跳速度曲线已经基本合理之后再做。

脚本会比较：

- 前后腿蹬地力矩比例
- 蹬地阶段俯仰目标
- 腾空阶段俯仰目标

并按空中前移能力和最终位移误差进行排序。

### 第五步：对更强的 profile 重新标定起跳速度

如果某个 profile 明显增强了空中前移，但最终位移超出目标，就不要立即改回姿态参数，
而是先保留这个 profile，再单独重标 `takeoff_speed_scale`。

```bash
./scripts/sweep_profile_takeoff_speed_scale.sh aggressive_airborne 0.25 0.94,0.97,1.00,1.03 1
```

当前项目里，这一步就是激进空中探索档的推荐后续动作。

## 常用命令速查

### 构建

```bash
./scripts/docker_build_workspace.sh
```

### 单次试验

```bash
./scripts/docker_run_single_jump_trial.sh 0.25
```

### 使用命名 profile

```bash
GO2_JUMP_PROFILE=aggressive_airborne \
./scripts/docker_run_single_jump_trial.sh 0.25
```

### 手动覆盖起跳速度

```bash
GO2_JUMP_USE_TAKEOFF_SPEED_SCALE_CURVE=false \
GO2_JUMP_TAKEOFF_SPEED_SCALE=1.09 \
./scripts/docker_run_single_jump_trial.sh 0.20
```

### 手动覆盖蹬地/俯仰参数

```bash
GO2_JUMP_PUSH_FRONT_TAU_SCALE=0.96 \
GO2_JUMP_PUSH_REAR_TAU_SCALE=1.12 \
GO2_JUMP_PUSH_PITCH_TARGET_DEG=-2.0 \
GO2_JUMP_FLIGHT_PITCH_TARGET_DEG=0.0 \
./scripts/docker_run_single_jump_trial.sh 0.25
```

### 手动覆盖 Landing Support

```bash
GO2_JUMP_LANDING_SUPPORT_BLEND=0.40 \
GO2_JUMP_LANDING_TOUCHDOWN_REFERENCE_BLEND=0.80 \
./scripts/docker_run_single_jump_trial.sh 0.25
```

## 如何解读试验报告

### 位移类指标

- `final_forward_displacement_m`
  恢复完成后的最终前向位移。
- `landing_forward_displacement_m`
  落地检测时的前向位移。
- `airborne_forward_progress_m`
  从起跳到落地检测之间的空中前向位移。
- `post_landing_forward_gain_m`
  落地之后额外增加的前向位移。
- `support_hold_forward_gain_m`
  控制器仍在 landing support hold 阶段时累积出来的前向位移。
- `release_to_complete_forward_gain_m`
  控制器从 support 开始释放到 recovery target 之后又增加的前向位移。

### 比例类指标

- `airborne_completion_ratio`
  `airborne_forward_progress_m / target_distance_m`
- `post_landing_completion_ratio`
  `post_landing_forward_gain_m / target_distance_m`

当目标距离相同时，这两个比值很适合比较不同试验。

### 实际判断经验

- 如果 `final_forward_displacement_m` 很高，但 `airborne_forward_progress_m`
  很低，通常说明真正的跳跃并不强，主要靠落地后恢复动作补位
- 如果 `airborne_forward_progress_m` 上升，同时 `max_abs_pitch_deg` 保持稳定
  或下降，通常说明这组参数值得继续追
- 如果空中前移增幅很大，但最终位移也明显超标，通常更适合把它当作
  “探索档”，而不是直接改成默认值
- 如果 `push_extension_after_plan_s` 很大，通常说明名义上的 push 时长还不够，
  控制器必须在原计划之外继续推一段时间才能真正起跳
- 如果 `flight_extension_after_plan_s` 很大，通常说明真实 flight 明显长于
  抛体近似估计，落地相位不应该再按旧的固定时长理解

## 当前参考结果

### 起跳速度曲线

2026 年 3 月 24 日的 focused sweep 给出了当前可用曲线：

- `0.20 m -> 1.09`
- `0.25 m -> 1.06`
- `0.30 m -> 1.06`

对应汇总文件位于本地 `reports/calibration/`：

- `speed_scale_sweep_20260324_214558_summary.txt`
- `speed_scale_sweep_20260324_215112_summary.txt`

### 2026 年 3 月 25 日的空中前移 sweep

当前 focused airborne sweep 汇总文件为：

- `reports/calibration/airborne_sweep_20260325_005214_summary.txt`

在 `0.25 m` 目标距离下，有两组参数值得作为参考：

- 保守改进档
  `push_front_tau_scale=0.96`，`push_rear_tau_scale=1.12`，
  `push_pitch_target_deg=-5.0`，`flight_pitch_target_deg=-2.0`
  结果：
  `avg_final_m ~= 0.2512`，`avg_airborne_m ~= 0.0462`
- 激进空中探索档
  `push_front_tau_scale=0.96`，`push_rear_tau_scale=1.12`，
  `push_pitch_target_deg=-2.0`，`flight_pitch_target_deg=0.0`
  结果：
  `avg_final_m ~= 0.3060`，`avg_airborne_m ~= 0.0655`

当前默认值采用保守档，因为它在提升空中前移的同时，最终位移仍然接近目标值。

### 2026 年 3 月 25 日的首次激进档重标验证

第一次对激进空中探索档做单点重标时，使用的是：

- `GO2_JUMP_PROFILE=aggressive_airborne`
- `GO2_JUMP_USE_TAKEOFF_SPEED_SCALE_CURVE=false`
- `GO2_JUMP_TAKEOFF_SPEED_SCALE=1.00`

单次验证结果为：

- `final_forward_displacement_m ~= 0.2593`
- `airborne_forward_progress_m ~= 0.0527`

这还不足以直接升级成默认档，但已经证明：保留激进姿态整形的同时，把起跳速度
往下调，确实能把最终位移重新拉回目标附近。

### 2026 年 3 月 25 日控制器重构后的重标结果

在事件驱动相位和前向几何偏置加入之后，新的 focused re-fit 汇总文件为：

- `reports/calibration/speed_scale_sweep_20260325_163702_summary.txt`

这轮 sweep 的工程结论是：

- 稳定主线已经能把 `0.25 m` 目标试验中的空中前移拉到大约
  `0.08-0.10 m`
- 但只靠继续往下调 `takeoff_speed_scale`，还不足以把最终位移压回目标，
  因为 `post_landing_forward_gain_m` 仍然占主导
- 一个实验性的 touchdown-hold 落地分支能明显压低落地后的前向补位，
  但当前还不够稳定，不能直接升级为默认落地策略

### 2026 年 3 月 25 日 Landing-Support 迭代

在继续只调起跳速度之前，控制器又做了一轮 landing 链路迭代，主要加入了：

- landing 和 recovery 之间连续的 support-hold 姿态
- `support_hold_forward_gain_m` 与 `release_to_complete_forward_gain_m`
- 可连续调节的 touchdown reference blend

当前默认参考方向是：

- `takeoff_angle_deg = 35.0`
- `landing_support_blend = 0.40`
- `landing_touchdown_reference_blend = 0.80`

2026 年 3 月 25 日最新几次默认 `0.25 m` 试验得到的大致结果是：

- `final_forward_displacement_m ~= 0.33-0.35`
- `airborne_forward_progress_m ~= 0.11`
- `post_landing_forward_gain_m ~= 0.23`
- `support_hold_forward_gain_m ~= 0.17`
- `release_to_complete_forward_gain_m ~= 0.06`
- `final_pitch_deg ~= -28`

这组默认值比旧 landing 路线更接近真正的前跳，因为它明显压低了“落地后补出来的
假增益”。但它仍然没有达到“绝大部分位移都靠腾空完成”的最终目标。

还有一组更激进的 landing-support 探针值得保留：

- `GO2_JUMP_LANDING_HOLD_USE_TOUCHDOWN_POSE=true`
- `GO2_JUMP_LANDING_SUPPORT_BLEND=0.40`

这一组大致得到：

- `final_forward_displacement_m ~= 0.3030`
- `airborne_forward_progress_m ~= 0.1054`
- `post_landing_forward_gain_m ~= 0.1974`
- `final_pitch_deg ~= -34.7`

它的意义在于证明“更强地贴近 touchdown 姿态”确实可以继续压低落地后补位，但当前
机身姿态仍然不够好，暂时还不适合作为默认值。

## 当前限制

- 现有 MuJoCo bridge 中 `foot_force_est` 仍为零
- 因此落地检测仍然是启发式判断，而不是基于真实接触力
- 报告中的落地位置应理解为事件估计值，而不是接触测量值

## 当前最推荐的下一组实验

下一步最值得做的，已经是沿着 touchdown-aware landing 路线继续搜索更干净的前跳
折中点。目标是：

- 保住较高的 `airborne_forward_progress_m`
- 明显压低 `post_landing_forward_gain_m`
- 同时保持 touchdown 后的机身稳定

最短命令路径就是：

```bash
GO2_JUMP_TAKEOFF_ANGLE_DEG=35.0 \
GO2_JUMP_LANDING_SUPPORT_BLEND=0.40 \
GO2_JUMP_LANDING_TOUCHDOWN_REFERENCE_BLEND=0.80 \
./scripts/docker_run_single_jump_trial.sh 0.25
```
