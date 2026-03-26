# Go2 Jump Workspace 中文文档

[English](README.md)

## 项目定位

这个仓库是一个面向教学和实验迭代的 Go2 前跳项目。项目目标不是直接做一套可上机器人实机的完整最优控制器，而是先搭建一条清晰、可运行、可验证、可调参的最小闭环链路：

- 使用 `unitree_mujoco` 作为主仿真平台
- 使用 `unitree_ros2` 打通 `LowCmd` 和 `LowState`
- 在 sport mode 之下做低层关节控制

当前版本聚焦于平地、短距离、左右对称前跳。

## 你能从这个项目学到什么

- 如何组织一个基于 ROS 2 Humble 的 Go2 仿真项目
- 如何在 MuJoCo 中验证低层 `LowCmd` 是否真的生效
- 如何把“目标距离”映射到一个参数化的跳跃动作
- 如何用试验报告和扫参脚本逐步优化跳跃效果
- 如何把一个实验项目整理成可复现、可维护的工程仓库

## 建议阅读顺序

如果你是第一次接触这个项目，建议按下面顺序阅读和操作：

1. 先读本文件，完成第一次构建和第一次单次试验。
2. 再读 [控制链路说明](docs/control_stack.zh-CN.md)，理解运行时数据怎么流动。
3. 再读 [算法说明](algorithm.md)，理解 planner、controller、WBC 各自在算什么。
4. 接着读 [调参流程](docs/calibration_workflow.zh-CN.md)，理解怎么读报告、怎么做 sweep。
5. 如果准备改代码或改依赖，再读 [开发流程](docs/development_workflow.zh-CN.md)。

## 项目范围

### 当前已经包含

- Docker 化的 ROS 2 Humble 开发环境
- Go2 的 Unitree MuJoCo 仿真
- `LowCmd` / `LowState` 低层通信链路
- 参数化的前跳控制器
- 单次试验、日志归档、试验报告
- 起跳速度和空中前移能力的基础调参脚本

### 当前不包含

- sport mode 的 `frontJump`
- 直接用于实机的控制安全机制
- 接触丰富的最优控制
- 完整的状态估计与感知系统

## 已验证环境基线

当前仓库按照以下基线完成过验证：

- Ubuntu 22.04
- Docker 中运行 ROS 2 Humble
- `unitree_ros2` 与 `unitree_mujoco` 作为 submodule 管理
- 低层话题链路：
  `rt/lowcmd`、`rt/lowstate`、`/lowcmd`、`/lowstate`、`/sportmodestate`

项目采用 Docker 的目的很明确：保证 ROS 2、MuJoCo 和 Unitree SDK 的运行环境可复现。

## 目录说明

- `src/unitree_ros2`
  官方 Unitree ROS 2 仓库，作为 submodule 管理。
- `src/unitree_mujoco`
  官方 Unitree MuJoCo 仓库，作为 submodule 管理。
- `src/go2_jump_planner`
  负责跳跃参数生成和目标发布。
- `src/go2_jump_controller`
  负责低层控制、相位机和试验报告。
- `src/go2_jump_bringup`
  负责 launch 文件和公共参数。
- `scripts/`
  负责构建、启动、验证、扫参。
- `tools/`
  负责直接 DDS 验证工具。
- `patches/`
  保存对上游 submodule 的本地补丁。
- `docs/`
  保存架构、调参、开发相关文档。
- `algorithm.md`
  说明当前前跳算法的 planner、controller 和 WBC 分工。

## 快速开始

### 1. 克隆仓库

```bash
git clone --recurse-submodules https://github.com/matrix050628/go2_jump.git /home/hayan/go2_jump_ws
cd /home/hayan/go2_jump_ws
```

### 2. 初始化工作区

```bash
./scripts/bootstrap_workspace_repo.sh
./scripts/bootstrap_third_party.sh
```

其中：

- `bootstrap_workspace_repo.sh` 会初始化 submodule，并自动应用
  `unitree_mujoco` 的兼容补丁
- `bootstrap_third_party.sh` 会准备第三方依赖缓存

### 3. 构建 Docker 镜像

```bash
./scripts/docker_build_image.sh
```

### 4. 构建工作区

```bash
./scripts/docker_build_workspace.sh
```

这一步会完成以下主要构建任务：

- `unitree_ros2` 相关依赖包
- `go2_jump_planner`
- `go2_jump_controller`
- `go2_jump_bringup`
- `tools/verify_rt_lowcmd`
- `unitree_mujoco`

### 5. 启动 MuJoCo 仿真

```bash
./scripts/docker_run_go2_mujoco.sh
```

如果宿主机没有图形桌面，会自动启动 `Xvfb`，保证 MuJoCo 的渲染循环和 DDS bridge 能稳定运行。

### 6. 启动跳跃控制栈

另开一个终端：

```bash
cd /home/hayan/go2_jump_ws
./scripts/docker_launch_jump_stack.sh 0.25
```

### 7. 直接运行一次完整试验

如果你只想快速确认项目能不能跑通，推荐直接用单次试验脚本：

```bash
cd /home/hayan/go2_jump_ws
./scripts/docker_run_single_jump_trial.sh 0.25
```

这个脚本会自动完成：

- 启动一个新的仿真实例
- 等待 `/lowstate` 稳定
- 启动 planner 和 controller
- 等待新的试验报告生成
- 归档日志、报告和试验上下文
- 关闭运行时容器

## 两种标准验证方式

### 方式一：直接验证低层 DDS

```bash
./scripts/docker_verify_rt_lowcmd.sh
```

这个脚本适合回答一个非常具体的问题：

“现在的 `LowCmd` 到底有没有真正驱动仿真中的关节？”

它会绕开 ROS 2 控制器，直接对 `rt/lowcmd` / `rt/lowstate` 做验证。

### 方式二：验证完整前跳闭环

```bash
./scripts/docker_run_single_jump_trial.sh 0.25
```

这个脚本适合验证完整链路是否打通：

- MuJoCo 仿真
- `/lowstate`
- planner
- controller
- `/lowcmd`
- 报告生成

## 如何读试验报告

如果你只看一个指标，很容易误判“跳得更好了”。建议至少同时看下面四项：

- `final_forward_displacement_m`
  最终稳定后的前向位移。
- `landing_forward_displacement_m`
  落地检测时的前向位移。
- `airborne_forward_progress_m`
  从起跳到落地之间的空中前向位移。
- `post_landing_forward_gain_m`
  落地之后又向前补出来的位移。

对于这个项目，`airborne_forward_progress_m` 更能反映“是不是真的跳出去了”，而 `final_forward_displacement_m` 更像“最后停在了哪里”。

最近一轮控制器重构后，还有两个时间字段很值得关注：

- `push_extension_after_plan_s`
  名义上的 push 阶段结束后，到真正检测到起跳之间又额外持续了多久。
- `flight_extension_after_plan_s`
  相比抛体近似给出的 flight 时间，真正检测到落地又多飞了多久。

## 调参入口

### 0. 切换命名参数档位

当前仓库支持用 `GO2_JUMP_PROFILE` 选择一组命名参数档位。

例如，显式使用当前默认风格的保守档：

```bash
GO2_JUMP_PROFILE=conservative_airborne \
./scripts/docker_run_single_jump_trial.sh 0.25
```

切到更激进的空中探索档：

```bash
GO2_JUMP_PROFILE=aggressive_airborne \
./scripts/docker_run_single_jump_trial.sh 0.25
```

当前内置档位包括：

- `config_default`
- `conservative_airborne`
- `aggressive_airborne`

### 1. 标定起跳速度

```bash
./scripts/sweep_takeoff_speed_scale.sh 0.20,0.25,0.30 1.00,1.03,1.06 1
```

这个 sweep 解决的是“给定目标距离，起跳速度应该放大多少”。

如果你已经切到某个命名档位，想在保留该档位姿态/力矩特征的同时重新标定
`takeoff_speed_scale`，可以直接使用：

```bash
./scripts/sweep_profile_takeoff_speed_scale.sh aggressive_airborne 0.25 0.94,0.97,1.00,1.03 1
```

这个脚本特别适合处理“空中前移明显增强，但最终位移超了”的情况。

### 2. 优化空中前移能力

```bash
./scripts/sweep_airborne_push_pitch.sh 0.25 0.88,0.92,0.96 1.08,1.12,1.16 -8.0,-5.0,-2.0 -2.0,0.0 1
```

这个 sweep 解决的是“在目标距离固定时，如何让真正的空中前移更明显”。它会比较：

- `airborne_forward_progress_m`
- `airborne_completion_ratio`
- 最终位移误差

### 3. 做多距离泛化验证

```bash
GO2_JUMP_GENERALIZATION_MAX_ATTEMPTS=3 \
./scripts/evaluate_airborne_generalization.sh 0.18,0.20,0.219,0.25,0.288,0.30,0.32 2 20260326 1
```

这个脚本会把固定目标距离和随机目标距离放到同一轮验证里跑完。

参数含义是：

- 第一个参数：固定目标距离列表
- 第二个参数：额外随机目标数量
- 第三个参数：随机种子
- 第四个参数：每个目标跑几次

脚本会输出：

- 各目标距离的最终位移误差
- 真正的腾空前移占比
- 落地后补位占比
- `support hold` 阶段增益和 `release_to_complete` 回摆
- 落地和最终俯仰角

如果仿真在长批次里偶发没写出报告，脚本会按 `GO2_JUMP_GENERALIZATION_MAX_ATTEMPTS`
自动重试。要做纯固定距离回归，把第二个参数设成 `0` 即可。

如果你要判断“控制器是不是只会跳 `0.25 m`”，优先看这个脚本，而不是只看单条试验报告。

## 当前参考参数

### 起跳速度曲线

当前距离到 `takeoff_speed_scale` 的标定结果是：

- `0.20 m -> 1.06`
- `0.25 m -> 1.03`
- `0.30 m -> 1.12`

### YAML 基线参数

YAML 里的默认参数仍然是当前仓库最方便的低层回归基线。它的姿态更保守，适合先确认
`LowCmd`、`LowState`、相位机和报告系统都正常，再切到更强调腾空前移的参考档。

这组基线参数的关键项是：

- `takeoff_angle_deg = 35.0`
- `push_front_tau_scale = 0.96`
- `push_rear_tau_scale = 1.12`
- `push_pitch_target_deg = -5.0`
- `flight_pitch_target_deg = -2.0`
- `landing_support_blend = 0.40`
- `landing_touchdown_reference_blend = 0.80`

它不是当前“主要靠腾空完成前移”的主参考，而是一条更稳、更适合做快速回归的工程基线。

### 当前空中优先参考

当前仓库里真正用于“前跳质量”验证的主入口是下面三份脚本：

- `scripts/airborne_priority_params.sh`
  统一管理空中优先参考档的默认环境变量。
- `scripts/run_airborne_priority_trial.sh`
  单目标基准入口。默认会把 `0.25 m` 当作手工探针，使用
  `takeoff_speed_scale=1.03`；其它目标距离默认改走 YAML 里的起跳速度曲线。
- `scripts/evaluate_airborne_generalization.sh`
  多目标验证入口，用同一套参数做固定距离和随机距离测试。

推荐先用下面这条命令看单点基准：

```bash
./scripts/run_airborne_priority_trial.sh 0.25
```

当前最有代表性的两份结果文件是：

- `reports/jump_metrics/trial_20260326_174521.txt`
  `0.25 m` 单点手工基准报告。
- `reports/calibration/airborne_generalization_20260326_174115_summary.txt`
  固定目标集合 `0.18,0.20,0.219,0.25,0.288,0.30,0.32` 的代表性汇总。

当前 `0.25 m` 手工基准大致是：

- `final_forward_displacement_m ~= 0.235`
- `airborne_forward_progress_m ~= 0.168`
- `post_landing_forward_gain_m ~= 0.052`
- `landing_pitch_deg ~= -46.5`
- `final_pitch_deg ~= -41.4`

固定目标汇总里的总体均值大致是：

- `avg_abs_error_m ~= 0.0170`
- `avg_airborne_ratio ~= 0.6777`
- `avg_post_ratio ~= 0.2049`
- `avg_landing_pitch_deg ~= -46.3`

和上一轮相比，这一版真正的变化不只是“某几个数字更好看了”，而是控制逻辑本身做了
距离自适应：

- `support` 阶段的前向 capture 不再是几乎固定的一段前伸，而是对短距离额外加了上限
- `landing_support_blend` 会随目标距离变化，短跳时自动更保守
- `support pitch capture` 的增益也会按目标距离缩放，短跳时更弱
- 起跳速度曲线是在这轮落地链路改动之后重新标定的
- 多距离验证脚本补上了自动重试逻辑，更适合做回归

从结果上看，短距离目标现在干净了不少，问题已经从“短距离靠落地后补位完成太多”
转成了“中长距离略发短”。

### 目前最值得继续盯的点

- `0.18-0.219 m`
  这一段已经比较接近“主要靠腾空完成”的目标。
- `0.25-0.32 m`
  当前仍然普遍偏短，代表性汇总里大约短 `1.5-4.5 cm`。
- `release_to_complete_forward_gain_m`
  长距离目标在 `support hold` 之后仍有一点回摆。

## 当前局限

- 当前空中优先参考落地时仍然比较前俯，俯仰角大致在 `-43` 到 `-49 deg`
- `0.25-0.32 m` 这段目标距离仍然容易发短
- 长距离目标在 `support hold` 之后还存在少量回摆
- 长批次验证偶尔还会触发运行时重试
- 当前版本是仿真优先版本，不应直接视为实机控制器

## 文档索引

- [English README](README.md)
- [Control Stack](docs/control_stack.md)
- [控制链路说明](docs/control_stack.zh-CN.md)
- [Calibration Workflow](docs/calibration_workflow.md)
- [调参流程](docs/calibration_workflow.zh-CN.md)
- [Development Workflow](docs/development_workflow.md)
- [开发流程](docs/development_workflow.zh-CN.md)
