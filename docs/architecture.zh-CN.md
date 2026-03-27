# 架构说明

[English](architecture.md)

## 1. 当前运行主线

当前主线是：

`JumpTask -> JumpIntent -> WholeBodyMpc -> LowCmd`

这是整个仓库的骨架。

- `JumpTask`：任务层请求
- `JumpIntent`：高层显式跳跃意图
- `WholeBodyMpc`：低层执行层
- `LowCmd`：最终执行到电机的命令路径

## 2. 各包职责

### `go2_jump_msgs`

定义控制栈使用的 ROS 2 接口。

关键消息包括：

- `JumpTask.msg`
- `JumpIntent.msg`
- `JumpControllerState.msg`

### `go2_jump_core`

放置与 solver backend 解耦的任务层工具。

职责包括：

- 从目标距离构建 `JumpTaskSpec`
- 对任务规格做归一化
- 提供相位参考和采样工具
- 为 planner 和 controller 提供共用参考逻辑

### `go2_jump_planner`

高层规划包。

职责包括：

- 订阅 `/go2_jump/task`
- 把任务转换成显式 `JumpIntent`
- 发布 `/go2_jump/intent`

当前 backend 还是启发式版本。这个包的长期定位，是承载未来 RL planner，同时不改变下游控制接口。

### `go2_jump_mpc`

低层执行包。

职责包括：

- 订阅 `JumpTask`
- 订阅 `JumpIntent`
- 从 `LowState` 和 `SportModeState` 读取机器人状态
- 做接触估计
- 管理跳跃相位切换
- 构造可执行参考
- 运行 MuJoCo-native backend
- 发布 `LowCmd`
- 发布 `/go2_jump/controller_state`

### `go2_jump_bringup`

launch 和参数管理包。

职责包括：

- 共享参数
- 统一启动控制栈
- 提供 planner 开关，便于做 A/B 对比

## 3. 运行时数据流

1. `go2_jump_core` 发布 `JumpTask`
2. `go2_jump_planner` 把任务转换成 `JumpIntent`
3. `go2_jump_mpc` 同时接收 task 和 intent
4. 控制器结合真实接触状态和真实机器人状态，对齐名义计划
5. MuJoCo-native MPC backend 做短时域动作搜索
6. 当前最优动作的第一拍被转成 `LowCmd`
7. MuJoCo 执行命令并发布下一拍观测

## 4. 设计原则

这个仓库当前遵循四条设计原则。

### 原则 1：planner 输出必须显式

planner 必须输出可解释的量，例如起跳速度、相位时长和落地准备参数。

### 原则 2：低层执行链路保持稳定

最终执行接口始终保持为 `LowCmd / LowState`。

这样高层 planner 可以迭代，而不需要每次重写通信层或执行层。

### 原则 3：接触处理放在低层

高层 planner 不应该假设起跳和触地时刻一定与名义计划严格一致。

真实接触处理必须在 low-level 完成，因为只有 low-level 持有当前真实执行状态。

### 原则 4：planner 和 solver 要能公平对比

同一套 low-level 控制器应当能够支持：

- 不使用高层 intent
- 使用启发式显式 intent
- 使用未来的 RL 显式 intent

这样后续做消融实验才有意义。

## 5. 当前的主要替换点

未来最主要的高层替换点是：

- 把 `go2_jump_planner` 里的 `heuristic_explicit` 替换成 RL planner

未来最主要的低层增强点是：

- 持续强化 `go2_jump_mpc` 里的 `mujoco_native_mpc`

目标是在保持接口边界稳定的前提下，同时迭代高层 planner 和低层执行器。
