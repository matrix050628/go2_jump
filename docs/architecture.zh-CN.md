# 架构说明

## 目标

现在这个仓库只围绕一条主线组织：

`按距离条件化的 jump task -> whole-body MPC -> LowCmd`

这样拆分的目的，是让后续方法切换、消融实验和论文对比都更清晰。

## 包职责

### `go2_jump_msgs`

保存任务层接口。

当前第一版接口是 `JumpTask.msg`，用于描述一个按目标距离条件化的跳跃任务，以及
由任务推导出的起跳速度和阶段时序目标。

### `go2_jump_core`

负责任务生成，不直接负责控制。

它的职责包括：

- 把目标距离映射成起跳速度目标
- 估计名义腾空时间
- 定义下蹲、蹬地、腾空、落地、稳定五个阶段的高层目标
- 发布当前激活的 `JumpTask`

这个包保持轻量很重要，因为后续不管是 whole-body MPC、centroidal/QP，还是学习
残差层，都应该可以共用这一层任务接口。

### `go2_jump_mpc`

这是新的控制主线包。

职责包括：

- 接收 `JumpTask`
- 构建 receding horizon preview
- 从低层观测估计接触状态
- 当实际接触和名义阶段不一致时，做接触感知阶段切换
- 管理 MPC 参数和后端切换
- 发布 `LowCmd`
- 发布 `/go2_jump/controller_state` 调试话题

这个包的设计原则是：先把外层接口定住，再逐步把内部后端从 preview 版本推进到
真正的 MuJoCo-native solver。

### `go2_jump_bringup`

只负责新主线的 launch 和共享参数。

控制逻辑不再堆在 launch 文件里，而是收回 `go2_jump_mpc` 包本身。

## 运行时链路

1. `go2_jump_core` 根据目标距离生成 `JumpTask`。
2. `go2_jump_mpc` 接收任务和机器人当前状态。
3. MPC 层先估计接触状态并修正阶段，再构建预测地平线并求出当前控制输出。
4. 控制器发布 `LowCmd`。
5. MuJoCo 执行这一拍控制，再把状态反馈给下一轮优化。

## 设计原则

任务层尽量稳定，求解器层允许快速演进。

这样做的好处是：

- 可以在同一任务接口下切换 preview / centroidal-QP / MuJoCo-native backend
- 方便做公平对比实验
- 随着控制器升级，实验仍然可复现
