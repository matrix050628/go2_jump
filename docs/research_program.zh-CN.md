# 研究路线

## 研究假设

如果把控制主线切到 `MuJoCo-native whole-body MPC + 接触感知落地控制`，那么在
Go2 上有机会做出比模板式关节控制更干净、更主要依靠腾空完成前移的前跳。

## 第一阶段

先把仓库重构成新的研究架构。

这一阶段的交付物包括：

- 任务层 jump specification
- MPC 主线包边界
- 可复现的上游依赖搭建方式
- 一条干净的新主线 launch 路径
- 已验证的 `/lowstate -> JumpTask -> controller_state -> /lowcmd` 链路

## 第二阶段

做出第一版闭环 MPC 后端。

最低要求：

- 能接收 `JumpTask`
- 能构建 receding horizon reference
- 能通过现有 ROS 2 外部接口输出低层控制量
- 能输出 solver bring-up 需要的调试信息

## 第三阶段

从 preview/reference 模式升级到真正的 MuJoCo-native solver backend。

近期重点包括：

- 用 MuJoCo 动力学做 horizon rollout
- 把起跳速度、机身俯仰、落地稳定和终端收敛写进 cost
- 明确的力矩和状态正则项
- touchdown-aware 的地平线自适应

## 第四阶段

进入论文级实验阶段。

至少需要的对比包括：

- whole-body MPC 主线
- centroidal / QP 退路基线
- 现实可复现的成熟公开方法

至少需要的指标包括：

- 成功率
- 腾空阶段前向位移
- 落地后补位距离
- 触地瞬间俯仰角和俯仰角速度
- 峰值力矩和能耗代理指标

## 论文方向

这次仓库重构的目的，就是让最后的工作能够围绕下面这个故事展开：

- 按目标距离条件化的前跳
- 主要靠腾空完成前移
- 落地干净并且可反应式调整
- Go2 上的低层控制实现
- 基于 MuJoCo 的 whole-body 优化控制
