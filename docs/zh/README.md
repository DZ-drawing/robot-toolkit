[English](../README.md) | [中文](README.md)

# robot-toolkit 文档索引

## 项目文档

| 文档 | 内容 |
|------|------|
| [README.md](../../README.md) | 快速入门、安装、使用示例 |
| [ROADMAP.md](../../ROADMAP.md) | 开发路线图 (已完成 + 计划) |

## 设计与原理

| 文档 | 内容 |
|------|------|
| [设计哲学](../design-philosophy.md) | 核心设计原则、数据约定、错误处理策略 |
| [架构与部件说明](architecture.md) | 各模块的详细 API 说明、数据结构、算法实现 |
| [科学原理](../scientific-principles.md) | DH 参数、运动学、动力学、RNEA 等数学基础 |

## 开发

| 文档 | 内容 |
|------|------|
| [构建与测试指南](../build-and-test.md) | 安装方式、测试套件、测试方法论、构建打包 |
| [应用场景](applications.md) | 研发阶段应用、工业/协作机器人场景、典型工作流 |
| [参与开源开发](../contributing.md) | 代码风格、开发流程、PR 指南、性能优化 |

## API 快速参考

```python
# 运动学
from robot_ik import RobotModel, DHParam, dh_transform
from robot_ik import six_dof_articulated, spherical_wrist_6dof

robot = six_dof_articulated()
T = robot.forward_kinematics(q)               # 4x4 齐次变换
J = robot.compute_jacobian(q)                  # 6x6 雅可比矩阵
success, q_sol, iters, errs = robot.ik_solve(T)  # IK 求解

# 动力学
from robot_ik import DynamicsSolver, RobotDynamicsModel, LinkInertia
from robot_ik import six_dof_articulated_dyn

solver = DynamicsSolver(six_dof_articulated_dyn())
tau = solver.inverse_dynamics(q, qd, qdd)      # 逆动力学 (RNEA)
tau_g = solver.gravity_torque(q)                # 重力力矩
tau_c = solver.coriolis_torque(q, qd)           # 科里奥利力矩
H = solver.inertia_matrix(q)                    # 惯性矩阵
qdd = solver.forward_dynamics(q, qd, tau)       # 正动力学 (CRBA)

# URDF
from robot_ik import urdf_to_dynamics_model, quick_urdf
solver = quick_urdf("robot.urdf")

# 可视化
from robot_ik.visualize import plot_robot, plot_convergence
```
