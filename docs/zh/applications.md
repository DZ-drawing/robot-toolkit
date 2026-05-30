[English](../applications.md) | [中文](applications.md)

# 机器人研发与应用场景

robot-toolkit 为机器人研发和应用的各个阶段提供基础计算能力。

## 研发阶段应用

### 1. 机器人设计验证

在物理样机制造前，验证运动学设计的可行性：

```python
from robot_ik import RobotModel, DHParam
import numpy as np

# 输入设计参数
robot = RobotModel([
    DHParam(a=0.35, alpha=-np.pi/2, d=0.2,  theta=0),
    DHParam(a=0.6,  alpha=0,        d=0,    theta=0),
    DHParam(a=0.15, alpha=-np.pi/2, d=0,    theta=0),
    DHParam(a=0,    alpha=np.pi/2,  d=0.5,  theta=0),
    DHParam(a=0,    alpha=-np.pi/2, d=0,    theta=0),
    DHParam(a=0,    alpha=0,        d=0.08, theta=0),
])

# 检查工作空间：遍历关节空间采样
reachable_points = []
for _ in range(10000):
    q = np.random.uniform(-np.pi, np.pi, 6)
    q = np.clip(q, [l[0] for l in robot.joint_limits], [l[1] for l in robot.joint_limits])
    T = robot.forward_kinematics(q)
    reachable_points.append(T[:3, 3])
```

**解决的问题**：
- 工作空间是否覆盖目标区域？
- 连杆长度和关节配置是否合理？
- 是否存在过多的奇异位形区域？

### 2. 运动学与动力学仿真

不依赖物理样机即可进行运动仿真：

```python
from robot_ik import DynamicsSolver, six_dof_articulated_dyn

solver = DynamicsSolver(six_dof_articulated_dyn())

# 计算特定构型下的重力补偿力矩
q = np.array([0, 0.5, -0.3, 0, 0.2, 0])
tau_gravity = solver.gravity_torque(q)

# 计算惯性矩阵 (用于控制器设计)
H = solver.inertia_matrix(q)

# 计算给定运动状态下的驱动力矩需求
qd = np.array([0.5, 0.3, -0.2, 0.1, 0.4, -0.1])
qdd = np.array([1.0, -0.5, 0.3, -0.2, 0.1, 0.5])
tau = solver.inverse_dynamics(q, qd, qdd)
```

**解决的问题**：
- 各关节的最大力矩需求是多少？
- 不同构型下惯性矩阵的条件数如何变化？
- 重力补偿力矩的范围是多少？

### 3. 控制器设计与调试

提供控制器设计所需的数学模型：

```python
# 计算力矩控制 (Computed Torque Control)
# τ = M(q)·(q̈_desired + Kd·(q̇_desired - q̇) + Kp·(q_desired - q)) + C(q,q̇)·q̇ + G(q)

Kp = np.diag([100, 100, 100, 50, 50, 20])
Kd = np.diag([20, 20, 20, 10, 10, 5])

def computed_torque_control(q, qd, q_desired, qd_desired, qdd_desired):
    solver = DynamicsSolver(model)
    tau_bias = solver.inverse_dynamics(q, qd, np.zeros(6))
    H = solver.inertia_matrix(q)
    e = q_desired - q
    ed = qd_desired - qd
    v = qdd_desired + Kd @ ed + Kp @ e
    return H @ v + tau_bias
```

### 4. URDF 模型导入与验证

从已有的 URDF 模型快速开始工作：

```python
from robot_ik import quick_urdf, urdf_to_dynamics_model

# 一行创建动力学求解器
solver = quick_urdf("path/to/robot.urdf")

# 或分步操作，获取完整模型
model = urdf_to_dynamics_model("path/to/robot.urdf")
print(f"连杆数: {len(model.links)}")
print(f"重力向量: {model.gravity}")
for i, link in enumerate(model.links):
    print(f"连杆 {i}: 质量={link.mass:.2f} kg")
```

---

## 应用场景

### 工业机器人

- **轨迹规划**：计算点到点、点到多点的高效关节轨迹
- **碰撞检测预处理**：快速 IK 求解后检查可达性
- **离线编程**：在仿真环境中验证机器人程序

### 协作机器人

- **重力补偿**：实时计算各关节重力力矩，降低手动引导的阻力
- **阻抗控制**：基于动力学模型的笛卡尔空间柔顺控制
- **安全监控**：验证关节轨迹是否在工作空间的安全区域内

### 移动操作 (Mobile Manipulation)

- **工作空间分析**：给定底盘位置，计算机械臂的可达空间
- **运动规划**：分解为底盘定位 + 机械臂 IK 求解
- **力控制**：基于动力学的接触力调节

### 机器人教育

- **运动学可视化**：3D 显示机械臂构型，直观理解 DH 参数
- **算法验证**：对比数值解与解析解，理解算法原理
- **快速原型**：修改 DH 参数即时看到效果

### 机器人仿真

- **物理仿真**：正动力学提供关节加速度，配合数值积分进行时间步进仿真
- **传感器仿真**：基于 FK 计算仿真传感器 (力/力矩、编码器) 读数
- **环境交互**：基于末端位姿计算与环境的接触几何

---

## 典型工作流

### 场景：新机器人从设计到控制

```
1. 设计阶段
   DHParam → RobotModel → FK → 工作空间分析

2. 建模阶段
   URDF → urdf_to_dynamics_model → DynamicsSolver → 力矩需求分析

3. 规划阶段
   目标位姿 → ik_solve → 关节角 → 轨迹规划

4. 控制阶段
   关节轨迹 → inverse_dynamics → 力矩指令

5. 仿真验证
   力矩指令 → forward_dynamics → 加速度 → 数值积分 → 运动仿真
```

### 场景：URDF 模型快速验证

```
1. 导入: quick_urdf("robot.urdf")
2. 验证: gravity_torque, inertia_matrix
3. 可视化: plot_robot(robot, joint_angles)
4. 测试: IK roundtrip, 动力学 roundtrip
```
