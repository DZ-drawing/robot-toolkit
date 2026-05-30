[English](../../tutorial/t5-dynamics.md) | **中文**

# T5：机器人动力学 —— 逆动力学、重力补偿和惯量分析

**难度：** 中级 | **时间：** 25 分钟 | **模块：** `robot_ik.robot_dyn`

本教程使用 `robot_ik.robot_dyn` 模块介绍 6 自由度串联机械臂的刚体动力学。你将学习通过逆动力学计算关节力矩、补偿重力、分离科里奥利/离心力效应，以及分析关节空间惯量矩阵。

## 你将学到

1. 创建带有 DH 参数和连杆惯量的 `RobotDynamicsModel`
2. 使用 `DynamicsSolver.inverse_dynamics()` 进行完整力矩计算
3. 计算重力补偿力矩
4. 分离科里奥利和离心力矩
5. 计算和分析关节空间惯量矩阵 `H(q)`
6. 运行正动力学仿真

## 前置要求

```bash
pip install robot-ik numpy
```

```python
import numpy as np
```

---

## 步骤 1：加载动力学模型

`six_dof_articulated_dyn()` 工厂函数返回一个预配置的 6 自由度机器人，
每个连杆具有真实的质量、质心和惯量张量。

```python
from robot_ik import six_dof_articulated_dyn, DynamicsSolver

model = six_dof_articulated_dyn()

# 检查模型
print(f"连杆数量：{len(model.links)}")
print(f"重力向量：  {model.gravity}")
print(f"阻尼：         {model.joint_damping}")
print()

for i, link in enumerate(model.links):
    print(f"连杆 {i}：质量={link.mass:.1f} kg，"
          f"质心={link.com.round(3)}，"
          f"I_diag={np.diag(link.inertia).round(4)}")
```

每个 `LinkInertia` 有三个字段：

| 字段 | 类型 | 描述 |
|------|------|------|
| `mass` | `float` | 连杆质量（kg） |
| `com` | `np.ndarray (3,)` | 连杆坐标系中的质心 |
| `inertia` | `np.ndarray (3,3)` | 关于质心的惯量张量（kg*m^2） |

## 步骤 2：逆动力学

给定关节位置 `q`、速度 `qd` 和加速度 `qdd`，递归牛顿-欧拉算法计算产生该运动所需的关节力矩 `tau`。

```python
solver = DynamicsSolver(model)

# 关节状态：全部为零
q = np.zeros(6)
qd = np.zeros(6)
qdd = np.zeros(6)

# 计算力矩（静止时，这等于重力补偿）
tau = solver.inverse_dynamics(q, qd, qdd)
print(f"归零位重力力矩：  {tau.round(3)} Nm")

# 现在关节 2 有加速度
qdd = np.array([0.0, 2.0, 0.0, 0.0, 0.0, 0.0])
tau_accel = solver.inverse_dynamics(q, qd, qdd)
print(f"关节2 qdd=2 时的力矩：        {tau_accel.round(3)} Nm")
print(f"关节2 的增量力矩：{tau_accel[1] - tau[1]:.3f} Nm")
```

### 外力/力矩

你还可以传入可选的 `external_wrench` 来建模施加在末端执行器上的力/力矩：

```python
# 末端执行器处 10N 向下的力
wrench = np.array([0, 0, -10, 0, 0, 0])  # [fx, fy, fz, nx, ny, nz]
tau_wrench = solver.inverse_dynamics(q, qd, qdd, external_wrench=wrench)
print(f"带 10N 负载的力矩：{tau_wrench.round(3)} Nm")
```

## 步骤 3：重力补偿

重力补偿计算机器人在重力作用下保持静止所需的力矩。这等价于在 `qd=0, qdd=0` 的情况下调用 `inverse_dynamics`。

```python
# 扫描关节 1 范围内的重力力矩
q_scan = np.zeros(6)
print("关节 1 角度 | 重力力矩 (Nm)")
print("-" * 50)
for angle in np.linspace(-np.pi, np.pi, 7):
    q_scan[0] = angle
    g_tau = solver.gravity_torque(q_scan)
    print(f"  {np.degrees(angle):+7.1f} 度  | {g_tau.round(3)}")
```

当手臂水平伸展时重力力矩最大，当垂直下垂时最小 —— 这就是你应该在输出中看到的结果。

## 步骤 4：科里奥利和离心力矩

`coriolis_torque(q, qd)` 通过从 `qdd=0` 的完整逆动力学中减去重力来分离速度相关力矩：

```
tau_coriolis = inverse_dynamics(q, qd, 0) - gravity_torque(q)
```

```python
# 关节 1 快速旋转
q = np.array([0.0, np.pi/4, 0, 0, 0, 0])
qd = np.array([3.0, 0, 0, 0, 0, 0])  # 关节 1 以 3 rad/s 旋转

c_tau = solver.coriolis_torque(q, qd)
g_tau = solver.gravity_torque(q)

print(f"科里奥利力矩（关节1 以 3 rad/s 旋转）：{c_tau.round(4)} Nm")
print(f"重力力矩（相同配置）：              {g_tau.round(4)} Nm")
print()
print("注意：来自关节 1 的离心耦合会加载关节 2-6，"
      "即使只有关节 1 在运动。")
```

## 步骤 5：关节空间惯量矩阵

`inertia_matrix(q)` 计算 `H(q)` —— 6x6 对称正定质量矩阵。对角线元素表示每个关节的有效惯量；非对角线元素表示关节之间的耦合。

```python
# 归零位的惯量
H_home = solver.inertia_matrix(np.zeros(6))

# 手臂伸展时的惯量
q_extended = np.array([0.0, np.pi/2, 0, 0, 0, 0])
H_extended = solver.inertia_matrix(q_extended)

print("H(q) 归零位：")
print(np.array2string(H_home, precision=4, suppress_small=True))
print()
print("H(q) 肩部抬起 90 度：")
print(np.array2string(H_extended, precision=4, suppress_small=True))

# 对角线分析
print()
print("有效关节惯量：")
for name, H in [("归零位", H_home), ("伸展", H_extended)]:
    diag = np.diag(H)
    print(f"  {name:8s}：{diag.round(4)}")
```

当手臂伸展时，关节 2 的惯量显著增大，因为必须加速整个手臂的质量。这就是反射惯量效应 —— 对于电机选型和控制器设计非常重要。

### 从 H(q) 计算可操作性

`H(q)` 的条件数表示惯量矩阵的条件好坏。条件差意味着某些方向比其他方向更难加速：

```python
cond_home = np.linalg.cond(H_home)
cond_extended = np.linalg.cond(H_extended)
print(f"归零位条件数：      {cond_home:.1f}")
print(f"伸展条件数：     {cond_extended:.1f}")
```

## 步骤 6：正动力学

给定关节力矩 `tau`，正动力学通过求解以下方程计算关节加速度：

```
H(q) * qdd = tau - C(q, qd) - G(q)
```

这使用复合刚体算法计算 `H(q)`，牛顿-欧拉算法计算偏置力。

```python
q = np.array([0.0, np.pi/4, 0, 0, 0, 0])
qd = np.zeros(6)
tau = np.array([5.0, 0, 0, 0, 0, 0])  # 在关节 1 施加 5 Nm

qdd = solver.forward_dynamics(q, qd, tau)
print(f"施加力矩：  {tau}")
print(f"结果加速度：  {qdd.round(4)} rad/s^2")
print()

# 简单欧拉积分 1 秒
q_sim = q.copy()
qd_sim = qd.copy()
dt = 0.01

for _ in range(int(1.0 / dt)):
    qdd = solver.forward_dynamics(q_sim, qd_sim, tau)
    qd_sim += qdd * dt
    q_sim += qd_sim * dt

print(f"1 秒仿真后：")
print(f"  q  = {q_sim.round(3)} rad")
print(f"  qd = {qd_sim.round(3)} rad/s")
```

## 关键要点

| 方法 | 用途 | 签名 |
|------|------|------|
| `inverse_dynamics` | 完整牛顿-欧拉力矩计算 | `(q, qd, qdd, external_wrench=None)` |
| `gravity_torque` | 静态重力补偿 | `(q)` |
| `coriolis_torque` | 速度相关力矩 | `(q, qd)` |
| `inertia_matrix` | 关节空间质量矩阵 H(q) | `(q)` |
| `forward_dynamics` | 从力矩计算关节加速度 | `(q, qd, tau)` |

## 下一步

- **T6：** 使用 Meshcat 在 3D 中可视化机器人
- **T7：** 规划笛卡尔直线路径并进行碰撞检测
