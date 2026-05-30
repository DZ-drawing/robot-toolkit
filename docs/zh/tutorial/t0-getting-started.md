[English](../../tutorial/t0-getting-started.md) | **中文**

# 教程 0 — robot-ik 入门

安装 robot-ik、构建机器人模型、运行正运动学和逆运动学，以及从 URDF 加载机器人的分步指南。

---

## 1. 安装

```bash
pip install robot-ik
```

或从源码安装：

```bash
git clone https://github.com/your-org/robot-toolkit.git
cd robot-toolkit
pip install -e .
```

---

## 2. 创建机器人模型

`six_dof_articulated()` 返回一个预配置的 6 自由度工业机械臂，包含 DH 参数、关节限位和球腕 —— 可直接使用。

```python
import numpy as np
from robot_ik.ik_solver import six_dof_articulated

# 构建模型
robot = six_dof_articulated()

print(type(robot))   # <class 'robot_ik.ik_solver.RobotModel'>
print(robot.dh_params)       # 6 个 DHParam 对象的列表
print(robot.joint_limits)    # [(min, max), ...]，单位为弧度
```

---

## 3. 正运动学

给定六个关节角度（弧度），`forward_kinematics` 返回从基座到末端执行器的 4×4 齐次变换矩阵。

```python
# 关节角度：[底座, 肩部, 肘部, 腕1, 腕2, 腕3]
q = np.array([0.0, 0.5, -0.3, 0.0, 0.2, 0.0])

T = robot.forward_kinematics(q)
print("末端执行器位置：", T[:3, 3])
print("末端执行器旋转：\n", T[:3, :3])

# 获取运动链上每个连杆的变换
T_ee, all_transforms = robot.forward_kinematics(q, return_all=True)
```

---

## 4. 逆运动学

`ik_solve` 查找将末端执行器放置在目标位姿的关节角度。它使用阻尼最小二乘（Levenberg-Marquardt）求解器。

```python
# 定义目标位姿为 4x4 齐次矩阵
target = np.eye(4)
target[:3, 3] = [0.3, 0.4, 0.6]  # 期望的 xyz 位置

# 运行 IK
success, q_sol, iterations, errors = robot.ik_solve(
    target_pose=target,
    initial_guess=None,        # 默认全零
    max_iterations=200,
    position_tolerance=1e-4,    # 米
    orientation_tolerance=1e-3, # 弧度
    damping=0.1,
)

print("是否收敛：", success)
print("解（弧度）：", q_sol)
print("迭代次数：", iterations)

# 验证：对解运行正运动学
T_check = robot.forward_kinematics(q_sol)
print("正运动学位置误差：", np.linalg.norm(T_check[:3, 3] - target[:3, 3]))
```

---

## 5. 雅可比矩阵

几何雅可比矩阵将关节速度映射到末端执行器速度旋量：

```python
J = robot.compute_jacobian(q)
print("雅可比矩阵形状：", J.shape)  # (6, 6)
```

---

## 6. 从 URDF 加载机器人

`urdf_to_dynamics_model` 将 URDF 文件解析为 `RobotDynamicsModel`，包含 DH 参数和每个连杆的惯量数据。

```python
from robot_ik.urdf_parser import urdf_to_dynamics_model

dyn_model = urdf_to_dynamics_model("my_robot.urdf")
print("解析的连杆数：", len(dyn_model.links))
print("DH 参数 (a)：", dyn_model.dh_a)
```

### 示例 URDF

将以下内容保存为 `simple_arm.urdf`：

```xml
<?xml version="1.0"?>
<robot name="simple_6dof">
  <!-- 基座连杆 -->
  <link name="base_link">
    <inertial>
      <mass value="5.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  <!-- 肩部连杆 -->
  <link name="shoulder_link">
    <inertial>
      <mass value="3.0"/>
      <origin xyz="0 0 0.15"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.005"/>
    </inertial>
  </link>
  <!-- 肘部连杆 -->
  <link name="elbow_link">
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 0.1"/>
      <inertia ixx="0.003" ixy="0" ixz="0" iyy="0.003" iyz="0" izz="0.003"/>
    </inertial>
  </link>

  <!-- 关节 -->
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="shoulder_link"/>
    <origin xyz="0 0 0.3"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14"/>
  </joint>
  <joint name="joint2" type="revolute">
    <parent link="shoulder_link"/>
    <child link="elbow_link"/>
    <origin xyz="0 0.5 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57"/>
  </joint>
</robot>
```

```python
# 快速辅助函数：直接从 URDF 路径获取 DynamicsSolver
from robot_ik.urdf_parser import quick_urdf

solver = quick_urdf("simple_arm.urdf")
```

---

## 7. 下一步

- **教程 1** — 动力学：正/逆动力学、重力补偿。
- **教程 2** — 轨迹规划和运动曲线。
- **API 参考** — 每个类和函数的完整文档。
