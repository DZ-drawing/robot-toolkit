[English](../architecture.md) | [中文](architecture.md)

# robot-toolkit 架构与部件说明

## 项目结构

```
robot-toolkit/
├── robot_ik/                  # Python 包 (pip install)
│   ├── __init__.py            # 包入口，统一导出，C++ 扩展检测
│   ├── ik_solver.py           # 运动学核心：FK、Jacobian、IK
│   ├── robot_dyn.py           # 刚体动力学：RNEA、CRBA
│   ├── urdf_parser.py         # URDF 解析：提取质量、惯量、DH 参数
│   ├── visualize.py           # 3D 可视化：matplotlib 绘制机械臂
│   └── ik_fast_wrapper.py     # C++ IK 扩展的 Python 接口
├── ik_fast.cpp                # C++ 运动学扩展 (pybind11)
├── robot_dyn_fast.cpp         # C++ 动力学扩展 (pybind11)
├── setup.py                   # 构建配置：pip install + C++ 编译
├── test_ik.py                 # 运动学测试套件 + benchmark
├── test_dyn.py                # 动力学测试套件 + benchmark
├── requirements.txt           # 依赖声明
├── README.md                  # 快速入门
├── ROADMAP.md                 # 开发路线图
└── docs/                      # 文档
```

## 部件详解

### 1. ik_solver.py — 运动学核心

这是整个工具包的基础模块，提供串联机械臂的正向运动学 (FK)、雅可比矩阵计算和逆运动学 (IK) 求解。

#### 数据结构

**DHParam** — Denavit-Hartenberg 参数

每个关节用四个参数描述：
- `a` (link length): 沿 x 轴的平移距离 (米)
- `alpha` (link twist): 绕 x 轴的旋转角度 (弧度)
- `d` (link offset): 沿 z 轴的平移距离 (米)
- `theta` (joint angle): 绕 z 轴的旋转角度 (弧度)，对于旋转关节这是变量

**RobotModel** — 机器人模型

由一组 DHParam 定义完整机器人，可选指定关节限位 `[(min, max)]`。

#### 核心函数

**dh_transform(dh) → 4×4 矩阵**

从单个 DH 参数计算齐次变换矩阵：

```
T = Rot_z(θ) · Trans_z(d) · Trans_x(a) · Rot_x(α)
```

这是标准 DH 约定。返回的 4×4 矩阵将坐标系 {i} 变换到坐标系 {i-1}。

**forward_kinematics(joint_angles) → 4×4 矩阵**

将所有 DH 变换矩阵依次相乘，得到从基座到末端执行器的变换。

```
T_0→6 = T_01 · T_12 · T_23 · T_34 · T_45 · T_56
```

可选参数 `return_all=True` 返回所有中间变换矩阵，用于雅可比计算和可视化。

**compute_jacobian(joint_angles) → 6×6 矩阵**

计算几何雅可比矩阵，描述关节速度到末端速度的映射：

```
[v_x, v_y, v_z, ω_x, ω_y, ω_z]ᵀ = J(q) · [q̇₁, q̇₂, ..., q̇₆]ᵀ
```

对第 i 个关节：
- 线速度部分：`J_v,i = z_i × (p_ee - p_i)`
- 角速度部分：`J_ω,i = z_i`

其中 `z_i` 是第 i 个关节坐标系的 z 轴在世界坐标系中的方向，`p_i` 是第 i 个关节坐标系原点。

**ik_solve(target_pose) → (success, joint_angles, iterations, errors)**

使用阻尼最小二乘法 (Damped Least Squares, DLS) 求解逆运动学。

算法步骤：
1. 计算当前位姿与目标位姿的误差（位置误差 + 姿态误差）
2. 姿态误差通过旋转矩阵差提取轴角表示
3. 计算雅可比矩阵 J
4. 自适应阻尼：`λ = λ₀ · (1 + 0.1 · log₁₀(cond(J·Jᵀ)))`
5. 关节增量：`Δq = Jᵀ · (J·Jᵀ + λ²I)⁻¹ · error`
6. 更新关节角并裁剪到限位范围
7. 检查收敛（位置 < 0.1 mm, 姿态 < 0.001 rad）

收敛后对关节角做限位裁剪，确保结果在安全范围内。

#### 预置机器人模型

**six_dof_articulated()** — 标准 6 轴工业机器人

DH 参数对应典型的人形臂构型：基座旋转、肩关节、肘关节 + 球形腕 (3 个相交轴)。关节限位按实际工业机器人范围设定。

**spherical_wrist_6dof()** — 球形腕构型

解析可解的特殊几何构型，用于验证数值 IK 算法的正确性。

---

### 2. robot_dyn.py — 刚体动力学

提供串联机械臂的刚体动力学计算，用于力控制、重力补偿和仿真。

#### 数据结构

**LinkInertia** — 连杆惯性属性

- `mass`: 质量 (kg)
- `com`: 质心在连杆坐标系中的位置向量 (3,)
- `inertia`: 关于质心的 3×3 转动惯量张量 (kg·m²)

**RobotDynamicsModel** — 动力学模型

将 DH 参数与惯性属性组合，附加重力向量和关节阻尼。

#### 核心算法

**inverse_dynamics(q, qd, qdd) → τ** — 递推牛顿-欧拉 (RNEA)

计算给定运动状态 (位置、速度、加速度) 下各关节所需的驱动力矩。

运动方程：
```
τ = M(q)·q̈ + C(q,q̇)·q̇ + G(q)
```

RNEA 算法步骤：
1. **正向递推** (基座→末端)：逐连杆计算角速度、角加速度、线加速度、质心加速度
2. **反向递推** (末端→基座)：逐连杆计算惯性力、惯性力矩，投影到关节轴得到驱动力矩

全部在世界坐标系 (Base Frame) 中计算，避免坐标系变换的复杂性。

**gravity_torque(q) → τ_g** — 重力补偿力矩

令 q̇=0, q̈=0 调用逆动力学，提取纯重力项。用于重力补偿控制。

**coriolis_torque(q, qd) → τ_c** — 科里奥利力矩

从完整逆动力学中减去重力项，得到科里奥利力和离心力项。

**inertia_matrix(q) → H** — 关节空间惯性矩阵

通过有限差分法计算：
```
H[i,j] = ∂τᵢ/∂q̈ⱼ (q̇=0, g=0)
```

精度依赖于有限差分步长 (ε=1e-6)，对于 6 自由度系统足够精确。

**forward_dynamics(q, qd, τ) → q̈** — 正向动力学 (CRBA)

已知关节力矩，求解关节加速度：
```
q̈ = H(q)⁻¹ · (τ - C(q,q̇)·q̇ - G(q))
```

惯性矩阵奇异时自动添加正则化项。

#### 预置模型

**six_dof_articulated_dyn()** — 带有真实质量参数的 6 轴机器人

各连杆质量从基座到末端递减 (2.0, 3.0, 2.0, 1.5, 0.8, 0.3 kg)，惯量张量根据简化几何估算。

---

### 3. urdf_parser.py — URDF 模型导入

从 URDF (Unified Robot Description Format) 文件中提取机器人参数，转换为 `RobotDynamicsModel`。

#### 解析流程

1. 解析 XML，索引所有 `<link>` 和 `<joint>` 元素
2. 提取每个 link 的质量、质心位置、惯量张量
3. 提取每个 joint 的类型、父子关系、坐标系变换、旋转轴、关节限位
4. 从根节点开始 DFS 遍历运动链
5. 将 URDF 关节参数转换为 DH 参数：
   - `d = p_j · z_parent` (沿关节轴的偏移)
   - `a = ‖p_j - d·z_parent‖` (垂直于关节轴的距离)
   - `alpha = arccos(z_prev · z_parent)` (相邻关节轴夹角)
6. 将质心从 URDF 连杆坐标系转换到 DH 连杆坐标系：`com_dh = com_urdf - [a, 0, d]`

#### 主要函数

**urdf_to_dynamics_model(urdf_path) → RobotDynamicsModel**

完整的 URDF 到动力学模型转换，可直接用于 `DynamicsSolver`。

**quick_urdf(urdf_path) → DynamicsSolver**

一步到位：解析 URDF 并创建动力学求解器实例。

---

### 4. ik_fast.cpp — C++ 运动学加速

使用 pybind11 将正向运动学和 IK 核心循环编译为 C++ 扩展。

#### 实现细节

- DH 变换使用展开的三角函数计算，避免矩阵通用乘法开销
- 4×4 矩阵乘法手动展开为 16 个浮点运算
- FK 使用固定长度 double 数组 (`double T[16]`)，避免堆分配
- IK 循环完全在 C++ 中执行，避免 Python ↔ C++ 反复调用开销

编译选项：`-O3 -march=native -ffast-math`

#### 性能对比

| 指标 | Python | C++ | 加速比 |
|------|--------|-----|--------|
| 平均求解时间 | 12.6 ms | 0.09 ms | 137× |
| P50 | 5.4 ms | 0.03 ms | 180× |
| P95 | 36.9 ms | 0.56 ms | 66× |

---

### 5. robot_dyn_fast.cpp — C++ 动力学加速

将 RNEA 算法编译为 C++ 扩展。

#### 实现细节

- 使用 `std::vector<double>` 存储中间变量 (角速度、角加速度等)
- 3×3 矩阵运算手动展开
- 惯量张量旋转变换 `I_b = R·I·Rᵀ` 在循环外预计算
- 叉积使用展开的三分量公式

#### 性能对比

| 指标 | Python | C++ | 加速比 |
|------|--------|-----|--------|
| 平均计算时间 | ~3 ms | ~0.008 ms | 358× |

---

### 6. visualize.py — 3D 可视化

基于 matplotlib 的轻量级 3D 可视化工具。

#### 功能

**plot_robot(robot, joint_angles, target_pose)**

绘制机械臂的 3D 形态：
- 各关节用圆点标记
- 连杆用线段连接
- 末端坐标系用 RGB 三色箭头表示 (红=x, 绿=y, 蓝=z)
- 可选显示目标位姿的半透明坐标系

**plot_convergence(error_history)**

绘制 IK 求解的收敛曲线，展示误差随迭代次数的变化。

---

### 7. __init__.py — 包入口

统一导出所有公共 API，处理 C++ 扩展的加载：

```python
from robot_ik import RobotModel, DynamicsSolver, urdf_to_dynamics_model
```

C++ 扩展加载失败时静默降级，设置 `HAS_IK_FAST=False` / `HAS_DYN_FAST=False`。

---

### 8. setup.py — 构建系统

使用 setuptools + pybind11 构建 Python 包和 C++ 扩展。

```bash
pip install -e .                    # 开发模式安装 (不编译 C++)
python setup.py build_ext --inplace # 编译 C++ 扩展
pip install .                       # 完整安装 (含 C++ 编译)
```

C++ 编译使用 `-O3 -march=native -ffast-math` 优化选项。
