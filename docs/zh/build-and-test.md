[English](../build-and-test.md) | [中文](build-and-test.md)

# 构建与测试指南

## 环境要求

- Python >= 3.8
- C++ 编译器 (GCC >= 7, Clang >= 5, MSVC >= 19)
- pip

## 安装方式

### 1. 纯 Python (无 C++ 加速)

```bash
pip install -r requirements.txt
```

所有功能可用，但运行速度为 Python 级别。

### 2. 开发模式安装

```bash
pip install -e .
```

### 3. 完整安装 (含 C++ 加速)

```bash
# 安装 pybind11
pip install pybind11

# 编译 C++ 扩展
python setup.py build_ext --inplace

# 或直接安装
pip install .
```

编译选项已配置为 `-O3 -march=native -ffast-math`，自动利用本机 CPU 指令集。

### 4. 验证安装

```python
from robot_ik import HAS_IK_FAST, HAS_DYN_FAST
print(f"C++ IK:  {HAS_IK_FAST}")
print(f"C++ Dyn: {HAS_DYN_FAST}")
```

---

## 测试套件

### 运动学测试 (test_ik.py)

```bash
python test_ik.py
```

包含 6 个测试用例 + benchmark：

| 测试 | 验证内容 |
|------|----------|
| test_fk_identity | 零角度 FK 结果为有效 4×4 变换，旋转正交 |
| test_ik_roundtrip | 20 个随机位姿的 IK 往返精度 < 5mm |
| test_ik_orientation | 10 个随机位姿的姿态精度 |
| test_jacobian_numerical | 解析 Jacobian 与数值有限差分一致性 |
| test_joint_limits | IK 解遵守关节限位 |
| test_custom_robot | 自定义 DH 参数机器人的 IK 正确性 |
| benchmark | 200 次求解的性能统计 |

### 动力学测试 (test_dyn.py)

```bash
python test_dyn.py
```

包含 5 个测试用例 + benchmark：

| 测试 | 验证内容 |
|------|----------|
| test_pendulum_gravity | 单摆重力力矩与解析解 `mgL sin(θ)` 的对比 |
| test_pendulum_zero_velocity | 零速零加速时仅有重力力矩 |
| test_coriolis_no_gravity | 科里奥利力矩排除了重力分量 |
| test_6dof_gravity_nonzero | 6 轴机器人在零位存在重力力矩 |
| test_6dof_roundtrip | `τ_full = τ_grav + τ_cor` 的分解一致性 |
| benchmark | 500 次逆动力学的性能统计 |

### 测试方法论

#### 解析验证 (Analytical Validation)

对于存在解析解的特殊情况，直接对比：

```python
# 单摆重力力矩的理论值
expected = m * g * L * sin(θ)
assert abs(tau_computed - expected) < 0.02
```

#### 往返验证 (Roundtrip Validation)

FK → IK → FK：原始关节角经 FK 得到位姿，再经 IK 求解，验证回代 FK 的位姿一致性。

逆动力学 → 正动力学：给定力矩通过正动力学得到加速度，验证逆动力学恢复原力矩。

#### 数值交叉验证 (Numerical Cross-Validation)

解析 Jacobian 与数值有限差分 Jacobian 对比：

```python
J_numerical[:, i] = (FK(q + ε·eᵢ) - FK(q)) / ε
assert ‖J_analytical - J_numerical‖ < 1e-3
```

#### 随机测试 (Randomized Testing)

使用固定随机种子生成大量随机配置，确保算法在非特殊位形下仍然正确：

```python
np.random.seed(123)
for _ in range(N):
    q = np.random.uniform(-1, 1, 6)
    # ... 验证逻辑
```

#### 边界条件测试

- 零位 (q = 0)
- 关节限位边界
- 奇异位形附近 (通过阻尼处理，不应崩溃)

### 添加新测试

测试风格约定：

```python
def test_something():
    """简短描述验证什么。"""
    # Arrange
    robot = six_dof_articulated()
    q = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])

    # Act
    result = some_function(q)

    # Assert
    assert condition, f"Descriptive error: actual={value}"

    print(f"  [PASS] test_something (extra info)")
```

---

## 构建与打包

### 构建 C++ 扩展

```bash
python setup.py build_ext --inplace
```

生成的 `.so` (Linux) / `.pyd` (Windows) 文件位于 `robot_ik/` 下。

### 构建 Wheel

```bash
pip install build
python -m build
```

输出在 `dist/` 目录。

### 清理

```bash
rm -rf build/ dist/ *.egg-info robot_ik/*.so robot_ik/*.pyd
```
