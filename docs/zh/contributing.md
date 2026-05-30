[English](../contributing.md) | [中文](contributing.md)

# 参与开源开发

感谢你对 robot-toolkit 的关注！本文档说明如何参与项目的开发。

## 快速开始

```bash
# 克隆仓库
git clone https://github.com/your-org/robot-toolkit.git
cd robot-toolkit

# 安装依赖
pip install -r requirements.txt

# (可选) 编译 C++ 扩展
pip install pybind11
python setup.py build_ext --inplace

# 运行测试
python test_ik.py
python test_dyn.py
```

## 代码风格

### Python

- 遵循 PEP 8，使用 4 空格缩进
- 使用 `numpy` 和 `dataclasses`，不引入不必要的第三方库
- 所有公共函数必须有 docstring (Google 风格)
- 使用 type hints：函数签名中标注参数和返回类型
- 变量命名：
  - `q` 关节角, `qd` 关节角速度, `qdd` 关节角加速度
  - `tau` 力矩
  - `T` 齐次变换矩阵, `R` 旋转矩阵
  - `J` 雅可比矩阵, `H` 惯性矩阵

### C++

- 使用 C++14 标准 (pybind11 要求)
- 手动展开矩阵运算，避免通用矩阵库依赖
- 函数命名：`snake_case`
- 注释使用 `//` 风格

### 示例

```python
def some_function(q: np.ndarray, max_iterations: int = 200) -> Tuple[bool, np.ndarray, int]:
    """Short description of what this function does.

    Args:
        q: Joint angles in radians, shape (6,).
        max_iterations: Maximum number of iterations.

    Returns:
        Tuple of (success, result, iterations).
    """
    ...
```

## 开发流程

### 1. 创建分支

```bash
git checkout -b feature/your-feature-name
```

分支命名约定：
- `feature/` 新功能
- `fix/` 错误修复
- `refactor/` 代码重构
- `docs/` 文档更新

### 2. 编写代码

#### 新功能开发 (推荐 TDD)

1. 在 `test_*.py` 中先写测试 (RED)
2. 实现功能使测试通过 (GREEN)
3. 优化代码 (REFACTOR)
4. 确认所有测试仍然通过

#### 测试要求

- 每个新功能至少 2 个测试用例
- 使用 `[PASS]` 格式打印成功信息
- 使用固定随机种子 (`np.random.seed(N)`) 确保可复现
- 验证数值精度时使用 `assert` 附带描述性错误信息

```python
def test_new_feature():
    """One-line description of what is tested."""
    # Arrange
    robot = six_dof_articulated()

    # Act
    result = new_function(robot)

    # Assert
    assert condition, f"Expected X, got {actual}"

    print(f"  [PASS] test_new_feature (info)")
```

### 3. 运行完整测试

```bash
python test_ik.py
python test_dyn.py
```

确保所有测试通过，包括你新增的和原有的。

### 4. 提交代码

```bash
git add .
git commit -m "feat: 简短描述变更内容"
```

提交信息约定 (Conventional Commits)：

| 前缀 | 用途 |
|------|------|
| `feat:` | 新功能 |
| `fix:` | 错误修复 |
| `refactor:` | 代码重构 |
| `test:` | 测试相关 |
| `docs:` | 文档更新 |
| `perf:` | 性能优化 |
| `chore:` | 构建、工具变更 |

### 5. 发起 Pull Request

- PR 标题使用 conventional commit 格式
- 描述中说明：做了什么、为什么做、如何验证
- 关联相关 Issue (如有)
- 确保所有测试通过

## 项目模块与职责

如果你要添加新功能，请放在合适的模块中：

| 模块 | 职责 | 何时添加 |
|------|------|----------|
| `ik_solver.py` | 运动学 (FK, Jacobian, IK) | 新的运动学算法、新机器人模型 |
| `robot_dyn.py` | 动力学 (RNEA, CRBA) | 新的动力学算法 |
| `trajectory.py` | 轨迹规划 | 插值、速度规划、路径优化 |
| `urdf_parser.py` | URDF 解析 | 新的文件格式支持 |
| `visualize.py` | 可视化 | 新的可视化功能 |
| `*_fast.cpp` | C++ 加速 | 已有 Python 算法的 C++ 移植 |

添加新模块时，记得在 `__init__.py` 中添加导出。

## 性能优化指南

### Python 层面

- 使用 `numpy` 向量化操作，避免 Python 循环
- 预分配数组 (`np.zeros`, `np.empty`) 而非动态 append
- 避免在循环中创建临时数组

### C++ 加速

当 Python 实现的性能不满足需求时：

1. 确保算法正确 (Python 测试全部通过)
2. 在 `*_fast.cpp` 中实现 C++ 版本
3. 使用 pybind11 暴露为 Python 函数
4. 在 `__init__.py` 中添加可选导入 (失败时回退 Python)
5. 对比测试：C++ 结果与 Python 结果的数值一致性

### 基准测试

每个模块都包含 benchmark 函数。修改核心算法后请运行并报告性能变化：

```python
# 在测试文件中
def benchmark():
    times = []
    for _ in range(N):
        start = time.perf_counter()
        function()
        times.append(time.perf_counter() - start)
    t = np.array(times) * 1000
    print(f"Avg: {np.mean(t):.2f} ms")
    print(f"P50: {np.median(t):.2f} ms")
    print(f"P95: {np.percentile(t, 95):.2f} ms")
```

## 报告问题

提交 Issue 时请包含：

1. Python 版本和操作系统
2. 安装方式 (pip / 源码编译 / 是否有 C++ 扩展)
3. 复现步骤
4. 期望行为和实际行为
5. 相关的代码片段和错误信息

## 路线图

参考 [ROADMAP.md](../ROADMAP.md) 了解项目的发展方向。欢迎领取未完成的任务，或在 Issue 中讨论新功能建议。

## 许可证

本项目使用 MIT 许可证。贡献的代码将同样以 MIT 许可证发布。
