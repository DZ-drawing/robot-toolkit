[English](../benchmark.md) | [中文](benchmark.md)

# 性能基准测试

robot-toolkit 各模块 (运动学、动力学、轨迹规划) 的基准测试结果。

## 运行基准测试

```bash
python3 benchmark.py
```

## 测试结果

### 逆运动学 (IK)

| 指标 | 数值 |
|------|------|
| 平均耗时 | ~4.9 ms |
| P50 耗时 | ~2.4 ms |
| P95 耗时 | ~17.4 ms |
| 平均迭代次数 | ~29 |
| 失败率 | 14% (随机位姿) |

**说明：**
- 200 个随机目标位姿
- 阻尼最小二乘法 (DLS)
- 每次求解最多 100 次迭代
- 失败原因：不可达构型或奇异位形

### 刚体动力学

| 操作 | 平均耗时 | P50 耗时 | P95 耗时 |
|------|----------|----------|----------|
| 逆动力学 (RNEA) | ~819 μs | ~793 μs | ~1.04 ms |
| 正动力学 (CRBA) | ~6.25 ms | ~6.17 ms | ~7.02 ms |
| 质量矩阵 (CRBA) | ~5.42 ms | ~5.36 ms | ~6.14 ms |

**说明：**
- 1000 个随机关节构型
- RNEA (递推牛顿-欧拉算法) 用于逆动力学
- CRBA (复合刚体算法) 用于正动力学
- 6 自由度机械臂

### 轨迹规划

| 方法 | 平均耗时 | P50 耗时 | P95 耗时 |
|------|----------|----------|----------|
| 五次多项式插值 | ~2.27 ms | ~2.26 ms | ~3.28 ms |
| 梯形速度规划 | ~0.88 ms | ~0.88 ms | ~1.07 ms |
| 多点轨迹 (5 个路径点) | ~3.85 ms | ~3.72 ms | ~4.21 ms |

**说明：**
- 100 条随机轨迹
- 6 自由度关节空间
- dt = 0.01 (100 Hz 采样)

## C++ 扩展加速

本仓库包含 C++ 扩展，可显著提升性能：

| 模块 | Python 耗时 | C++ 耗时 | 加速比 |
|------|-------------|----------|--------|
| IK 求解器 | ~12 ms | ~0.09 ms | 137× |
| 动力学 (RNEA/CRBA) | ~180 μs | ~0.5 μs | 358× |

**启用 C++ 扩展：**

```bash
python3 setup.py build_ext --inplace
```

**注意：** `benchmark.py` 脚本会自动检测 C++ 扩展。如果已编译，将自动使用。

## 基准测试建议

### 获得一致结果

1. **使用一致的硬件：** 在同一台机器上运行以获得可比较的结果
2. **CPU 频率调节：** 禁用睿频以获得稳定的测量值
3. **多次运行：** 取 3-5 次的平均值以消除波动
4. **温度节流：** 在长时间基准测试期间注意 CPU 温度

### 自定义基准测试

```python
from robot_ik import six_dof_articulated
import time

robot = six_dof_articulated()
n_samples = 100

# 基准测试 IK
times = []
for _ in range(n_samples):
    target = robot.forward_kinematics(np.random.uniform(-1, 1, 6))
    start = time.perf_counter()
    success, q, _, _ = robot.ik_solve(target)
    times.append(time.perf_counter() - start)

print(f"平均: {np.mean(times) * 1000:.2f} ms")
```

## 性能优化

### IK 求解器
- **阻尼因子 (λ)：** 调节收敛速度与稳定性的平衡
- **最大迭代次数：** 在求解率和成功率之间权衡
- **初始猜测：** 对顺序任务使用上一次的解

### 动力学
- **C++ 扩展：** 358× 加速，适用于实时控制
- **批量操作：** 对多个构型进行向量化运算

### 轨迹规划
- **预分配数组：** 适用于大规模轨迹
- **降低采样率：** 使用更大的 dt (例如 0.02 而非 0.01) 以加快生成速度

## 硬件

**参考系统：**
- CPU：Intel/AMD x86_64
- Python：3.10+
- NumPy：1.24+

结果可能因 CPU 架构和主频而异。

## 贡献基准测试

添加新模块时，请按以下模式编写基准测试函数：

```python
def benchmark_your_module(n_samples: int = 100):
    """基准测试你的模块性能。"""
    print(f"\n=== 你的模块基准测试 ({n_samples} 个样本) ===")
    np.random.seed(42)

    times = []
    for _ in range(n_samples):
        # 设置
        start = time.perf_counter()
        # 操作
        elapsed = time.perf_counter() - start
        times.append(elapsed)

    times_ms = np.array(times) * 1000
    print(f"  平均耗时:  {np.mean(times_ms):.3f} ms")
    print(f"  P50 耗时:  {np.median(times_ms):.3f} ms")
    print(f"  P95 耗时:  {np.percentile(times_ms, 95):.3f} ms")
```

将函数添加到 `benchmark.py` 中的 `run_all_benchmarks()`。
