[English](../../tutorial/t1-workspace-analysis.md) | **中文**

# 教程 1：双臂工作空间分析

**目标**：可视化和分析并排放置的两个 6 自由度机械臂的重叠可达工作空间。

**演示功能**：
- `six_dof_articulated()` 模型创建
- 用于工作空间点采样的正运动学
- 带基座偏移的双臂设置
- 工作空间交集估计

---

## 工作原理

我们创建两个标准 6 自由度铰接机器人的实例。第二个臂沿 X 轴偏移 1.5 米。我们对每个臂采样随机关节角度配置，通过正运动学计算末端执行器位置，然后估计它们共享工作空间的体积。

```python
"""教程 1：双臂工作空间分析。

可视化和分析两个 6 自由度机械臂的重叠可达工作空间。
运行方式：python -m docs.tutorial.t1_workspace_analysis
"""

from robot_ik import six_dof_articulated
import numpy as np


def sample_workspace(robot, n_samples: int = 5000, seed: int = 42) -> np.ndarray:
    """通过随机关节角度扫描采样可达的末端执行器位置。

    返回：
        (n_samples, 3) 的 [x, y, z] 位置数组。
    """
    rng = np.random.default_rng(seed)
    points = np.zeros((n_samples, 3))

    for i in range(n_samples):
        # 在机器人关节限位内的随机角度
        q = np.array([
            rng.uniform(lo, hi)
            for lo, hi in robot.joint_limits
        ])
        T = robot.forward_kinematics(q)
        points[i] = T[:3, 3]

    return points


def estimate_overlap_volume(
    pts_a: np.ndarray,
    pts_b: np.ndarray,
    resolution: float = 0.1,
) -> dict:
    """使用体素占位粗略估计共享工作空间的体积。

    参数：
        pts_a, pts_b: (N, 3) 工作空间点云。
        resolution: 体素边长，单位为米。

    返回：
        包含 'volume'、'overlap_count' 等键的字典。
    """
    # 合并点云，对齐到体素网格
    all_pts = np.vstack([pts_a, pts_b])

    vox_a = set()
    vox_b = set()

    for p in pts_a:
        vox_a.add(tuple(np.floor(p / resolution).astype(int)))
    for p in pts_b:
        vox_b.add(tuple(np.floor(p / resolution).astype(int)))

    overlap_voxels = vox_a & vox_b
    voxel_volume = resolution ** 3

    return {
        "volume": len(overlap_voxels) * voxel_volume,
        "overlap_count": len(overlap_voxels),
        "voxel_resolution": resolution,
    }


def main() -> None:
    # -- 构建两个机械臂 ------------------------------------------------
    arm1 = six_dof_articulated()
    arm2 = six_dof_articulated()

    # 将 arm2 的基座沿 X 轴偏移 1.5 m。
    # RobotModel 存储 DH 参数；我们对 arm2 的正运动学输出进行偏移。
    arm2_base_offset = np.array([1.5, 0.0, 0.0])

    def fk_arm2(q: np.ndarray) -> np.ndarray:
        T = arm2.forward_kinematics(q)
        T[:3, 3] += arm2_base_offset
        return T

    # -- 采样工作空间 -------------------------------------------------
    n = 5000
    ws1 = sample_workspace(arm1, n_samples=n)
    ws2_raw = sample_workspace(arm2, n_samples=n)
    # 将基座偏移应用到 arm2 的工作空间点
    ws2 = ws2_raw + arm2_base_offset

    print(f"臂 1 工作空间：{n} 个采样点，"
          f"X 范围 [{ws1[:, 0].min():.2f}, {ws1[:, 0].max():.2f}] m")
    print(f"臂 2 工作空间：{n} 个采样点，"
          f"X 范围 [{ws2[:, 0].min():.2f}, {ws2[:, 0].max():.2f}] m")

    # -- 计算重叠量 ---------------------------------------------------
    overlap = estimate_overlap_volume(ws1, ws2)
    print(f"\n共享工作空间体积：{overlap['volume']:.4f} m^3  "
          f"({overlap['overlap_count']} 个体素 @ {overlap['voxel_resolution']} m)")

    return ws1, ws2, overlap


if __name__ == "__main__":
    main()
```

## 学习要点

- 通过随机关节角度扫描和正运动学进行工作空间采样
- 双臂设置和坐标系偏移
- 基于体素的重叠估计用于共享任务规划
- 理解基座位置如何影响可用工作空间的实际应用

## 注意事项

- 增加 `n_samples` 可提高精度，但会降低速度。
- 体素分辨率（默认 `0.1 m`）控制体积估计的粒度。
- 如需出版级工作空间地图，使用 `matplotlib` 3D 散点图或 `pyvista` 网格。
