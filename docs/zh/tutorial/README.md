[English](../../tutorial/README.md) | **中文**

# robot-toolkit 教程

robot-toolkit 的分步教程 —— 一个纯 Python 的 6 自由度逆运动学、动力学、轨迹、碰撞和路径规划库。

## 快速开始

安装并验证：

```bash
pip install robot-ik
python -c "from robot_ik import six_dof_articulated; print('OK')"
```

## 按难度分类的教程

### 入门

| # | 教程 | 主题 | 预计时间 |
|---|------|------|----------|
| [T0](./t0-getting-started.md) | 入门与 URDF | 安装、URDF 导入、正运动学、逆运动学 | 15 分钟 |
| [T1](./t1-workspace-analysis.md) | 双臂工作空间 | 正运动学采样、3D 可视化、重叠区域 | 20 分钟 |
| [T2](./t2-collision-detection.md) | 碰撞检测 | CollisionChecker、基本几何体、安全检查 | 20 分钟 |

### 中级

| # | 教程 | 主题 | 预计时间 |
|---|------|------|----------|
| [T3](./t3-trajectory-planning.md) | 轨迹规划 | 多路点、S 曲线、双臂同步 | 25 分钟 |
| [T4](./t4-path-planning-rrt.md) | RRT* 路径规划 | 基于采样的规划、碰撞约束 | 25 分钟 |
| [T5](./t5-dynamics.md) | 机器人动力学 | 逆动力学、重力补偿、惯量 | 25 分钟 |

### 高级

| # | 教程 | 主题 | 预计时间 |
|---|------|------|----------|
| [T6](./t6-meshcat-visualization.md) | Meshcat 3D 可视化 | 基于网络的实时机器人可视化 | 30 分钟 |
| [T7](./t7-cartesian-path-collision.md) | 笛卡尔路径与碰撞 | 直线笛卡尔运动、实时碰撞检测 | 30 分钟 |

## 模块覆盖

| 模块 | 教程 |
|------|------|
| `ik_solver` | T0, T1, T2, T3, T4, T7 |
| `robot_dyn` | T5 |
| `trajectory` | T3, T7 |
| `collision` | T2, T4, T7 |
| `path_planning` | T4, T7 |
| `urdf_parser` | T0 |
| `visualize` (matplotlib) | T1, T2 |
| `visualize_meshcat` | T6 |
| `hardware` | T6 |

## 前置要求

- Python 3.10+
- `numpy`（自动安装）
- `matplotlib`（T1、T2 所需）
- `meshcat`（T6 所需）
- 无需硬件 —— 所有教程均在软件仿真中运行

## 运行方式

每个教程都是一个独立的 Markdown 文件，包含可运行的代码块。将代码复制到 `.py` 文件或 Jupyter notebook 中执行即可。

## 贡献

查看[贡献指南](../../contributing.md)了解如何添加新教程。
