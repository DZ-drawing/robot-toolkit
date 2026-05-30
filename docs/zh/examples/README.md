[English](../../examples/README.md) | [中文](README.md)

# 示例

使用 robot-toolkit 构建的机器人项目示例与演示。每个示例展示一个真实应用场景。

部分示例**完整可运行**，其他为**进行中**的草稿，展示了工具箱的潜在改进方向和实际应用中缺失的功能。

## 运行示例

```bash
pip install -e .        # 从源码安装
python examples/<name>.py
```

## 完整示例

| 示例 | 描述 | 使用的模块 |
|------|------|------------|
| [solve_ik.py](./solve_ik.py) | 对目标位置求解 IK，通过 FK 验证 | ik_solver |
| [tutorial_ik.ipynb](./tutorial_ik.ipynb) | 交互式 Jupyter 教程 | ik_solver |

## 项目演示

| 示例 | 状态 | 描述 | 使用的模块 |
|------|------|------|------------|
| [dual_arm_pick_place.py](./dual_arm_pick_place.py) | 草稿 | 带碰撞检测的双臂抓取放置 | ik_solver, trajectory, collision, path_planning |
| [force_simulation.py](./force_simulation.py) | 草稿 | 使用动力学求解器的简单力/力矩仿真 | robot_dyn, trajectory |

## 贡献示例

欢迎添加项目演示：

1. 在此目录创建 `your_example.py`
2. 添加 docstring 说明项目想法
3. 在本 README 中标记为**草稿**或**完成**
4. 如果暴露了缺失的工具箱功能，添加 `# TODO:` 注释说明需要改进的内容

### 示例模板

```python
"""
PROJECT NAME — Status: Draft/Complete

DESCRIPTION of the robotics project.

Modules used: ik_solver, collision, trajectory
Missing features: [list any toolkit improvements needed]
"""

import numpy as np
from robot_ik import ...

# TODO: implement
```
