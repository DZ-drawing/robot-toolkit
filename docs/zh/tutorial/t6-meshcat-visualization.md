[English](../../tutorial/t6-meshcat-visualization.md) | **中文**

# T6：Meshcat 3D 可视化 —— 基于网络的实时机器人可视化

**难度：** 高级 | **时间：** 30 分钟 | **模块：** `robot_ik.visualize_meshcat`

本教程展示如何使用 `MeshcatVisualizer` 在浏览器和 Jupyter notebook 中进行交互式 3D 机器人可视化。你将设置可视化器、加载机器人模型、动画化关节运动，以及从硬件接口流式传输实时状态。

## 你将学到

1. 初始化 `MeshcatVisualizer` 并连接到浏览器
2. 加载带有程序化 3D 几何体的机器人模型
3. 更新关节角度并查看实时运动
4. 将可视化器嵌入 Jupyter notebook
5. 从硬件接口流式传输实时关节状态

## 前置要求

```bash
pip install robot-ik[meshcat]
# 或：pip install meshcat
```

用于 Jupyter 显示：

```bash
pip install ipython
```

```python
import numpy as np
```

---

## 步骤 1：创建可视化器

`MeshcatVisualizer` 启动一个基于 ZeroMQ 的 Web 服务器。在浏览器中打开 URL（推荐 Chrome）。它支持用作上下文管理器以可靠清理。

```python
from robot_ik.visualize_meshcat import MeshcatVisualizer

# 创建可视化器（默认在端口 7000 上启动服务器）
vis = MeshcatVisualizer(port=7000)
```

或使用上下文管理器（推荐用于脚本）：

```python
with MeshcatVisualizer(port=7000) as vis:
    # ... 使用 vis ...
    pass  # 清理自动进行
```

在浏览器中打开 **http://127.0.0.1:7000/static/**。你应该看到一个带有轨道控制的空 3D 视口。

### 连接到现有服务器

如果你已经有一个运行中的 Meshcat 服务器（例如来自另一个进程）：

```python
vis = MeshcatVisualizer(zmq_url="tcp://127.0.0.1:6000")
```

## 步骤 2：加载机器人模型

`set_robot()` 从机器人的 DH 参数生成程序化 3D 几何体：

- **基座：** 长方体
- **连杆 1-N：** 圆柱体
- **关节 1-N：** 球体（灰色）
- **每个关节坐标系：** RGB 坐标三脚架
- **末端执行器：** RGB 坐标三脚架

```python
from robot_ik import six_dof_articulated

robot = six_dof_articulated()

# 使用默认蓝色加载机器人
vis.set_robot(robot)

# 或使用自定义 RGBA 颜色 [R, G, B, A]
vis.set_robot(robot, color=[0.2, 0.8, 0.3, 1.0])  # 绿色
```

你的浏览器现在应该显示归零位的完整 6 自由度机器人。

### 机器人模型要求

机器人必须具有：
- `dof` 属性或 `dh_params` 属性（用于推断关节数量）
- `forward_kinematics(q, return_all=True)` 方法，返回 `(pose, transforms)`，其中 `transforms` 是 4x4 齐次矩阵的列表

## 步骤 3：更新关节角度

`update_joints(q)` 计算正运动学并更新 3D 场景中每个连杆、关节和坐标系变换：

```python
# 归零位
vis.update_joints(np.zeros(6))

# 向前伸展
vis.update_joints(np.array([0.0, np.pi/6, 0.0, 0.0, np.pi/4, 0.0]))

# 抬起肘部
vis.update_joints(np.array([0.0, np.pi/3, -np.pi/4, 0.0, np.pi/6, 0.0]))
```

### 验证

该方法会验证关节数组的形状以及是否已设置机器人：

```python
try:
    vis.update_joints(np.zeros(5))  # 错误的尺寸
except ValueError as e:
    print(f"预期的错误：{e}")
```

## 步骤 4：动画化轨迹

结合 `joint_cubic_interpolation` 实现平滑动画：

```python
import time
from robot_ik import joint_cubic_interpolation

q_start = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
q_end   = np.array([0.5, np.pi/4, -np.pi/6, 0.0, np.pi/3, 0.0])

traj = joint_cubic_interpolation(q_start, q_end, duration=3.0, dt=0.02)

print(f"轨迹：{len(traj.time_points)} 个路点，"
      f"{traj.duration:.1f}s")

# 实时动画
for i, t in enumerate(traj.time_points):
    vis.update_joints(traj.joint_positions[i])
    time.sleep(0.02)  # 匹配 dt 采样率
```

### 双臂动画

对于双臂设置，使用具有不同端口和命名空间路径的独立 MeshcatVisualizer 实例：

```python
vis_left  = MeshcatVisualizer(port=7000)
vis_right = MeshcatVisualizer(port=7001)

robot_left  = six_dof_articulated()
robot_right = six_dof_articulated()

vis_left.set_robot(robot_left)
vis_right.set_robot(robot_right)

# 镜像运动
for i in range(len(traj.time_points)):
    q = traj.joint_positions[i]
    vis_left.update_joints(q)
    vis_right.update_joints(q)  # 或镜像：q * [-1, 1, 1, -1, 1, 1]
    time.sleep(0.02)
```

## 步骤 5：Jupyter Notebook 显示

`start_jupyter()` 返回一个 `IFrame`，将 3D 视图直接嵌入到 notebook 单元格中：

```python
# 在 Jupyter notebook 单元格中：
iframe = vis.start_jupyter()
display(iframe)  # 以 800x600 像素内联渲染
```

URL 从 Meshcat 自动检测（支持旧版 `.url()` 和新版 `.viewer_url()` API）。

## 步骤 6：实时硬件流式传输

`start_realtime_stream()` 运行一个后台线程，轮询硬件接口并持续更新可视化。

### 硬件接口协议

你的硬件类必须实现：

```python
class MyHardware:
    def get_joint_positions(self) -> np.ndarray:
        """返回当前关节角度为 (6,) 数组。"""
        # 从串口、EtherCAT 等读取
        return np.array([q1, q2, q3, q4, q5, q6])
```

### 启动/停止流式传输

```python
hardware = MyHardware()

# 以 30 Hz 启动（默认）
vis.start_realtime_stream(hardware, freq=30)

# 后台线程自动运行。
# 在这里执行其他工作...

# 完成后停止
vis.stop_realtime_stream()
```

流式传输是线程安全的，使用锁保护。启动两次会引发 `RuntimeError`：

```python
try:
    vis.start_realtime_stream(hardware)
    vis.start_realtime_stream(hardware)  # 错误！
except RuntimeError as e:
    print(f"预期结果：{e}")
```

### 上下文管理器清理

当使用 `with MeshcatVisualizer() as vis:` 时，流式传输在退出时自动停止 —— 即使发生异常：

```python
with MeshcatVisualizer() as vis:
    vis.set_robot(robot)
    vis.start_realtime_stream(hardware, freq=30)
    time.sleep(5.0)  # 监控 5 秒
# 流式传输在此处自动停止
```

## 配置参考

| 参数 | 默认值 | 描述 |
|------|--------|------|
| `port` | 7000 | Meshcat 服务器端口 |
| `zmq_url` | None | ZeroMQ URL（如果为 None 则自动生成） |
| `DEFAULT_FREQ` | 30 | 默认流式传输频率 (Hz) |
| `BASE_SIZE` | [0.1, 0.1, 0.1] | 基座长方体尺寸 |
| `LINK_RADIUS` | 0.05 | 程序化连杆圆柱体半径 |
| `JOINT_RADIUS` | 0.06 | 程序化关节球体半径 |
| `DEFAULT_COLOR` | [0.3, 0.6, 0.9, 1.0] | 默认连杆颜色 (RGBA) |

## 错误处理

| 异常 | 触发条件 |
|------|----------|
| `InitializationError` | meshcat 未安装，或服务器启动失败 |
| `MeshcatError` | 机器人未设置，正运动学失败，模型创建失败 |
| `StreamingError` | 硬件接口缺少 `get_joint_positions()` |
| `ValueError` | 关节数组形状错误 |

## 关键要点

| 方法 | 用途 |
|------|------|
| `MeshcatVisualizer(port)` | 创建带 Web 服务器的可视化器 |
| `set_robot(robot, color)` | 使用程序化 3D 几何体加载机器人 |
| `update_joints(q)` | 通过正运动学更新所有关节变换 |
| `start_jupyter()` | 获取用于 notebook 嵌入的 IFrame |
| `start_realtime_stream(hw, freq)` | 启动后台硬件轮询 |
| `stop_realtime_stream()` | 停止后台轮询 |

## 下一步

- **T7：** 结合笛卡尔轨迹规划与碰撞检测
