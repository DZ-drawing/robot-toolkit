[English](../../ROADMAP.md) | [中文](ROADMAP.md)

# robot-toolkit 路线图

版本：0.3.0 | 最后更新：2026-05-27

## 项目概述

6-DOF 串联机械臂工程工具箱：逆运动学、刚体动力学、轨迹规划、碰撞检测、
路径规划、URDF 导入、可视化、硬件抽象及 C++ 加速。
设计理念：独立、可组合的模块 —— 类似机器人领域的 numpy/scipy。

---

## 已完成

### 阶段 1 —— 逆运动学（2026-05-08）
- [x] DH 参数正运动学
- [x] 阻尼最小二乘逆运动学（Levenberg-Marquardt）
- [x] 解析雅可比矩阵计算
- [x] 关节限制约束（梯度投影法）
- [x] 预置模型：6-DOF 关节型、球腕型
- [x] 3D matplotlib 可视化（机械臂 + 目标坐标系）
- [x] 测试套件：正运动学恒等性、逆运动学闭环、雅可比矩阵、关节限制、基准测试
- [x] 性能：平均 ~3 ms，位置精度 <0.1 mm

### 阶段 2 —— C++ 逆运动学扩展（2026-05-09）
- [x] pybind11 C++ 扩展（正运动学 + 雅可比矩阵 + 逆运动学循环）
- [x] 相比纯 Python 加速 137 倍（平均 ~0.09 ms）
- [x] C++ 未编译时优雅降级

### 阶段 3 —— 刚体动力学（2026-05-09）
- [x] 递归牛顿-欧拉逆动力学（RNEA）
- [x] 组合刚体算法（CRBA）正动力学
- [x] 重力力矩、科里奥利力、惯性矩阵计算
- [x] 逆/正动力学闭环测试（50 组配置）
- [x] 单摆重力验证（与解析解对比）

### 阶段 4 —— C++ 动力学扩展（2026-05-09）
- [x] pybind11 C++ RNEA 实现
- [x] 相比纯 Python 动力学加速 358 倍
- [x] 50 组配置验证（与 Python 参考实现对比）

### 阶段 5 —— URDF + 打包（2026-05-09）
- [x] URDF 解析器：质量、质心、惯量提取
- [x] URDF 到 DH 参数转换
- [x] `robot_ik` 命名空间包结构
- [x] `setup.py` 用于 pip 安装 + C++ build_ext

### 阶段 6 —— 轨迹规划（2026-05-12）
- [x] 关节空间插值（线性、三次、五次）
- [x] 笛卡尔空间直线插值（带 SLERP）
- [x] 梯形速度曲线（加速度限制）
- [x] S 曲线（7 段加加速度限制）
- [x] 航点轨迹（抛物线过渡）
- [x] 12 个 TDD 测试（边界条件、连续性）

### 阶段 7 —— CI/CD 流水线（2026-05-12）
- [x] GitHub Actions CI（Ubuntu/macOS/Windows，Python 3.10-3.12）
- [x] Pre-commit 钩子（black、ruff、mypy）
- [x] 依赖文件（开发依赖）
- [x] 代码覆盖率报告

### 阶段 8 —— 碰撞检测（2026-05-12）
- [x] 几何图元：球体、胶囊体、长方体
- [x] 距离函数（球-球、球-胶囊等）
- [x] 自碰撞检测（相邻连杆过滤）
- [x] 环境障碍物碰撞
- [x] 接触点近似
- [x] 10 个综合测试

### 阶段 9 —— 动力学基准测试（2026-05-12）
- [x] 性能测试套件：逆运动学、动力学、轨迹规划
- [x] 基准测试文档（结果、优化建议）
- [x] C++ 加速对比框架

### 阶段 10 —— 路径规划（2026-05-12）
- [x] RRT* 算法实现
- [x] 无碰撞路径规划
- [x] 路径平滑（shortcut）
- [x] 3 个测试用例（基础、碰撞、便捷性）

### 阶段 11 —— ROS2 集成（2026-05-12）
- [x] ROS2 包结构（package.xml、setup.py）
- [x] 逆运动学服务节点示例
- [x] Launch 文件和文档

### 阶段 12 —— 示例与教程（2026-05-12）
- [x] Jupyter notebook：逆运动学教程
- [x] 常见任务示例脚本
- [x] 4 个挑战教程（工作空间分析、碰撞检测、协调轨迹、路径规划）
- [x] API 文档更新

### 阶段 13 —— 许可证与法律（2026-05-12）
- [x] MIT LICENSE 文件添加
- [x] 项目文件中许可证一致性
- [x] setup.py 许可证字段验证

### 阶段 14 —— PyPI 发布配置（2026-05-12）
- [x] cibuildwheel GitHub Actions 工作流（Linux/macOS/Windows）
- [x] pyproject.toml 完整 PyPI 元数据
- [x] MANIFEST.in 包资源文件
- [x] 发布文档（docs/RELEASE.md）
- [x] 多平台 wheel 构建配置（Python 3.10-3.12）

### 阶段 15 —— PyPI Token 与首次发布（2026-05-13）
- [x] PyPI API Token 配置于 GitHub Secrets
- [x] 版本升级至 0.3.0
- [x] CI 工作流修复（YAML 语法、CMAKE_ARGS、可移植 wheel）

### 阶段 A —— 自托管 macOS 运行器（2026-05-20）
- [x] Mac mini M1 环境配置（用户 `danny`，IP 192.168.3.143）
- [x] 组织级运行器注册至 DZ-drawing
- [x] GitHub Actions 工作流使用自托管 macOS（ci.yml + build-wheels.yml）
- [x] cibuildwheel 绕过 pypa action，使用 `pip install cibuildwheel` + `CIBW_PYTHON_SOURCE=system`
- [x] 为 CI 任务配置免密 sudo
- [x] CI 稳定化（setuptools 安装、ruff lint 清扫、python3 -m cibuildwheel）

### 阶段 B —— Meshcat 可视化与硬件 HAL（2026-05-21）
- [x] MeshcatVisualizer 类（程序化 3D 机器人、set_robot、update_joints）
- [x] Jupyter 集成（start_jupyter 返回 IFrame）
- [x] 多线程实时流传输（30 Hz）
- [x] HardwareInterface ABC + SimulatedHardware + HardwareRegistry
- [x] Meshcat 可视化测试（未安装 meshcat 时跳过）
- [x] 硬件 HAL 测试

### 阶段 C —— 代码质量与结构（2026-05-21）
- [x] Ruff lint 清扫：修复 108 个错误（89 个自动、9 个不安全、10 个手动）
- [x] Black 格式化强制执行
- [x] Ruff 配置迁移至 `[tool.ruff.lint]`（弃用修复）
- [x] 审查者修复：动态 DOF 支持、上下文管理器、线程安全
- [x] 教程重组至 `docs/tutorial/`（8 个独立指南 T0-T7）
- [x] 示例重构为项目演示（未完成草稿）

### 阶段 D —— 项目重构为 src 布局（2026-05-21）
- [x] 迁移至 PEP 421 src 布局：`src/robot_ik/` 含 8 个子包
- [x] 子包：ik/、dynamics/、trajectory/、collision/、path_planning/、urdf/、visualization/、hardware/
- [x] C++ 源码移至 `csrc/`
- [x] 测试按模块重组至 `tests/`
- [x] 所有跨模块导入已更新
- [x] 向后兼容 `__init__.py` 重导出保留
- [x] CI 工作流更新（测试路径、lint 路径、覆盖率路径）
- [x] pyproject.toml `[tool.setuptools.packages.find] where = ["src"]`

### 阶段 E —— CI 修复与 README（2026-05-25）
- [x] meshcat_viz.py + test_meshcat.py Black 重新格式化
- [x] pytest.importorskip("meshcat") 用于 CI（meshcat 未安装）
- [x] CI macOS 任务从 macos-13（24 小时超时）切换至自托管运行器
- [x] README 更新：徽章 URL、src 布局结构、功能列表

---

## 规划中（阶段 16+）

### PyPI 就绪（来自 2026-05-27 审查）

计划：`docs/plans/pypi-readiness.md`

| # | 项目 | 优先级 |
|---|------|----------|
| P1 | 在 pyproject.toml 中添加 `[meshcat]` 可选扩展 | 必须修复 |
| P2 | 添加 `readme = "README.md"` 字段 | 必须修复 |
| P3 | 修复 `requires-python` >=3.8 -> >=3.10 | 必须修复 |
| P4 | 扩展 classifiers（Python 版本、机器人学） | 必须修复 |
| P5 | 精简 setup.py（仅保留扩展模块） | 必须修复 |
| P6 | 在 build-wheels.yml 中添加可信发布者 | 必须修复 |
| P7 | 首次正式发布至 PyPI | 必须完成 |

### 阶段 16 —— 双臂协调框架
- [ ] MasterSlaveController 类
- [ ] ClosedChainConstraint 用于双臂夹持物体
- [ ] 机器人实例间实时通信
- [ ] 基于约束的轨迹优化

### 阶段 17 —— 高级功能
- [ ] 力控制（混合位置-力、阻抗控制）
- [ ] 视觉系统集成（多相机标定）
- [ ] FCL/网格碰撞检测
- [ ] 仿真集成（PyBullet/MuJoCo）

### 阶段 HAL —— 硬件抽象层（扩展）
```
robot-toolkit 核心（逆运动学、动力学、轨迹规划）
  |
HardwareInterface (ABC)
  - get_joint_positions()
  - set_joint_targets()
  - get_joint_velocities()
  - stop()
  |
+----------+----------+----------+----------+
|Simulated| ROS2     | Modbus   | Custom   |
|(已完成)  | (可选)   | (可选)   | (用户)   |
+----------+----------+----------+----------+
```
- [x] `hardware/` 基础 ABC + SimulatedHardware + Registry（阶段 B）
- [ ] `hardware/ros2.py` —— ROS2 实现（可选）
- [ ] `hardware/modbus.py` —— Modbus 实现（可选）
- [ ] pyproject.toml 可选依赖：`[ros2]`、`[modbus]`

### 阶段 Viz-Future —— 可视化增强
- [ ] 从 URDF 加载真实 STL/OBJ 网格
- [ ] WebSocket 远程监控
- [ ] 多机器人场景支持

---

## 设计决策

| 决策 | 选择 | 理由 |
|----------|--------|-----------|
| 范围 | 保留全部 8 个模块 | 工程工具箱理念 |
| 许可证 | MIT | 最大化采用率、宽松集成 |
| CI macOS | 自托管运行器 | 无排队延迟，服务所有组织仓库 |
| 可视化 | Meshcat | 基于网页 + Jupyter + 高性能 |
| 硬件协议 | 多协议 HAL | 机器人生态多样性、可插拔 |
| 依赖管理 | 可选扩展 | 用户仅安装所需组件 |
| 运行器范围 | 组织级 | 一台 Mac mini 服务所有项目 |
| 布局 | src 布局含子包 | PEP 421、整洁命名空间、可编辑安装 |
| CI macOS 构建 | 绕过 pypa/cibuildwheel action | setup-python 与自托管运行器不兼容 |
| Meshcat 测试 | pytest.importorskip | CI 中未安装 meshcat 时优雅跳过 |

---

## 当前模块状态

| 模块 | 路径 | 状态 |
|--------|------|--------|
| 逆运动学求解器 | `src/robot_ik/ik/` | 已完成 |
| 动力学 | `src/robot_ik/dynamics/` | 已完成 |
| 轨迹规划 | `src/robot_ik/trajectory/` | 已完成 |
| 碰撞检测 | `src/robot_ik/collision/` | 已完成 |
| 路径规划 | `src/robot_ik/path_planning/` | 已完成 |
| URDF 解析器 | `src/robot_ik/urdf/` | 已完成 |
| 可视化 (matplotlib) | `src/robot_ik/visualization/` | 已完成 |
| 可视化 (meshcat) | `src/robot_ik/visualization/` | 已完成（阶段 B） |
| 硬件 HAL | `src/robot_ik/hardware/` | 已完成（基础 + 仿真） |
| ROS2 | `ros2/` | 已完成（阶段 11） |
| C++ 扩展 | `csrc/` | 已完成（ik_fast、dynamics_fast） |

---

## 项目统计

| 指标 | 数值 |
|--------|-------|
| Python 源码行数 | 2,991（19 个文件） |
| C++ 源码行数 | 666（2 个文件） |
| 测试代码行数 | 1,483（7 个文件） |
| 测试用例数 | 67 |
| 子包数 | 8 |
| 版本 | 0.3.0 |
| 许可证 | MIT |
| 组织 | DZ-drawing |

---

## 风险登记

| 风险 | 影响 | 缓解措施 |
|------|--------|------------|
| Mac mini 维护 | 中等 | 自动化更新 + 监控脚本 |
| Meshcat 性能瓶颈 | 低 | 30-60 FPS 满足需求 |
| HAL API 破坏性变更 | 高 | 严格语义化版本、弃用警告 |
| 多协议维护 | 中等 | 插件架构、社区贡献 |

---

## 成就

- 完整的 6-DOF 机械臂控制流水线
- C++ 扩展实现 137 倍逆运动学加速
- C++ 扩展实现 358 倍动力学加速
- 完整 CI/CD（GitHub Actions + 自托管 macOS）
- 无碰撞路径规划（RRT*）
- ROS2 集成就绪
- Meshcat 基于网页的 3D 可视化
- 硬件抽象层（注册表模式）
- src 布局含 8 个可组合子包
- 67 个测试，ruff + black 检查通过
- 所有新模块采用 TDD 方法
- docs/tutorial/ 中 8 个独立教程
