[English](../RELEASE.md) | [中文](RELEASE.md)

# PyPI 发布指南

## 本地测试

```bash
# 安装构建工具
python3 -m pip install --upgrade pip build twine --user

# 构建源码分发和 Wheel 包
python3 -m build

# 检查构建产物
twine check dist/*

# 从本地 Wheel 安装测试
pip install dist/robot_ik-0.2.0-*.whl
```

## 自动化构建

`.github/workflows/build-wheels.yml` 工作流：

- **触发条件**: 推送到 main 分支、以 `v*` 开头的标签、手动触发
- **构建目标**: Linux/macOS/Windows 的 Wheel 包 (Python 3.10-3.12)
- **测试**: 在构建的 Wheel 包上运行 pytest
- **发布**: 标签发布时自动发布到 PyPI

## 发布流程

1. 更新 `pyproject.toml` 和 `setup.py` 中的版本号
2. 更新 `ROADMAP.md` 的发布说明
3. 提交更改
4. 创建并推送标签：
   ```bash
   git tag v0.2.0
   git push origin v0.2.0
   ```
5. GitHub Actions 自动构建并发布

## PyPI 配置

- **密钥**: 必须在 GitHub 仓库设置中配置 `PYPI_API_TOKEN`
- **令牌**: 在 https://pypi.org/manage/account/token/ 创建
- **测试 PyPI**: 使用 `https://test.pypi.org/legacy/` 进行测试

## 故障排除

**macOS 构建失败**: ARM64 Wheel 包需要 macOS 11+ (Big Sur)

**Windows 构建失败**: CMake 依赖项会自动处理

**发布失败**: 检查令牌是否具有正确的权限（必须是"可信发布者"或 API 令牌）

**Wheel 未上传**: 标签必须以 `v` 开头（如 `v0.2.0`）
