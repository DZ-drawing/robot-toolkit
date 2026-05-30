[English](RELEASE.md) | [中文](zh/RELEASE.md)

# PyPI Release Guide

## Local Testing

```bash
# Install build tools
python3 -m pip install --upgrade pip build twine --user

# Build source distribution and wheels
python3 -m build

# Check the built package
twine check dist/*

# Test install from local wheel
pip install dist/robot_ik-0.2.0-*.whl
```

## Automated Build

The `.github/workflows/build-wheels.yml` workflow:

- **Triggers**: Push to main, tags starting with `v*`, manual dispatch
- **Builds**: Wheels for Linux/macOS/Windows (Python 3.10-3.12)
- **Tests**: Runs pytest on built wheels
- **Publishes**: To PyPI on tagged releases

## Release Process

1. Update version in `pyproject.toml` and `setup.py`
2. Update `ROADMAP.md` with release notes
3. Commit changes
4. Create and push tag:
   ```bash
   git tag v0.2.0
   git push origin v0.2.0
   ```
5. GitHub Actions builds and publishes automatically

## PyPI Configuration

- **Secret**: `PYPI_API_TOKEN` must be set in GitHub repo settings
- **Token**: Create at https://pypi.org/manage/account/token/
- **Test PyPI**: Use `https://test.pypi.org/legacy/` for testing

## Troubleshooting

**Build fails on macOS**: ARM64 wheels require macOS 11+ (Big Sur)

**Build fails on Windows**: CMake dependencies handled automatically

**Publish fails**: Check token has correct permissions (must be " Trusted Publisher" or API token)

**Wheels not uploaded**: Tag must start with `v` (e.g., `v0.2.0`)
