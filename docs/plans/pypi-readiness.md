# PyPI Publishing Readiness — Implementation Plan

> **For Hermes:** Use subagent-driven-development skill to implement this plan task-by-task.

**Goal:** Fix all must-fix issues so robot-toolkit can be published to PyPI with `pip install robot-ik`.

**Architecture:** Pure pyproject.toml packaging (remove redundant setup.py). Add missing optional extras, fix version bounds, add readme metadata, configure trusted publisher in CI.

**Tech Stack:** pyproject.toml, GitHub Actions OIDC trusted publishing, PyPI

---

### Task 1: Add `meshcat` optional extra to pyproject.toml

**Objective:** Code references `pip install robot-ik[meshcat]` but the extra doesn't exist. Users who follow that instruction get a cryptic pip error.

**Files:**
- Modify: `pyproject.toml:15-17`

**Step 1: Add meshcat extra**

```toml
[project.optional-dependencies]
viz = ["matplotlib>=3.7.0"]
meshcat = ["meshcat>=0.3.0"]
dev = ["pytest>=7.0", "pytest-cov", "black", "ruff", "mypy"]
all = ["robot-ik[viz,meshcat]"]
```

**Step 2: Verify**

Run: `python3 -c "import tomllib; t = tomllib.load(open('pyproject.toml','rb')); print(list(t['project']['optional-dependencies'].keys()))"`
Expected: `['viz', 'meshcat', 'dev', 'all']`

**Step 3: Commit**

```bash
git add pyproject.toml
git commit -m "fix(pypi): add meshcat optional extra for pip install robot-ik[meshcat]"
```

---

### Task 2: Add `readme` field to pyproject.toml

**Objective:** Without this field, PyPI renders the project page with no README — looks broken to users.

**Files:**
- Modify: `pyproject.toml:7-8` (after `description`)

**Step 1: Add readme field**

Add after the `description` line:

```toml
readme = "README.md"
```

**Step 2: Verify**

Run: `grep readme pyproject.toml`
Expected: `readme = "README.md"`

**Step 3: Commit**

```bash
git add pyproject.toml
git commit -m "fix(pypi): add readme field so PyPI renders the README"
```

---

### Task 3: Fix `requires-python` to match CI matrix

**Objective:** `requires-python = ">=3.8"` is untested — CI only runs 3.10+. numpy>=1.24 dropped 3.7 support. Claiming 3.8 is a lie.

**Files:**
- Modify: `pyproject.toml` (requires-python line)
- Modify: `setup.py` (python_requires line)

**Step 1: Update pyproject.toml**

Change:
```toml
requires-python = ">=3.8"
```
To:
```toml
requires-python = ">=3.10"
```

**Step 2: Update setup.py**

Change:
```python
python_requires=">=3.8",
```
To:
```python
python_requires=">=3.10",
```

**Step 3: Verify**

Run: `grep requires-python pyproject.toml && grep python_requires setup.py`
Expected: both show `>=3.10`

**Step 4: Commit**

```bash
git add pyproject.toml setup.py
git commit -m "fix(pypi): bump requires-python to >=3.10 to match CI matrix"
```

---

### Task 4: Add Python version classifiers and expand metadata

**Objective:** PyPI uses classifiers for search/filter. Currently only 3 generic ones. Add specific versions and audience.

**Files:**
- Modify: `pyproject.toml` (classifiers section)
- Modify: `setup.py` (classifiers list)

**Step 1: Expand classifiers in pyproject.toml**

Replace existing classifiers with:

```toml
classifiers = [
    "Development Status :: 4 - Beta",
    "Intended Audience :: Science/Research",
    "Intended Audience :: Developers",
    "License :: OSI Approved :: MIT License",
    "Programming Language :: Python :: 3",
    "Programming Language :: Python :: 3.10",
    "Programming Language :: Python :: 3.11",
    "Programming Language :: Python :: 3.12",
    "Programming Language :: C++",
    "Topic :: Scientific/Engineering",
    "Topic :: Scientific/Engineering :: Artificial Intelligence",
    "Topic :: Scientific/Engineering :: Robotics",
    "Operating System :: OS Independent",
]
```

**Step 2: Update setup.py classifiers to match**

Same list in setup.py's `classifiers=[...]`.

**Step 3: Verify**

Run: `python3 -c "import tomllib; print(len(tomllib.load(open('pyproject.toml','rb'))['project']['classifiers']))"`
Expected: 14

**Step 4: Commit**

```bash
git add pyproject.toml setup.py
git commit -m "fix(pypi): expand classifiers with Python versions, audience, and robotics topic"
```

---

### Task 5: Consolidate to pure pyproject.toml (remove setup.py)

**Objective:** Having both setup.py and pyproject.toml creates version drift risk. Modern packaging uses pyproject.toml only. C++ extensions can use `ext_modules` via `setuptools.build_meta` with a small `setup.cfg` or via `pyproject.toml` + `[tool.setuptools]`.

**Files:**
- Delete: `setup.py`
- Modify: `pyproject.toml` (add ext_modules config)

**Step 1: Add C++ extension config to pyproject.toml**

The C++ extensions use pybind11. Since pybind11 requires setuptools for compilation, keep setuptools as build backend but configure extensions. Add after `[tool.setuptools.packages.find]`:

```toml
[tool.setuptools]
ext-modules = [
    {name = "robot_ik.ik_fast", sources = ["csrc/ik_fast.cpp"], language = "c++"},
    {name = "robot_ik.robot_dyn_fast", sources = ["csrc/robot_dyn_fast.cpp"], language = "c++"},
]

[tool.setuptools.package-dir]
robot_ik = "src/robot_ik"
```

Note: pyproject.toml's `ext-modules` doesn't support `include_dirs` for pybind11. The pybind11 include dir is resolved automatically when `pybind11` is in `build-system.requires`. However, setuptools >=69 supports `ext-modules` but some fields are limited. The safest approach is to keep a minimal `setup.py` that ONLY defines ext_modules (no metadata). This is the recommended hybrid approach.

**Step 1 (revised): Trim setup.py to ext_modules only**

Replace entire `setup.py` with:

```python
from setuptools import Extension, setup
import pybind11

setup(
    ext_modules=[
        Extension(
            "robot_ik.ik_fast",
            sources=["csrc/ik_fast.cpp"],
            include_dirs=[pybind11.get_include()],
            language="c++",
            extra_compile_args=["-O3"],
        ),
        Extension(
            "robot_ik.robot_dyn_fast",
            sources=["csrc/robot_dyn_fast.cpp"],
            include_dirs=[pybind11.get_include()],
            language="c++",
            extra_compile_args=["-O3"],
        ),
    ],
)
```

All metadata (name, version, deps, etc.) lives in pyproject.toml only.

**Step 2: Verify import still works**

Run: `pip install -e . && python3 -c "from robot_ik import six_dof_articulated; print('OK')"`

**Step 3: Verify version comes from pyproject.toml**

Run: `python3 -c "from robot_ik import __version__; print(__version__)"`
Expected: `0.3.0`

**Step 4: Run tests**

Run: `PYTHONPATH=src pytest tests/ -v`
Expected: 67 passed

**Step 5: Commit**

```bash
git add pyproject.toml setup.py
git commit -m "refactor(pypi): trim setup.py to ext-modules only, metadata in pyproject.toml"
```

---

### Task 6: Add PyPI trusted publisher to build-wheels.yml

**Objective:** CI builds wheels but never publishes. Add OIDC trusted publisher so pushes to main automatically publish to PyPI.

**Files:**
- Modify: `.github/workflows/build-wheels.yml`
- Modify: `pyproject.toml` (add `publisher-id` in comments for reference)

**Step 1: Configure PyPI trusted publisher on PyPI side**

This is a manual step — must be done on https://pypi.org:
1. Go to https://pypi.org/manage/account/publishing/
2. Add a new pending publisher:
   - PyPI Project Name: `robot-ik`
   - Owner: `DZ-drawing`
   - Repository name: `robot-toolkit`
   - Workflow name: `build-wheels.yml`
   - Environment name: `pypi`

**Step 2: Add publish job to build-wheels.yml**

Add at the end of the jobs section (after build step):

```yaml
  publish-pypi:
    name: Publish to PyPI
    if: github.event_name == 'push' && github.ref == 'refs/heads/main'
    needs: [build-wheels]
    runs-on: ubuntu-latest
    environment: pypi
    permissions:
      id-token: write
    steps:
      - name: Download all artifacts
        uses: actions/download-artifact@v4
        with:
          path: dist/
          merge-multiple: true

      - name: Publish to PyPI
        uses: pypa/gh-action-pypi-publish@release/v1
```

Note: This uses OIDC trusted publishing — no API token needed. The `environment: pypi` is required for the protection rule.

**Step 3: Add pypi environment protection**

In GitHub repo settings → Settings → Environments:
1. Create environment `pypi`
2. Add protection rule: only `main` branch
3. Add required reviewers (optional but recommended for first publish)

**Step 4: Verify**

Trigger CI with a push to main. The `publish-pypi` job should appear and publish wheels.

**Step 5: Commit**

```bash
git add .github/workflows/build-wheels.yml
git commit -m "ci: add PyPI trusted publisher via OIDC to build-wheels.yml"
```

---

### Task 7: Final verification — build sdist + test install

**Objective:** After all fixes, verify the package builds and installs correctly from a clean sdist.

**Files:** None (verification only)

**Step 1: Build sdist**

```bash
python3 -m pip install build --break-system-packages
python3 -m build --sdist
```

**Step 2: Test install from sdist**

```bash
python3 -m venv /tmp/test-venv
/tmp/test-venv/bin/pip install dist/robot_ik-0.3.0.tar.gz
/tmp/test-venv/bin/python -c "from robot_ik import RobotModel, DynamicsSolver; print('Install OK')"
rm -rf /tmp/test-venv
```

**Step 3: Verify metadata**

```bash
python3 -m twine check dist/*
```

Expected: `PASSED` for all checks (long_description, readme rendered, etc.)

**Step 4: Run full test suite**

```bash
PYTHONPATH=src pytest tests/ -v
```

Expected: 67 passed

**Step 5: Commit (if any fixes needed)**
