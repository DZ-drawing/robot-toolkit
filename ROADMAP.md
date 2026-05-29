# robot-toolkit Roadmap

Version: 0.3.0 | Last updated: 2026-05-27

## Project Summary

6-DOF serial manipulator engineering toolbox: IK, rigid body dynamics, trajectory
planning, collision detection, path planning, URDF import, visualization, hardware
abstraction, and C++ acceleration.
Design philosophy: independent, composable modules — like numpy/scipy for robotics.

---

## Completed

### Phase 1 — Inverse Kinematics (2026-05-08)
- [x] DH parameter forward kinematics
- [x] Damped least-squares IK (Levenberg-Marquardt)
- [x] Analytical Jacobian computation
- [x] Joint limit enforcement (gradient projection)
- [x] Pre-built models: 6-DOF articulated, spherical wrist
- [x] 3D matplotlib visualization (arm + target frame)
- [x] Test suite: FK identity, IK roundtrip, Jacobian, joint limits, benchmark
- [x] Performance: ~3 ms avg, <0.1 mm position accuracy

### Phase 2 — C++ IK Extension (2026-05-09)
- [x] pybind11 C++ extension for FK + Jacobian + IK loop
- [x] 137x speedup over pure Python (~0.09 ms avg)
- [x] Graceful fallback when C++ not built

### Phase 3 — Rigid Body Dynamics (2026-05-09)
- [x] Recursive Newton-Euler inverse dynamics (RNEA)
- [x] Composite Rigid Body Algorithm (CRBA) forward dynamics
- [x] Gravity torque, Coriolis, inertia matrix computation
- [x] Inverse/forward dynamics roundtrip tests (50 configs)
- [x] Pendulum gravity validation against analytical solution

### Phase 4 — C++ Dynamics Extension (2026-05-09)
- [x] pybind11 C++ RNEA implementation
- [x] 358x speedup over pure Python dynamics
- [x] 50-configuration verification against Python reference

### Phase 5 — URDF + Packaging (2026-05-09)
- [x] URDF parser: mass, COM, inertia extraction
- [x] URDF to DH parameter conversion
- [x] `robot_ik` namespace package structure
- [x] `setup.py` for pip install + C++ build_ext

### Phase 6 — Trajectory Planning (2026-05-12)
- [x] Joint-space interpolation (linear, cubic, quintic)
- [x] Cartesian-space straight-line interpolation with SLERP
- [x] Trapezoidal velocity profile with acceleration limits
- [x] S-curve profile (7-segment jerk-limited)
- [x] Waypoint trajectories with parabolic blends
- [x] 12 TDD tests (boundary conditions, continuity)

### Phase 7 — CI/CD Pipeline (2026-05-12)
- [x] GitHub Actions CI (Ubuntu/macOS/Windows, Python 3.10-3.12)
- [x] Pre-commit hooks (black, ruff, mypy)
- [x] Requirements files (dev dependencies)
- [x] Code coverage reporting

### Phase 8 — Collision Detection (2026-05-12)
- [x] Geometry primitives: Sphere, Capsule, Box
- [x] Distance functions (sphere-sphere, sphere-capsule, etc.)
- [x] Self-collision detection with adjacent link filtering
- [x] Environment obstacle collision
- [x] Contact point approximation
- [x] 10 comprehensive tests

### Phase 9 — Dynamics Benchmark (2026-05-12)
- [x] Performance suite: IK, dynamics, trajectory
- [x] Benchmark documentation (results, optimization tips)
- [x] C++ speedup comparison framework

### Phase 10 — Path Planning (2026-05-12)
- [x] RRT* algorithm implementation
- [x] Collision-free path planning
- [x] Path smoothing (shortcut)
- [x] 3 test cases (basic, collision, convenience)

### Phase 11 — ROS2 Integration (2026-05-12)
- [x] ROS2 package structure (package.xml, setup.py)
- [x] IK service server node example
- [x] Launch files and documentation

### Phase 12 — Examples & Tutorials (2026-05-12)
- [x] Jupyter notebook: IK tutorial
- [x] Example scripts for common tasks
- [x] 4 tutorials from challenges (workspace analysis, collision detection,
      coordinated trajectory, path planning)
- [x] API documentation updates

### Phase 13 — License & Legal (2026-05-12)
- [x] MIT LICENSE file added
- [x] License consistency across project files
- [x] setup.py license field verified

### Phase 14 — PyPI Distribution Setup (2026-05-12)
- [x] cibuildwheel GitHub Actions workflow (Linux/macOS/Windows)
- [x] pyproject.toml with full PyPI metadata
- [x] MANIFEST.in for package assets
- [x] Release documentation (docs/RELEASE.md)
- [x] Multi-platform wheel build configuration (Python 3.10-3.12)

### Phase 15 — PyPI Token & First Release (2026-05-13)
- [x] PyPI API token configured in GitHub secrets
- [x] Version bumped to 0.3.0
- [x] CI workflow fixes (YAML syntax, CMAKE_ARGS, portable wheels)

### Phase A — Self-Hosted macOS Runner (2026-05-20)
- [x] Mac mini M1 environment configured (user `danny` at 192.168.3.143)
- [x] Organization-level runner registered for DZ-drawing
- [x] GitHub Actions workflows use self-hosted macOS (ci.yml + build-wheels.yml)
- [x] cibuildwheel bypasses pypa action, uses `pip install cibuildwheel` + `CIBW_PYTHON_SOURCE=system`
- [x] Passwordless sudo configured for CI jobs
- [x] CI stabilized (setuptools install, ruff lint sweep, python3 -m cibuildwheel)

### Phase B — Meshcat Visualization & Hardware HAL (2026-05-21)
- [x] MeshcatVisualizer class (procedural 3D robot, set_robot, update_joints)
- [x] Jupyter integration (start_jupyter returns IFrame)
- [x] Real-time streaming via threading (30 Hz)
- [x] HardwareInterface ABC + SimulatedHardware + HardwareRegistry
- [x] Tests for meshcat visualization (skip when meshcat not installed)
- [x] Tests for hardware HAL

### Phase C — Code Quality & Structure (2026-05-21)
- [x] Ruff lint sweep: 108 errors fixed (89 auto, 9 unsafe, 10 manual)
- [x] Black formatting enforced
- [x] Ruff config migrated to `[tool.ruff.lint]` (deprecation fix)
- [x] Reviewer fixes: dynamic DOF support, context manager, thread safety
- [x] Tutorials reorganized into `docs/tutorial/` (8 standalone guides T0-T7)
- [x] Examples restructured as project demos (unfinished drafts)

### Phase D — Project Restructure to src layout (2026-05-21)
- [x] Migrated to PEP 421 src layout: `src/robot_ik/` with 8 subpackages
- [x] Subpackages: ik/, dynamics/, trajectory/, collision/, path_planning/,
      urdf/, visualization/, hardware/
- [x] C++ sources moved to `csrc/`
- [x] Tests reorganized into `tests/` by module
- [x] All cross-module imports updated
- [x] Backward-compat `__init__.py` re-exports preserved
- [x] CI workflows updated (test paths, lint paths, cov paths)
- [x] pyproject.toml `[tool.setuptools.packages.find] where = ["src"]`

### Phase E — CI Fixes & README (2026-05-25)
- [x] Black reformat on meshcat_viz.py + test_meshcat.py
- [x] pytest.importorskip("meshcat") for CI (meshcat not installed)
- [x] CI macOS job switched from macos-13 (24h timeout) to self-hosted runner
- [x] README updated: badge URLs, src layout structure, features list

---

## Planned (Phase 16+)

### PyPI Readiness (from review 2026-05-27)

Plan: `docs/plans/pypi-readiness.md`

| # | Item | Priority |
|---|------|----------|
| P1 | Add `[meshcat]` optional extra to pyproject.toml | Must fix |
| P2 | Add `readme = "README.md"` field | Must fix |
| P3 | Fix `requires-python` >=3.8 -> >=3.10 | Must fix |
| P4 | Expand classifiers (Python versions, robotics) | Must fix |
| P5 | Consolidate setup.py (trim to ext-modules only) | Must fix |
| P6 | Add trusted publisher to build-wheels.yml | Must fix |
| P7 | Actual first publish to PyPI | Must do |

### Phase 16 — Dual-Arm Coordination Framework
- [ ] MasterSlaveController class
- [ ] ClosedChainConstraint for dual-arm object holding
- [ ] Real-time communication between robot instances
- [ ] Constraint-based trajectory optimization

### Phase 17 — Advanced Features
- [ ] Force control (hybrid position-force, impedance)
- [ ] Vision system integration (multi-camera calibration)
- [ ] FCL/mesh-based collision detection
- [ ] Simulation integration (PyBullet/MuJoCo)

### Phase HAL — Hardware Abstraction Layer (expand)
```
robot-toolkit core (IK, Dynamics, Trajectory)
  |
HardwareInterface (ABC)
  - get_joint_positions()
  - set_joint_targets()
  - get_joint_velocities()
  - stop()
  |
+----------+----------+----------+----------+
|Simulated| ROS2     | Modbus   | Custom   |
|(done)   | (opt)   | (opt)    | (user)   |
+----------+----------+----------+----------+
```
- [x] `hardware/` base ABC + SimulatedHardware + Registry (Phase B)
- [ ] `hardware/ros2.py` — ROS2 implementation (optional)
- [ ] `hardware/modbus.py` — Modbus implementation (optional)
- [ ] pyproject.toml optional-dependencies: `[ros2]`, `[modbus]`

### Phase Viz-Future — Visualization Enhancements
- [ ] Load real STL/OBJ meshes from URDF
- [ ] WebSocket for remote monitoring
- [ ] Multi-robot scene support

---

## Design Decisions

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Scope | Keep all 8 modules | Engineering toolbox philosophy |
| License | MIT | Maximum adoption, permissive integration |
| CI macOS | Self-hosted runner | No queue delays, serves all org repos |
| Visualization | Meshcat | Web-based + Jupyter + high performance |
| Hardware protocols | Multi-protocol HAL | Robot ecosystem diversity, pluggable |
| Dependencies | Optional extras | Users install only what they need |
| Runner scope | Organization-level | One Mac mini serves all projects |
| Layout | src layout with subpackages | PEP 421, clean namespace, editable installs |
| CI macOS builds | Bypass pypa/cibuildwheel action | setup-python incompatible with self-hosted runner |
| Meshcat tests | pytest.importorskip | Skip gracefully when meshcat not installed in CI |

---

## Current Module Status

| Module | Path | Status |
|--------|------|--------|
| IK Solver | `src/robot_ik/ik/` | Done |
| Dynamics | `src/robot_ik/dynamics/` | Done |
| Trajectory | `src/robot_ik/trajectory/` | Done |
| Collision | `src/robot_ik/collision/` | Done |
| Path Planning | `src/robot_ik/path_planning/` | Done |
| URDF Parser | `src/robot_ik/urdf/` | Done |
| Visualization (matplotlib) | `src/robot_ik/visualization/` | Done |
| Visualization (meshcat) | `src/robot_ik/visualization/` | Done (Phase B) |
| Hardware HAL | `src/robot_ik/hardware/` | Done (base + simulated) |
| ROS2 | `ros2/` | Done (Phase 11) |
| C++ Extensions | `csrc/` | Done (ik_fast, dynamics_fast) |

---

## Project Stats

| Metric | Value |
|--------|-------|
| Python source LOC | 2,991 (19 files) |
| C++ source LOC | 666 (2 files) |
| Test LOC | 1,483 (7 files) |
| Test cases | 67 |
| Subpackages | 8 |
| Version | 0.3.0 |
| License | MIT |
| Org | DZ-drawing |

---

## Risk Register

| Risk | Impact | Mitigation |
|------|--------|------------|
| Mac mini maintenance | Medium | Automated update + monitoring scripts |
| Meshcat perf bottleneck | Low | 30-60 FPS sufficient for needs |
| HAL API breaking changes | High | Strict semver, deprecation warnings |
| Multi-protocol maintenance | Medium | Plugin architecture, community contributions |

---

## Achievements

- Complete 6-DOF manipulator control pipeline
- 137x IK speedup with C++ extension
- 358x dynamics speedup with C++ extension
- Full CI/CD (GitHub Actions + self-hosted macOS)
- Collision-free path planning (RRT*)
- ROS2 integration ready
- Meshcat web-based 3D visualization
- Hardware abstraction layer with registry pattern
- src layout with 8 composable subpackages
- 67 tests, ruff + black clean
- TDD approach for all new modules
- 8 standalone tutorials in docs/tutorial/
