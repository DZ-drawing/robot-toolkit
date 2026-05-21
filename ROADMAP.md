# robot-toolkit Roadmap

Version: 0.3.0 | Last updated: 2026-05-21

## Project Summary

6-DOF serial manipulator engineering toolbox: IK, rigid body dynamics, trajectory
planning, collision detection, path planning, URDF import, and C++ acceleration.
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

### Phase A — Self-Hosted macOS Runner (2026-05-14)
- [x] Mac mini environment configured (DZ-drawing org)
- [x] Organization-level runner registered at 192.168.3.143
- [x] GitHub Actions workflow updated for self-hosted macOS
- [x] Runner tool cache configured (setup-python compatible)
- [x] Passwordless sudo configured for CI jobs
- [x] CI stabilized (cibuildwheel path fix, setuptools workaround, ruff lint sweep)

---

## v0.3.0 Priorities

Priority order decided 2026-05-14: **A (scope) -> D (tech debt) -> B (expand) -> C (release)**

### D1 — Self-Hosted macOS Runner
**Status: Done**
- Mac mini runner operational, serving DZ-drawing org
- Build times predictable (~20 min), no GitHub Actions queue delays

### D2 — Meshcat Visualization
**Status: In Progress (worktree: feature-meshcat)**
- Design approved: `docs/plans/2026-05-14-phase-b-meshcat-design.md`
- Web-based 3D viz, Jupyter native, 30-60 FPS
- Threading-based real-time streaming for monitoring
- Estimated: 2-3 days

### A — Scope Completion
**Status: 8/8 modules complete**
All 8 modules (IK, dynamics, trajectory, collision, path planning, visualization,
URDF, ROS2) are independently composable. Visualization flagged for meshcat upgrade.

### B — Expand: Hardware Abstraction Layer (HAL)
**Status: Design complete, not started**
- Design: `docs/plans/2026-05-14-scope-roadmap-review.md` (Section 3)
- Protocol-agnostic: ROS2/Modbus/custom as optional extras
- Unified `HardwareInterface` ABC with registry/factory pattern
- Estimated: 3-5 days

### C — Release
**Status: Infrastructure ready**
- CI stable across all platforms
- PyPI token configured
- Pending: actual publish + release notes

---

## Planned (Phase 16+)

### Phase 16 — Dual-Arm Coordination Framework
- [ ] MasterSlaveController class
- [ ] ClosedChainConstraint for dual-arm object holding
- [ ] Real-time communication between robot instances
- [ ] Constraint-based trajectory optimization
- [ ] Tutorial 5: Master-slave coordinated grasping
- [ ] Tutorial 6: Closed-chain constraint control

### Phase 17 — Advanced Features
- [ ] Force control (hybrid position-force, impedance)
- [ ] Vision system integration (multi-camera calibration)
- [ ] FCL/mesh-based collision detection
- [ ] Simulation integration (PyBullet/MuJoCo)
- [ ] Tutorial 7: Dual-arm assembly with force control
- [ ] Tutorial 8: Vision-guided manipulation
- [ ] Tutorial 9: Real-time mesh collision with FCL

### Phase HAL — Hardware Abstraction Layer
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
|(built-in)| (opt)   | (opt)    | (user)   |
+----------+----------+----------+----------+
```
- [ ] `hardware/base.py` — HardwareInterface ABC
- [ ] `hardware/simulated.py` — built-in simulation
- [ ] `hardware/registry.py` — protocol registry + factory
- [ ] `hardware/ros2.py` — ROS2 implementation (optional)
- [ ] `hardware/modbus.py` — Modbus implementation (optional)
- [ ] pyproject.toml optional-dependencies: `[meshcat]`, `[ros2]`, `[modbus]`

### Phase Meshcat — Visualization Upgrade
- [ ] `visualize_meshcat.py` — MeshcatVisualizer class
- [ ] Procedural 3D robot model generation (box/cylinder/sphere/triad)
- [ ] `set_robot()`, `update_joints()` for FK-based display
- [ ] Jupyter integration (`start_jupyter()` returns IFrame)
- [ ] Real-time streaming (`start_realtime_stream()` via threading, 30Hz)
- [ ] Unit + integration tests (TDD)
- [ ] Jupyter notebook tutorial

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

---

## Current Module Status

| Module | File | Status |
|--------|------|--------|
| IK Solver | `ik_solver.py` | Done |
| Dynamics | `robot_dyn.py` | Done |
| Trajectory | `trajectory.py` | Done |
| Collision | (in ik_solver.py) | Done |
| Path Planning | (in ik_solver.py) | Done |
| Visualization | `visualize.py` | Done (matplotlib) |
| URDF Parser | `urdf_parser.py` | Done |
| ROS2 | `ros2/` | Done |
| Meshcat Viz | `visualize_meshcat.py` | In Progress |
| HAL | `hardware/` | Design Only |

---

## Project Stats

| Metric | Value |
|--------|-------|
| Python LOC | ~4,500 |
| C++ LOC | ~500 |
| Test cases | 60+ |
| Modules | 8 (+ 2 in progress) |
| Version | 0.3.0 |
| License | MIT |

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
- TDD approach for all new modules
- Comprehensive documentation (10+ docs)
