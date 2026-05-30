[English](benchmark.md) | [中文](zh/benchmark.md)

# Performance Benchmarks

Benchmark results for robot-toolkit modules (IK, Dynamics, Trajectory Planning).

## Running Benchmarks

```bash
python3 benchmark.py
```

## Results

### Inverse Kinematics (IK)

| Metric | Value |
|--------|-------|
| Avg Time | ~4.9 ms |
| P50 Time | ~2.4 ms |
| P95 Time | ~17.4 ms |
| Avg Iterations | ~29 |
| Failure Rate | 14% (random poses) |

**Notes:**
- 200 random target poses
- Damped least-squares (DLS) method
- Max 100 iterations per solve
- Failures due to unreachable configurations or singularities

### Rigid Body Dynamics

| Operation | Avg Time | P50 Time | P95 Time |
|-----------|----------|----------|----------|
| Inverse Dynamics (RNEA) | ~819 μs | ~793 μs | ~1.04 ms |
| Forward Dynamics (CRBA) | ~6.25 ms | ~6.17 ms | ~7.02 ms |
| Mass Matrix (CRBA) | ~5.42 ms | ~5.36 ms | ~6.14 ms |

**Notes:**
- 1000 random joint configurations
- RNEA (Recursive Newton-Euler Algorithm) for inverse dynamics
- CRBA (Composite Rigid Body Algorithm) for forward dynamics
- 6-DOF manipulator

### Trajectory Planning

| Method | Avg Time | P50 Time | P95 Time |
|--------|----------|----------|----------|
| Quintic Interpolation | ~2.27 ms | ~2.26 ms | ~3.28 ms |
| Trapezoidal Velocity Profile | ~0.88 ms | ~0.88 ms | ~1.07 ms |
| Waypoint Trajectory (5 points) | ~3.85 ms | ~3.72 ms | ~4.21 ms |

**Notes:**
- 100 random trajectories
- 6-DOF joint space
- dt = 0.01 (100 Hz sampling)

## C++ Extension Speedup

The repository includes C++ extensions for significant performance improvements:

| Module | Python Time | C++ Time | Speedup |
|--------|-------------|----------|---------|
| IK Solver | ~12 ms | ~0.09 ms | 137x |
| Dynamics (RNEA/CRBA) | ~180 μs | ~0.5 μs | 358x |

**To enable C++ extensions:**

```bash
python3 setup.py build_ext --inplace
```

**Note:** The benchmark.py script checks for C++ extensions automatically. If built, it will use them automatically.

## Benchmarking Tips

### Consistent Results

1. **Use consistent hardware:** Run on the same machine for comparable results
2. **CPU frequency scaling:** Disable turbo boost for stable measurements
3. **Multiple runs:** Average over 3-5 runs to account for variance
4. **Thermal throttling:** Watch CPU temperature during long benchmarks

### Custom Benchmarks

```python
from robot_ik import six_dof_articulated
import time

robot = six_dof_articulated()
n_samples = 100

# Benchmark IK
times = []
for _ in range(n_samples):
    target = robot.forward_kinematics(np.random.uniform(-1, 1, 6))
    start = time.perf_counter()
    success, q, _, _ = robot.ik_solve(target)
    times.append(time.perf_counter() - start)

print(f"Avg: {np.mean(times) * 1000:.2f} ms")
```

## Performance Optimizations

### IK Solver
- **Damping factor (λ):** Adjust for convergence speed vs stability
- **Max iterations:** Balance between solve rate and success rate
- **Initial guess:** Use previous solution for sequential tasks

### Dynamics
- **C++ extension:** 358x speedup for real-time control
- **Batch operations:** Vectorize operations for multiple configurations

### Trajectory Planning
- **Pre-allocate arrays:** For large trajectories
- **Reduce sampling rate:** Lower dt (e.g., 0.02 instead of 0.01) for faster generation

## Hardware

**Reference system:**
- CPU: Intel/AMD x86_64
- Python: 3.10+
- NumPy: 1.24+

Results may vary based on CPU architecture and clock speed.

## Contributing Benchmarks

When adding new modules, include benchmark functions following this pattern:

```python
def benchmark_your_module(n_samples: int = 100):
    """Benchmark your module performance."""
    print(f"\n=== Your Module Benchmark ({n_samples} samples) ===")
    np.random.seed(42)

    times = []
    for _ in range(n_samples):
        # Setup
        start = time.perf_counter()
        # Operation
        elapsed = time.perf_counter() - start
        times.append(elapsed)

    times_ms = np.array(times) * 1000
    print(f"  Avg time:  {np.mean(times_ms):.3f} ms")
    print(f"  P50 time:  {np.median(times_ms):.3f} ms")
    print(f"  P95 time:  {np.percentile(times_ms, 95):.3f} ms")
```

Add to `run_all_benchmarks()` in `benchmark.py`.
