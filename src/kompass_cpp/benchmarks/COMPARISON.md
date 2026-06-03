# Kompass Core vs. Open-Source Robotics — Comparative Benchmarks

> **Status: work in progress.** This document describes an ongoing effort to
> benchmark the `kompass-core` GPGPU kernels against equivalent components from
> other open-source robotics stacks. The integration lives in this fork
> (`kompass-core-nav2`) and is **not** part of upstream `kompass-core`.

## Purpose

The standard benchmark suite ([`README.md`](./README.md)) measures `kompass-core`
against *itself* across hardware backends (CPU-native, CPU-SYCL, CUDA, ROCm). It
answers *"how much does the GPU help?"* but not *"how do we compare to the
ecosystem?"*.

This comparison work answers the second question: for each `kompass-core`
functional component, run an equivalent component from a widely-used open-source
robotics package on the **same input data**, through the **same timing harness**,
and report both numbers side by side.

## ⚠️ On "apples to apples"

A perfectly fair comparison is rarely possible — the matched components were
designed with different scopes, data structures, and API contracts. Each entry
in the table below is an *approximate functional match*, not a drop-in
equivalent. Known mismatches are documented per test in
[Caveats & open issues](#caveats--open-issues). **Read those before quoting any
number.**

## What is being compared

The reference stack chosen for the first round is **[Nav2](https://navigation.ros.org/)**,
the ROS 2 navigation framework — it is the most widely deployed open-source
navigation stack and ships C++ components that overlap with the `kompass-core`
kernels.

| # | Test | `kompass-core` component | Open-source counterpart |
|---|------|--------------------------|-------------------------|
| 1 | Cost Evaluator (motion planning) | `Control::CostEvaluator::getMinTrajectoryCost` | **Nav2 MPPI** `mppi::CriticManager::evalTrajectoriesScores` |
| 2 | Local Mapper — LaserScan (occupancy grid) | `Mapping::LocalMapper` raycast → 400×400 grid | **Nav2 `Costmap2D`** (`raytraceLine` + `setCost`) |
| 2b | Local Mapper — PointCloud (occupancy grid, GPU only) | `Mapping::LocalMapperGPU::scanToGrid` (100k cloud → PC-to-laserscan kernel → raycast) | **Nav2 `Costmap2D`** (cloud → laserscan reduction → `raytraceLine` + `setCost`) |
| 3 | Critical Zone — PointCloud (safety) | `CriticalZoneChecker::check` (100k-point cloud) | **Nav2 Collision Monitor** (`PointCloud` source + `Polygon::getPointsInside`) |
| 4 | Critical Zone — LaserScan (safety) | `CriticalZoneChecker::check` (3600-ray scan) | **Nav2 Collision Monitor** (`Scan` source + `Polygon::getPointsInside`) |


## How the comparison is wired in

All comparison code lives in [`benchmark_runner.cpp`](./benchmark_runner.cpp).
The design principle is *minimal divergence from upstream*:

- Each Nav2 workload is added **inside the same test block** as the
  `kompass-core` workload, and timed with the same `measure_performance(...)`
  helper, so both numbers land in the same output JSON.
- Both sides are fed the **same generated input** (the `generate_*` helpers
  produce the scans / clouds / trajectories once per block).
- Nav2's sensor and polygon classes normally ingest data only via ROS topic
  callbacks. To drive them synchronously from the benchmark, thin subclasses
  expose the protected entry points:
  - `PublicCostmap2D` — exposes `raytraceLine`.
  - `DummyPointCloud` / `DummyScan` — expose `dataCallback` via `injectData(...)`.
  - `DummyPolygon` — sets the polygon vertices directly via `injectPolygon(...)`.
- `rclcpp::init` / `rclcpp::shutdown` bracket `main`; each Nav2 block spins up a
  short-lived `nav2_util::LifecycleNode`. Both are compiled out when Nav2 is off.

Two CMake switches select what a build measures, so the kompass GPU kernels and
the Nav2 reference can be benchmarked on **separate machines** and merged at plot
time:

- `BENCHMARK_WITH_NAV2` (default `ON`) — compile + run the Nav2 workloads. Turn
  **OFF** for the GPU device builds (CUDA / ROCm): the benchmark then has no ROS
  dependency and emits only the kompass-core numbers.
- `BENCHMARK_WITH_KOMPASS` (default `ON`) — time + report the kompass-core
  workloads. Turn **OFF** for the Nav2-reference container: the kompass library
  is still linked (it generates the shared inputs) but its workloads are not
  timed, so the JSON carries only the `Nav2_*` numbers.

Both sides use the same `test_name`s, so `plot_benchmarks.py` aggregates them by
test across machines regardless of which side produced each entry.

## Build & run

The comparison spans two machines, joined at plot time. Run the Nav2 reference on
the **same device's CPU** as the kompass GPU run, so each comparison is one SoC
(kompass on the device GPU vs Nav2 on that device's CPU).

| Role | Where | Key flags | JSON |
|------|-------|-----------|------|
| **kompass kernels** | natively on the GPU device (Jetson CUDA / Strix ROCm) | `-DBENCHMARK_WITH_NAV2=OFF` (+ AdaptiveCpp) | `kompass-core` numbers |
| **Nav2 reference** | the container (multi-arch, CPU) | `-DBENCHMARK_WITH_KOMPASS=OFF -DFORCE_CPU_BUILD=ON` | `Nav2_*` numbers |

### Nav2 reference container (recommended)

The [`dockerfile`](../../../dockerfile) at the repo root **is** the Nav2 reference:
ROS 2 Humble (Ubuntu 22.04, LTS — matches JetPack 6 on Jetson), **CPU-only**
(no CUDA / ROCm / AdaptiveCpp), built with `BENCHMARK_WITH_KOMPASS=OFF`. It is
multi-arch (`linux/amd64` for AMD Strix, `linux/arm64` for Jetson), so the same
image runs on each device's CPU.

```bash
# Build natively on the device, or cross-build both arches with buildx:
docker build -f dockerfile -t kompass-nav2-ref .
#   docker buildx build --platform linux/amd64,linux/arm64 -f dockerfile -t kompass-nav2-ref .

# Run -> writes /out/benchmark_nav2_<arch>.json, tagged with the device label:
docker run --rm -v "$PWD/out:/out" kompass-nav2-ref Nav2_Jetson_Orin_CPU
```

### kompass device build (native, GPU)

On the Jetson / Strix, build the kompass side with AdaptiveCpp and **no** Nav2
(see the upstream [`README.md`](./README.md) for the AdaptiveCpp setup):

```bash
mkdir -p build_gpu && cd build_gpu
cmake .. -DCMAKE_BUILD_TYPE=Release -DBENCHMARK_WITH_NAV2=OFF
cmake --build . --target kompass_benchmark -- -j$(nproc)

# CUDA (Jetson):
ACPP_VISIBILITY_MASK=cuda ./src/kompass_cpp/benchmarks/kompass_benchmark "Jetson_Orin_CUDA" "../benchmark_cuda.json"
# ROCm (Strix):
ACPP_VISIBILITY_MASK=hip  ./src/kompass_cpp/benchmarks/kompass_benchmark "Strix_ROCm" "../benchmark_rocm.json"
```

### Plot

Collect the device JSON(s) and the container's `Nav2_*` JSON into one directory
and run `plot_benchmarks.py`; it groups every `test_name` across all machines.

## Caveats & open issues

These are tracked so the comparison is not over-interpreted before it is
finished:

- **Test 1 — scope mismatch.** `getMinTrajectoryCost` *scores* trajectories
  **and** reduces to the minimum-cost one; `evalTrajectoriesScores` only scores.
  This is the remaining apples-to-apples caveat now that both workloads run in
  the same block: the `kompass-core` side does strictly more work (score +
  argmin reduction) than the Nav2 side (score only). Test 1 now emits both
  `CostEvaluator_5k_Trajs` and `Nav2_MPPI_CriticManager_5k_Trajs` into the same
  JSON, consistent with Tests 2–4.
- **Test 1 — shared obstacle field.** Both sides now evaluate obstacle cost
  against the **same** obstacles: two walls parallel to the path at `y = ±1.7 m`
  (just outside the trajectory envelope). The identical points feed the
  `kompass-core` side via `CostEvaluator::setPointScan` (with
  `obstacles_distance_weight = 1.0`) and the Nav2 side by stamping them as
  `LETHAL` cells into the `Costmap2DROS` and inflating them, so `ObstaclesCritic`
  reads graded proximity costs. Central/linear trajectories pass between the
  walls (graded repulsion), wide angular ones collide — a non-degenerate mix
  (`fail_flag = 0`, costs spanning ~0.05 → 2e5).
- **Test 1 — critic set.** Four critics are loaded: `PathAlignCritic`,
  `GoalCritic`, `PathFollowCritic`, `ObstaclesCritic`. Whether this is the final
  set (≈5 covering collision checking) is still open.
- **Test 1 — per-iteration reset (methodology).** `evalTrajectoriesScores`
  *accumulates* into its `costs` tensor (`costs += …`) and `ObstaclesCritic`
  latches `CriticData::fail_flag` once all trajectories collide, which then
  short-circuits the critic loop on the next call. The Nav2 workload therefore
  resets `costs` to zero and `fail_flag` to false each iteration — without this
  the measured time collapses to a no-op (it previously read ~150 ns instead of
  the real ~150 ms). The reset is *outside* the comparison's hot path on the
  `kompass-core` side, which has no equivalent latch.
- **Test 1 — obstacle-cost asymmetry.** Nav2's obstacle distance field is
  precomputed once (the costmap inflation, done outside the timed loop), so the
  critic's per-call obstacle work is O(trajectory points) costmap lookups.
  `kompass-core` computes trajectory-to-obstacle distances inside the timed call
  (work grows with obstacle count). This reflects a real architectural
  difference, not a bug, but means the obstacle term is not a like-for-like
  slice.
- **Test 2 — what is timed.** The Nav2 timed lambda now mirrors the slice the
  kompass `LocalMapperGPU::scanToGrid` kernel performs each call: the per-frame
  **grid reset** and the **polar→cartesian→`worldToMap`** conversion are both
  *inside* the timed region (previously the conversion was precomputed outside
  and there was no reset). The kompass kernel additionally pays host↔device
  transfers (H→D scan, D→H of the 640 KB grid) and a host `double`→`float`
  conversion; these have no pure-CPU equivalent on the Nav2 side and are **not**
  matched — most visible on the CPU-SYCL run, where they are real `memcpy`s that
  inflate the kompass number against a native CPU loop. Note the two sides still
  differ semantically: kompass does a binary `EMPTY`/`OCCUPIED` stamp, Nav2 does
  `FREE_SPACE`/`LETHAL_OBSTACLE` over a `NO_INFORMATION`-initialised grid.
- **Test 2b — pointcloud, dimensionality reduction (GPU only).** Both sides
  reduce the 100k-point cloud to a 3600-bin laserscan (nearest range per angular
  bin, z-filtered) *before* raycasting — kompass in its `pointcloud_to_laserscan`
  kernel, Nav2 in a CPU projection that mirrors the same binning. This is the
  deliberate, fair match: a `VoxelLayer`-style native 3D raytrace of every point
  would pit a 2D-reduced pipeline against a 3D one. The Nav2 timed slice (reset +
  projection + raytrace) mirrors the kompass `scanToGrid(cloud)` scope; H↔D
  transfers are again not matched. This test only builds under `GPU=1` (the
  pointcloud mapping path is GPU-only), so the Nav2 counterpart lives in the same
  `#ifdef GPU` block and only appears in SYCL/CUDA/ROCm runs.
- **Stale results file.** `benchmark_cpu_sycl.json` at the repo root predates
  the Bonxai → `Costmap2D` switch (it still lists `Bonxai_Dense_400x400`) and
  must be regenerated before being used.
- **No GPU comparison yet.** The provided `dockerfile` is CPU-only (no SYCL /
  AdaptiveCpp), so it cannot exercise the GPU kernels. A GPU comparison
  environment is still needed.

## Roadmap

- [ ] Confirm the Nav2 MPPI critic set for Test 1 is final (currently
      `PathAlignCritic`, `GoalCritic`, `PathFollowCritic`, `ObstaclesCritic`).
      Running both workloads, with a shared obstacle field exercised by both
      sides, is done.
- [ ] Regenerate `benchmark_cpu_sycl.json` after the Costmap2D switch.
- [ ] Add a GPU-capable comparison environment (SYCL + Nav2).
- [ ] Consider additional reference stacks beyond Nav2 where a closer functional
      match exists.
