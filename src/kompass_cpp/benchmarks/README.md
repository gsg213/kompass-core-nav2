# kompass-core vs. Nav2 - Comparison Benchmark Suite

> **This is the fork `kompass-core-nav2`.** Upstream's benchmark measures `kompass-core` against _itself_ across hardware backends. This fork adds a head-to-head comparison against [Nav2](https://navigation.ros.org/): for each `kompass-core` GPGPU kernel, an equivalent Nav2 C++ component is run on the **same generated input** through the **same timing harness** (`measure_performance`), and the two numbers are plotted side by side.
>
> The functional matches are _approximate_, not drop-in equivalents, and several have important fairness caveats (what exactly is timed, scope mismatches, host/device transfer asymmetry). These are documented in the per-test comments in [`benchmark_runner.cpp`](./benchmark_runner.cpp).

`kompass-core` kernels are single-source GPGPU written in SYCL and JIT-compiled with [AdaptiveCpp](https://github.com/AdaptiveCpp/AdaptiveCpp) to CPU (OpenMP), NVIDIA (CUDA) and AMD (ROCm). Nav2's components are plain CPU C++.

---

## What is compared

| #   | Test                       | `kompass-core`                                                                            | Nav2 equivalent                                               | JSON entries                                                   |
| --- | -------------------------- | ----------------------------------------------------------------------------------------- | ------------------------------------------------------------- | -------------------------------------------------------------- |
| 1   | Cost evaluator             | `Control::CostEvaluator::getMinTrajectoryCost`                                            | MPPI `CriticManager::evalTrajectoriesScores`                  | `CostEvaluator_5k_Trajs` / `Nav2_MPPI_CriticManager_5k_Trajs`  |
| 2   | Local map - laserscan      | `Mapping::LocalMapper` raycast to 400x400 grid                                            | `Costmap2D` `raytraceLine` + `setCost`                        | `Mapper_Dense_400x400` / `Nav2_costmap_Dense_400x400`          |
| 2b  | Local map - pointcloud     | `Mapping::LocalMapperGPU::scanToGrid` (PC-to-laserscan kernel then raycast), **GPU only** | `Costmap2D` (PC-to-laserscan reduction then raytrace)         | `Mapper_PointCloud_100k` / `Nav2_costmap_PointCloud_100k`      |
| 3   | Critical zone - pointcloud | `CriticalZoneChecker::check` (100k cloud)                                                 | Collision Monitor (`PointCloud` + `Polygon::getPointsInside`) | `CriticalZone_100k_Cloud` / `Nav2_CollisionMonitor_100k_Cloud` |
| 4   | Critical zone - laserscan  | `CriticalZoneChecker::check` (3600 rays)                                                  | Collision Monitor (`Scan` + `Polygon::getPointsInside`)       | `CriticalZone_Dense_Scan` / `Nav2_CollisionMonitor_Dense_Scan` |

Both sides of a test use the same generated input and emit one `kompass` and one `Nav2_*` entry. `plot_benchmarks.py` groups entries by name across machines.

---

## How it runs

The kompass kernels run on the accelerator; the Nav2 components run on the CPU. To keep every comparison on one device, however the two sides are built and run separately:

| Side                | Where it runs                                      | Build                                   | Emits            |
| ------------------- | -------------------------------------------------- | --------------------------------------- | ---------------- |
| **Nav2 reference**  | the provided container, on the target device's CPU | `BENCHMARK_WITH_KOMPASS=OFF` (CPU only) | `Nav2_*` numbers |
| **kompass kernels** | natively on the target device's GPU                | `BENCHMARK_WITH_NAV2=OFF` + AdaptiveCpp | kompass numbers  |

Two CMake switches select what a build measures (both default `ON`):

- **`BENCHMARK_WITH_NAV2`** - build/run the Nav2 workloads. Set `OFF` on the GPU device build so it needs no ROS install.
- **`BENCHMARK_WITH_KOMPASS`** - time/report the kompass workloads. Set `OFF` in the Nav2 container (the kompass library is still linked to generate the shared inputs, but its workloads are not timed), so the JSON carries only `Nav2_*` entries.

Leaving both `ON` gives the original single-machine build that emits every entry in one JSON - handy if you have ROS 2 + AdaptiveCpp on one host.

---

## Step 1 - Nav2 reference numbers (container)

The root [`dockerfile`](../../../dockerfile) is the Nav2 reference: ROS 2 Humble (Ubuntu 22.04 LTS, which matches JetPack 6 on Jetson), CPU-only (no CUDA / ROCm / AdaptiveCpp), multi-arch (`linux/amd64` for AMD, `linux/arm64` for Jetson), built with `BENCHMARK_WITH_KOMPASS=OFF`.

Run it on the device whose CPU you want as the baseline:

```bash
# from the repo root, on the target device:
docker build -f dockerfile -t kompass-nav2-ref .
#   cross-build both arches with buildx:
#   docker buildx build --platform linux/amd64,linux/arm64 -f dockerfile -t kompass-nav2-ref .

# Writes ./out/benchmark_nav2_<arch>.json, tagged with the label you pass:
mkdir -p out
docker run --rm -v "$PWD/out:/out" kompass-nav2-ref Nav2_Jetson_Orin_CPU
```

The argument (`Nav2_Jetson_Orin_CPU` above) becomes the `platform` field in the JSON - use it to identify the device.

## Step 2 - kompass numbers (native, on the device GPU)

Install AdaptiveCpp + the kompass build deps (the helper [`build_dependencies/install_gpu.sh`](../../../build_dependencies/install_gpu.sh) does this; pass `--cuda-root <path>` for the CUDA backend), then build with Nav2 off and run with the backend mask for the accelerator:

```bash
mkdir -p build_gpu && cd build_gpu
cmake .. -DCMAKE_BUILD_TYPE=Release -DBENCHMARK_WITH_NAV2=OFF
cmake --build . --target kompass_benchmark -- -j"$(nproc)"

# pick the backend for the device (writes one JSON per run):
ACPP_VISIBILITY_MASK=cuda ./src/kompass_cpp/benchmarks/kompass_benchmark "Jetson_Orin_CUDA" "../out/benchmark_cuda.json"  # NVIDIA
ACPP_VISIBILITY_MASK=hip  ./src/kompass_cpp/benchmarks/kompass_benchmark "Strix_ROCm"        "../out/benchmark_rocm.json"  # AMD
ACPP_VISIBILITY_MASK=omp  ./src/kompass_cpp/benchmarks/kompass_benchmark "CPU_SYCL"          "../out/benchmark_cpu.json"   # CPU (OpenMP)
```

For a pure CPU baseline without AdaptiveCpp, add `-DFORCE_CPU_BUILD=ON` (the kernels then run as the native C++ CPU implementation).

Note (Test 2b): the kompass pointcloud mapper is GPU-only, so `Mapper_PointCloud_100k` only appears in AdaptiveCpp builds. Its Nav2 counterpart `Nav2_costmap_PointCloud_100k` is CPU and is produced by the container in Step 1.

### Power / efficiency (optional)

Add `-DENABLE_POWER_MONITOR=ON` to sample on-board power rails (Jetson INA3221, AMD `amdgpu` hwmon, Rockchip/RPi supplies) and report Perf/Watt. The monitor polls sysfs on a background thread, so run speed and power sweeps separately for clean timing.

## Step 3 - compare (plot)

Collect every JSON - the device kompass run(s) and the container's Nav2 run(s) - into one folder and plot:

```bash
cd src/kompass_cpp/benchmarks
mkdir -p benchmark_files
cp /path/to/out/*.json benchmark_files/      # device + container JSONs
python3 plot_benchmarks.py
```

`plot_benchmarks.py` groups every `test_name` across all the JSONs, so each kompass kernel and its Nav2 counterpart show up per test, per device. It writes log-scale performance charts (and, when power data is present, Perf/Watt charts) to `docs/`. It separates "pure" timing runs from power-instrumented runs automatically, so you can mix both.

---

## Prerequisites

- **Nav2 side:** Docker only (the container bundles ROS 2 Humble + Nav2 + the build). To build it natively instead: ROS 2 Humble with `nav2_mppi_controller`, `nav2_costmap_2d`, `nav2_core`, `nav2_msgs`, `nav2_collision_monitor`, `geometry_msgs`, plus the kompass C++ deps and `PCL`.
- **kompass side:** AdaptiveCpp + the kompass build deps (Eigen 3.4+, FCL, OMPL, OpenCV, Boost, PCL, `nlohmann-json`). Use `build_dependencies/install_gpu.sh`.
- **Plotting:** Python 3 + `matplotlib`.
