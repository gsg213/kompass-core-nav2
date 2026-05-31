# =============================================================================
# Nav2 reference container for the kompass-core comparison benchmarks.
#
# Builds the benchmark in "Nav2-only" mode (BENCHMARK_WITH_KOMPASS=OFF): the
# kompass kernels are NOT timed -- only the Nav2 components are benchmarked and
# written to JSON. That JSON is a peer of the native CUDA / ROCm kompass device
# runs; plot_benchmarks.py aggregates them by test name.
#
#   docker buildx build --platform linux/amd64,linux/arm64 -f dockerfile -t kompass-nav2-ref .
# =============================================================================
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# kompass-core C++ build deps + the ROS 2 Humble Nav2 comparison packages.
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential \
      cmake \
      git \
      python3 \
      python3-dev \
      libeigen3-dev \
      libfcl-dev \
      libompl-dev \
      nlohmann-json3-dev \
      libboost-all-dev \
      libopencv-dev \
      libpcl-dev \
      ros-humble-nav2-mppi-controller \
      ros-humble-nav2-costmap-2d \
      ros-humble-nav2-core \
      ros-humble-nav2-msgs \
      ros-humble-nav2-collision-monitor \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
COPY . /workspace

# Build the Nav2-reference benchmark: CPU-native (FORCE_CPU_BUILD, no SYCL) and
# Nav2-only reporting (BENCHMARK_WITH_KOMPASS=OFF). The kompass C++ library is
# still compiled (it provides the shared input generators) but its workloads are
# not timed.
RUN . /opt/ros/humble/setup.sh && \
    cmake -S /workspace -B /workspace/build_nav2_ref \
      -DCMAKE_BUILD_TYPE=Release \
      -DFORCE_CPU_BUILD=ON \
      -DBENCHMARK_WITH_KOMPASS=OFF && \
    cmake --build /workspace/build_nav2_ref --target kompass_benchmark -j"$(nproc)" && \
    chmod +x /workspace/src/kompass_cpp/benchmarks/run_nav2_reference.sh

# Default run: emit /out/benchmark_nav2_<arch>.json. Pass a platform label as the
# first arg to tag the run with the device, e.g.:
#   docker run --rm -v "$PWD/out:/out" kompass-nav2-ref Nav2_Jetson_Orin_CPU
ENTRYPOINT ["/workspace/src/kompass_cpp/benchmarks/run_nav2_reference.sh"]
