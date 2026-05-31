#!/usr/bin/env bash
#
# Run the Nav2-reference benchmark and emit a JSON of ONLY the Nav2 numbers.
#
# The container builds the benchmark with BENCHMARK_WITH_KOMPASS=OFF, so the
# kompass-core workloads are not timed -- the JSON it produces is a peer of the
# native CUDA / ROCm kompass device runs and is aggregated by plot_benchmarks.py
# (by test name). Nav2 runs on the host CPU.
#
# Usage (inside the container):
#   run_nav2_reference.sh [platform_label] [output_json_path]
# Defaults:
#   platform = Nav2_Reference_<arch>
#   output   = /out/benchmark_nav2_<arch>.json   (mount -v "$PWD/out:/out")
source /opt/ros/humble/setup.bash
set -eo pipefail

ARCH="$(uname -m)"
PLATFORM="${1:-Nav2_Reference_${ARCH}}"
OUT="${2:-/out/benchmark_nav2_${ARCH}.json}"
BIN="${KOMPASS_BENCH_BIN:-/workspace/build_nav2_ref/src/kompass_cpp/benchmarks/kompass_benchmark}"

mkdir -p "$(dirname "${OUT}")"
echo "[run_nav2_reference] arch=${ARCH} platform=${PLATFORM} out=${OUT}"
exec "${BIN}" "${PLATFORM}" "${OUT}"
