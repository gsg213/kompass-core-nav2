#!/usr/bin/env bash
# =============================================================================
# Install ROS 2 Humble + Nav2 on a non-22.04 host (e.g. Ubuntu 24.04) WITHOUT
# Docker or root namespaces, via RoboStack (conda packages).
#
# Why: ROS apt binaries are pinned to one Ubuntu release (Humble->22.04,
# Jazzy->24.04). RoboStack ships Humble as conda packages that bundle their own
# libstdc++/deps, so the SAME Humble Nav2 we use on the Jetson baseline installs
# on any Linux, in userspace, with no Docker daemon and no user namespaces
# (which Ubuntu 24.04 restricts). Timing is native -- no ptrace/VM overhead.
#
# This creates a conda env with the same Nav2 packages + kompass build deps the
# Nav2-reference Dockerfile installs, so the benchmark builds the SAME way, just
# against the conda env instead of the apt ROS install. See build steps printed
# at the end (and the project chat) for how to compile + run.
#
# Usage:
#   ./setup_robostack_humble.sh            # installs into ~/micromamba, env "nav2ref"
#   ENV_NAME=foo ROOT_PREFIX=/opt/mm ./setup_robostack_humble.sh
# =============================================================================
set -euo pipefail

ENV_NAME="${ENV_NAME:-nav2ref}"
ROOT_PREFIX="${ROOT_PREFIX:-${HOME}/micromamba}"
export MAMBA_ROOT_PREFIX="${ROOT_PREFIX}"

ARCH="$(uname -m)"
case "${ARCH}" in
  x86_64)  MM_ARCH="linux-64"  ;;
  aarch64) MM_ARCH="linux-aarch64" ;;
  *) echo "[error] unsupported arch '${ARCH}'"; exit 1 ;;
esac
echo "[*] arch=${ARCH} (micromamba ${MM_ARCH}); root_prefix=${ROOT_PREFIX}; env=${ENV_NAME}"

# --- 1. Bootstrap micromamba (userspace, single static binary) ---------------
MM_BIN="${ROOT_PREFIX}/bin/micromamba"
if [ ! -x "${MM_BIN}" ]; then
  echo "[*] downloading micromamba -> ${MM_BIN}"
  mkdir -p "${ROOT_PREFIX}"
  curl -Ls "https://micro.mamba.pm/api/micromamba/${MM_ARCH}/latest" \
    | tar -xj -C "${ROOT_PREFIX}" bin/micromamba
else
  echo "[*] micromamba already present"
fi

# --- 2. Create the env: RoboStack Humble + Nav2 + kompass build deps ---------
# Channels: robostack-staging (ROS Humble pkgs) + conda-forge (everything else).
# Nav2 list mirrors the Dockerfile's apt packages exactly. The rest (eigen,
# nlohmann_json, pcl, fcl, ompl, opencv, boost) are what libkompass links.
if "${MM_BIN}" env list | grep -qE "[/ ]${ENV_NAME}\$"; then
  echo "[*] env '${ENV_NAME}' already exists -- skipping create."
  echo "    (delete with: ${MM_BIN} env remove -n ${ENV_NAME})"
else
  echo "[*] creating env '${ENV_NAME}' (this pulls a few GB; ~10-20 min)..."
  "${MM_BIN}" create -y -n "${ENV_NAME}" \
    -c robostack-staging -c conda-forge \
    ros-humble-ros-base \
    ros-humble-nav2-mppi-controller \
    ros-humble-nav2-costmap-2d \
    ros-humble-nav2-core \
    ros-humble-nav2-msgs \
    ros-humble-nav2-collision-monitor \
    compilers cmake make ninja pkg-config \
    eigen nlohmann_json pcl fcl ompl opencv libboost-devel
fi

# --- 3. Next steps -----------------------------------------------------------
cat <<EOF

[done] ROS 2 Humble + Nav2 installed via RoboStack in env '${ENV_NAME}'.

Activate it in this shell:

    export MAMBA_ROOT_PREFIX="${ROOT_PREFIX}"
    eval "\$(${MM_BIN} shell hook -s bash)"
    micromamba activate ${ENV_NAME}

Activating the env auto-sources ROS (sets AMENT_PREFIX_PATH etc.) -- no
'source /opt/ros/humble/setup.bash' needed. Then build + run the Nav2-only
benchmark from the repo root (see the build instructions).
EOF
