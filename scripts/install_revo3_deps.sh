#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "${SCRIPT_DIR}/.." && pwd)"
if command -v python >/dev/null 2>&1; then
  PYTHON_BIN="${PYTHON:-python}"
else
  PYTHON_BIN="${PYTHON:-python3}"
fi

APT_PACKAGES=(
  git
  git-lfs
  curl
  unzip
  build-essential
  cmake
  libgl1
  libglx-mesa0
  libusb-1.0-0
  python3-pip
  python3-venv
  python3-yaml
  python3-tk
  libeigen3-dev
  "ros-${ROS_DISTRO}-ros-base"
  "ros-${ROS_DISTRO}-ros2-control"
  "ros-${ROS_DISTRO}-ros2-controllers"
  "ros-${ROS_DISTRO}-xacro"
  "ros-${ROS_DISTRO}-robot-state-publisher"
  "ros-${ROS_DISTRO}-joint-state-publisher-gui"
  "ros-${ROS_DISTRO}-rviz2"
  "ros-${ROS_DISTRO}-eigen3-cmake-module"
  "ros-${ROS_DISTRO}-pinocchio"
  "ros-${ROS_DISTRO}-rosbag2-storage-mcap"
)

echo "[deps] Installing apt dependencies for ROS ${ROS_DISTRO}..."
sudo apt-get update
sudo apt-get install -y "${APT_PACKAGES[@]}"

echo "[deps] Initializing Git LFS and submodules..."
git lfs install --local
git submodule update --init --recursive

echo "[deps] Checking MANUS SDK shared libraries..."
"${SCRIPT_DIR}/install_manus_sdk.sh"

echo "[deps] Installing Python dependencies into: $("${PYTHON_BIN}" -c 'import sys; print(sys.executable)')"
"${PYTHON_BIN}" -m pip install --upgrade pip
"${PYTHON_BIN}" -m pip install -r "${WORKSPACE}/requirements.txt"

echo "[deps] Verifying dependency set..."
"${SCRIPT_DIR}/check_system_deps.sh" revo3

cat <<EOF
[deps] Revo3 dependencies are ready.

Next:
  source /opt/ros/${ROS_DISTRO}/setup.bash
  ${PYTHON_BIN} -m colcon build --symlink-install --packages-select \\
    manus_ros2_msgs manus_ros2 \\
    revo3_mit_controller_msgs revo3_description revo3_mit_controller revo3_driver \\
    manus_revo3_retarget

For real hardware serial aliases and permissions:
  cd src/brainco_revo3_ros2/revo3_driver/setup
  bash bootstrap_revo3.sh
  bash check_revo3_setup.sh

EOF
