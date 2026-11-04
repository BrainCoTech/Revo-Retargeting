#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "${SCRIPT_DIR}/.." && pwd)"
MODEL="${1:-revo3}"
if [[ "${MODEL}" == "3" ]]; then
  MODEL="revo3"
fi
case "${MODEL}" in
  revo3) ;;
  *)
    echo "Usage: check_system_deps.sh [revo3]" >&2
    exit 2
    ;;
esac

ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_ROOT="/opt/ros/${ROS_DISTRO}"
missing_ros=()
missing_cmds=()
missing_paths=()
missing_python=()
missing_manus_sdk=()

if command -v python >/dev/null 2>&1; then
  PYTHON_BIN="${PYTHON:-python}"
elif command -v python3 >/dev/null 2>&1; then
  PYTHON_BIN="${PYTHON:-python3}"
else
  PYTHON_BIN="${PYTHON:-python}"
fi

require_ros_share() {
  local pkg="$1"
  if [[ ! -d "${ROS_ROOT}/share/${pkg}" ]]; then
    missing_ros+=("${pkg}")
  fi
}

require_cmd() {
  local cmd="$1"
  if ! command -v "${cmd}" >/dev/null 2>&1; then
    missing_cmds+=("${cmd}")
  fi
}

require_path() {
  local path="$1"
  local label="$2"
  if [[ ! -e "${path}" ]]; then
    missing_paths+=("${label}: ${path}")
  fi
}

is_lfs_pointer() {
  local file="$1"
  [[ -f "${file}" ]] && head -c 128 "${file}" | grep -q "version https://git-lfs.github.com/spec/v1"
}

require_real_file() {
  local path="$1"
  local label="$2"
  if [[ ! -f "${path}" ]]; then
    missing_manus_sdk+=("${label}: ${path}")
  elif is_lfs_pointer "${path}"; then
    missing_manus_sdk+=("${label} is a Git LFS pointer: ${path}")
  fi
}

require_python_import() {
  local module="$1"
  if ! "${PYTHON_BIN}" - <<PY >/dev/null 2>&1
import ${module}
PY
  then
    missing_python+=("${module}")
  fi
}

require_python_expr() {
  local label="$1"
  local expr="$2"
  if ! "${PYTHON_BIN}" - <<PY >/dev/null 2>&1
${expr}
PY
  then
    missing_python+=("${label}")
  fi
}

require_cmd git
require_cmd git-lfs
require_cmd "${PYTHON_BIN}"
require_cmd colcon

require_path "${WORKSPACE}/src/brainco_revo3_ros2/revo3_driver/package.xml" "Revo3 driver submodule"
require_path "${WORKSPACE}/src/manus_revo3_retarget/package.xml" "MANUS Revo3 retarget package"
require_path "${WORKSPACE}/src/manus_ros2/ManusSDK/include/ManusSDK.h" "MANUS SDK header"
if [[ -f "${WORKSPACE}/src/manus_ros2/ManusSDK/lib/libManusSDK.so" || -f "${WORKSPACE}/src/manus_ros2/ManusSDK/lib/libManusSDK_Integrated.so" ]]; then
  if [[ -f "${WORKSPACE}/src/manus_ros2/ManusSDK/lib/libManusSDK.so" ]]; then
    require_real_file "${WORKSPACE}/src/manus_ros2/ManusSDK/lib/libManusSDK.so" "MANUS SDK library"
  fi
  if [[ -f "${WORKSPACE}/src/manus_ros2/ManusSDK/lib/libManusSDK_Integrated.so" ]]; then
    require_real_file "${WORKSPACE}/src/manus_ros2/ManusSDK/lib/libManusSDK_Integrated.so" "MANUS SDK integrated library"
  fi
else
  missing_manus_sdk+=("MANUS SDK library: ${WORKSPACE}/src/manus_ros2/ManusSDK/lib/libManusSDK*.so")
fi

require_ros_share ament_cmake
require_ros_share ament_cmake_ros
require_ros_share rosidl_default_generators
require_ros_share rosidl_default_runtime
require_ros_share hardware_interface
require_ros_share controller_interface
require_ros_share controller_manager
require_ros_share forward_command_controller
require_ros_share joint_state_broadcaster
require_ros_share joint_trajectory_controller
require_ros_share pluginlib
require_ros_share rclcpp
require_ros_share rclcpp_lifecycle
require_ros_share rclpy
require_ros_share rcl_interfaces
require_ros_share std_msgs
require_ros_share sensor_msgs
require_ros_share geometry_msgs
require_ros_share launch
require_ros_share launch_ros
require_ros_share xacro
require_ros_share robot_state_publisher
require_ros_share rviz2
require_ros_share joint_state_publisher_gui
require_ros_share eigen3_cmake_module
require_ros_share pinocchio

require_python_import numpy
require_python_import yaml
require_python_import catkin_pkg
require_python_import lark
require_python_import colcon_core
require_python_expr "empy==3.3.4 compatible em module" "import em; assert hasattr(em, 'BUFFERED_OPT')"

if [[ ${#missing_cmds[@]} -gt 0 || ${#missing_paths[@]} -gt 0 || ${#missing_manus_sdk[@]} -gt 0 || ${#missing_ros[@]} -gt 0 || ${#missing_python[@]} -gt 0 ]]; then
  if [[ ${#missing_cmds[@]} -gt 0 ]]; then
    cat >&2 <<EOF
[deps] Missing commands:
  ${missing_cmds[*]}

Install system dependencies:
  sudo apt-get install -y git git-lfs build-essential cmake libeigen3-dev libgl1 libglx-mesa0 libusb-1.0-0 python3-pip python3-venv python3-yaml python3-tk
EOF
  fi

  if [[ ${#missing_paths[@]} -gt 0 ]]; then
    printf '[deps] Missing workspace paths:\n' >&2
    printf '  %s\n' "${missing_paths[@]}" >&2
    cat >&2 <<EOF

Initialize submodules from the repository root:
  git submodule update --init --recursive
EOF
  fi

  if [[ ${#missing_manus_sdk[@]} -gt 0 ]]; then
    printf '[deps] Missing MANUS SDK files:\n' >&2
    printf '  %s\n' "${missing_manus_sdk[@]}" >&2
    cat >&2 <<EOF

Download the official MANUS SDK, then install it with one of:
  MANUS_SDK_ARCHIVE=/path/to/MANUS_SDK.zip ./scripts/install_manus_sdk.sh
  MANUS_SDK_DIR=/path/to/unpacked/ManusSDK ./scripts/install_manus_sdk.sh
  MANUS_SDK_URL=https://.../MANUS_SDK.zip ./scripts/install_manus_sdk.sh
EOF
  fi

  if [[ ${#missing_ros[@]} -gt 0 ]]; then
  cat >&2 <<EOF
[deps] Missing ROS ${ROS_DISTRO} packages required for ${MODEL}:
  ${missing_ros[*]}

Install the Revo3 ROS dependency set:
  sudo apt-get install -y \\
    git git-lfs build-essential cmake \\
    libeigen3-dev libgl1 libglx-mesa0 libusb-1.0-0 \\
    python3-pip python3-venv python3-yaml python3-tk \\
    ros-${ROS_DISTRO}-ros-base \\
    ros-${ROS_DISTRO}-ros2-control \\
    ros-${ROS_DISTRO}-ros2-controllers \\
    ros-${ROS_DISTRO}-xacro \\
    ros-${ROS_DISTRO}-robot-state-publisher \\
    ros-${ROS_DISTRO}-joint-state-publisher-gui \\
    ros-${ROS_DISTRO}-rviz2 \\
    ros-${ROS_DISTRO}-eigen3-cmake-module \\
    ros-${ROS_DISTRO}-pinocchio \\
    ros-${ROS_DISTRO}-rosbag2-storage-mcap
EOF
  fi

  if [[ ${#missing_python[@]} -gt 0 ]]; then
    cat >&2 <<EOF
[deps] Missing Python imports in the active environment:
  ${missing_python[*]}

Activate the intended Python/conda environment and install:
  ${PYTHON_BIN} -m pip install -r requirements.txt
EOF
  fi

  cat >&2 <<EOF

You can install the full dependency set with:
  ./scripts/install_revo3_deps.sh
EOF
  exit 1
fi

echo "[deps] ${MODEL} dependency check completed."
