#!/usr/bin/env bash
# Configure Python tooling for the Revo2 / Revo3 ROS 2 Humble workflows.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

usage() {
  cat <<'HELP'
Usage: bash scripts/setup_revo_conda.sh [ENV_NAME] [--check]

Default environment: revo_teleop
Creates or updates a Python 3.10 environment. Requires an existing conda
installation and /opt/ros/humble. Does not install ROS, hardware SDKs, or
system C++ Pinocchio; does not build the workspace or start hardware.
--check only checks an existing environment; it installs nothing.
HELP
}

env_name=revo_teleop
check_only=false
name_given=false
for arg in "$@"; do
  case "$arg" in
    -h|--help) usage; exit 0 ;;
    --check) check_only=true ;;
    -*) echo "Unknown option: $arg" >&2; exit 2 ;;
    *)
      if $name_given; then usage >&2; exit 2; fi
      env_name=$arg
      name_given=true
      ;;
  esac
done
if [[ ! "$env_name" =~ ^[a-zA-Z0-9_][a-zA-Z0-9_.-]*$ || "$env_name" == base ]]; then
  echo 'Use a named environment other than base (letters, digits, _, ., -).' >&2
  exit 2
fi
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo 'ROS 2 Humble is required at /opt/ros/humble.' >&2
  exit 1
fi

# Support both initialized shells and common uninitialized conda installations.
if command -v conda >/dev/null 2>&1; then
  conda_base=$(conda info --base)
else
  conda_base=''
  for candidate in "$HOME/miniforge3" "$HOME/miniconda3" "$HOME/anaconda3"; do
    if [[ -f "$candidate/etc/profile.d/conda.sh" ]]; then
      conda_base=$candidate
      break
    fi
  done
fi
if [[ ! -f "$conda_base/etc/profile.d/conda.sh" ]]; then
  echo 'Conda was not found. Install conda or initialize it before running this script.' >&2
  exit 1
fi
source "$conda_base/etc/profile.d/conda.sh"
export PYTHONNOUSERSITE=1

if ! $check_only; then
  # Keep ROS Python extensions on the Python 3.10 / NumPy 1.x ABI.
  packages=(python=3.10 pip 'numpy>=1.24,<2' pyyaml scipy matplotlib pytest 'setuptools<80')
  if conda run -n "$env_name" python -c 'pass' >/dev/null 2>&1; then
    conda run -n "$env_name" python -c \
      'import sys; assert sys.version_info[:2] == (3, 10), "Existing environment must use Python 3.10; choose a new name."'
    conda install --yes --name "$env_name" --override-channels --channel conda-forge "${packages[@]}"
  else
    conda create --yes --name "$env_name" --override-channels --channel conda-forge "${packages[@]}"
  fi
  # Retargeting does not need CUDA. Preserve a working torch installation.
  if ! conda run -n "$env_name" python -c 'import torch' >/dev/null 2>&1; then
    conda run --no-capture-output -n "$env_name" python -m pip install \
      --index-url https://download.pytorch.org/whl/cpu torch
  fi
  conda run --no-capture-output -n "$env_name" python -m pip install \
    'setuptools<80' 'numpy>=1.24,<2' colcon-common-extensions 'empy==3.3.4' catkin-pkg lark \
    'mujoco>=3.0' dex-retargeting pyserial typeguard
fi

conda activate "$env_name"
source /opt/ros/humble/setup.bash
python -m pip check
python - "${SCRIPT_DIR}/.." <<'PY'
import sys
from pathlib import Path
sys.path.insert(0, str(Path(sys.argv[1]) / "src/brainco_capabilities/revohuman_kinematics"))
assert sys.version_info[:2] == (3, 10), sys.version
import numpy
assert int(numpy.__version__.split('.')[0]) == 1, numpy.__version__
import yaml
import scipy
import matplotlib
import pytest
import em
import catkin_pkg
import lark
import colcon_core
from revohuman_kinematics.joint_fk import JointStateFK
import mujoco
import serial
from dex_retargeting.retargeting_config import RetargetingConfig
import rclpy
from rclpy.impl.implementation_singleton import rclpy_implementation
from launch import LaunchDescription
from launch_ros.actions import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseArray
print('Python:', sys.executable)
print('NumPy:', numpy.__version__)
print('ROS and Python import checks passed; no nodes were started.')
PY
printf '\nEnvironment ready. In your terminal run:\n'
printf 'source %q\n' "$conda_base/etc/profile.d/conda.sh"
printf 'conda activate %q\n' "$env_name"
printf '%s\n' 'export PYTHONNOUSERSITE=1' 'source /opt/ros/humble/setup.bash' \
  'source install/setup.bash'
