#!/usr/bin/env bash
set -euo pipefail

MODE="${1:-right}"
if [[ $# -gt 0 ]]; then
  shift
fi

if [[ "${MODE}" != "left" && "${MODE}" != "right" && "${MODE}" != "both" ]]; then
  echo "Usage: start_revo3_driver.sh [left|right|both] [extra ros2 launch args...]" >&2
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "${SCRIPT_DIR}/.." && pwd)"
SETUP="${WORKSPACE}/install/setup.bash"

if [[ ! -f "${SETUP}" ]]; then
  echo "[revo3_driver] Missing ${SETUP}. Run python -m colcon build --symlink-install first." >&2
  exit 1
fi

set +u
source "${SETUP}"
set -u

REVO3_LAUNCH_RSP="${REVO3_LAUNCH_RSP:-true}"
REVO3_LAUNCH_RVIZ="${REVO3_LAUNCH_RVIZ:-false}"
REVO3_UPDATE_RATE="${REVO3_UPDATE_RATE:-200}"

if [[ "${MODE}" == "both" ]]; then
  launch_file="dual_revo3_system.launch.py"
  launch_args=(
    "if_sim:=false"
    "launch_rsp:=${REVO3_LAUNCH_RSP}"
    "launch_rviz:=${REVO3_LAUNCH_RVIZ}"
    "update_rate:=${REVO3_UPDATE_RATE}"
  )
  if [[ -n "${REVO3_LEFT_PROTOCOL_CONFIG:-}" ]]; then
    launch_args+=("left_protocol_config_file:=${REVO3_LEFT_PROTOCOL_CONFIG}")
  fi
  if [[ -n "${REVO3_RIGHT_PROTOCOL_CONFIG:-}" ]]; then
    launch_args+=("right_protocol_config_file:=${REVO3_RIGHT_PROTOCOL_CONFIG}")
  fi
else
  launch_file="revo3_system.launch.py"
  config_var="REVO3_${MODE^^}_PROTOCOL_CONFIG"
  launch_args=(
    "hand_side:=${MODE}"
    "if_sim:=false"
    "launch_rsp:=${REVO3_LAUNCH_RSP}"
    "launch_rviz:=${REVO3_LAUNCH_RVIZ}"
    "update_rate:=${REVO3_UPDATE_RATE}"
  )
  if [[ -n "${!config_var:-}" ]]; then
    launch_args+=("protocol_config_file:=${!config_var}")
  elif [[ -n "${REVO3_PROTOCOL_CONFIG:-}" ]]; then
    launch_args+=("protocol_config_file:=${REVO3_PROTOCOL_CONFIG}")
  fi
fi

echo "[revo3_driver] Starting ${MODE} hardware driver (${launch_file})"
ros2 launch revo3_driver "${launch_file}" "${launch_args[@]}" "$@"
