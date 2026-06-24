#!/usr/bin/env bash
set -euo pipefail

SIDE="${1:-right}"
if [[ $# -gt 0 ]]; then
  shift
fi

if [[ "${SIDE}" != "left" && "${SIDE}" != "right" ]]; then
  echo "Usage: calibrate_manus.sh [left|right] [extra manus_calibration_tool args...]" >&2
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "${SCRIPT_DIR}/.." && pwd)"
SETUP="${WORKSPACE}/install/setup.bash"

if [[ ! -f "${SETUP}" ]]; then
  echo "[calibrate] Missing ${SETUP}. Run python -m colcon build --symlink-install first." >&2
  exit 1
fi

set +u
source "${SETUP}"
set -u

ros2 run manus_ros2 manus_calibration_tool --side "${SIDE}" --overwrite "$@"
