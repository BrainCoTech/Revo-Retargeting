#!/usr/bin/env bash
set -euo pipefail

MODE="${1:-right}"
if [[ $# -gt 0 ]]; then
  shift
fi

if [[ "${MODE}" != "left" && "${MODE}" != "right" && "${MODE}" != "both" ]]; then
  echo "Usage: teleop_revo3.sh [left|right|both] [extra manus_revo3_retarget launch args...]" >&2
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "${SCRIPT_DIR}/.." && pwd)"
SETUP="${WORKSPACE}/install/setup.bash"

if [[ ! -f "${SETUP}" ]]; then
  echo "[teleop_revo3] Missing ${SETUP}. Run python -m colcon build --symlink-install first." >&2
  exit 1
fi

START_MANUS_PUBLISHER="${START_MANUS_PUBLISHER:-1}"
START_REVO3_DRIVER="${START_REVO3_DRIVER:-1}"

set +u
source "${SETUP}"
set -u

managed_pids=()

start_managed() {
  local label="$1"
  shift
  echo "[teleop_revo3] Starting ${label}..."
  setsid "$@" &
  local pid=$!
  managed_pids+=("${pid}:${label}")
}

signal_process_groups() {
  local signal="$1"
  local entry pgid label
  for ((idx=${#managed_pids[@]}-1; idx>=0; idx--)); do
    entry="${managed_pids[$idx]}"
    pgid="${entry%%:*}"
    label="${entry#*:}"
    if kill -0 "-${pgid}" 2>/dev/null; then
      echo "[teleop_revo3] Sending ${signal} to ${label}..."
      kill "-${signal}" "-${pgid}" 2>/dev/null || true
    fi
  done
}

wait_process_groups() {
  local attempts="$1"
  local entry pgid alive
  for ((attempt=0; attempt<attempts; attempt++)); do
    alive=0
    for entry in "${managed_pids[@]}"; do
      pgid="${entry%%:*}"
      if kill -0 "-${pgid}" 2>/dev/null; then
        alive=1
        break
      fi
    done
    if [[ "${alive}" == "0" ]]; then
      return 0
    fi
    sleep 0.1
  done
  return 1
}

reap_managed_pids() {
  local entry pid
  for entry in "${managed_pids[@]}"; do
    pid="${entry%%:*}"
    wait "${pid}" 2>/dev/null || true
  done
}

cleanup() {
  trap - EXIT
  trap '' INT TERM
  signal_process_groups INT
  wait_process_groups 30 || {
    signal_process_groups TERM
    wait_process_groups 20 || signal_process_groups KILL
  }
  reap_managed_pids
}

wait_for_any() {
  while true; do
    local entry pid
    for entry in "${managed_pids[@]}"; do
      pid="${entry%%:*}"
      if ! kill -0 "-${pid}" 2>/dev/null; then
        wait "${pid}" 2>/dev/null || true
        return
      fi
    done
    sleep 0.2
  done
}

handle_signal() {
  cleanup
  exit 130
}

trap cleanup EXIT
trap handle_signal INT TERM

if [[ "${START_REVO3_DRIVER}" == "1" ]]; then
  start_managed "Revo3 driver" "${SCRIPT_DIR}/start_revo3_driver.sh" "${MODE}"
fi

if [[ "${START_MANUS_PUBLISHER}" == "1" ]]; then
  start_managed "MANUS publisher" ros2 run manus_ros2 manus_data_publisher
fi

start_managed "Revo3 retarget" ros2 launch manus_revo3_retarget pipeline_launch.py \
  hand_mode:="${MODE}" \
  launch_manus_publisher:=false \
  "$@"

wait_for_any
