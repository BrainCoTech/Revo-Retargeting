#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "Usage: teleop.sh [left|right|both] [input_source:=manus|humandex|dv1|external] [launch arguments...]"
  echo "  retarget_backend:=legacy|vector (default: legacy; vector requires MANUS input)."
  echo "  Vector starts disabled. Use its enable_retarget service after calibration."
  echo "DV1: acquisition runs separately; sdk_path:=DIR or urdf_path:=FILE selects the SDK URDF."
  echo "Optional DV1 arguments: adapter_config:=FILE, left_adapter_config:=FILE, right_adapter_config:=FILE,"
  echo "  left_joint_topic:=TOPIC, right_joint_topic:=TOPIC. Default input remains manus."
  echo "  joint_state_layout:=sdk_single|sdk_pair|legacy (default: sdk_pair for both, sdk_single otherwise),"
  echo "  left_source_frame_id:=FRAME, right_source_frame_id:=FRAME."
}
if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  usage
  exit 0
fi
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

# Resolve input before starting any process, especially the hardware driver.
INPUT_SOURCE=manus
RETARGET_BACKEND=legacy
SDK_PATH="${WORKSPACE}/../brainco_revohuman_sdk"
URDF_PATH=""
ADAPTER_CONFIG=""
LEFT_ADAPTER_CONFIG=""
RIGHT_ADAPTER_CONFIG=""
LEFT_JOINT_TOPIC=""
RIGHT_JOINT_TOPIC=""
JOINT_STATE_LAYOUT=""
LEFT_SOURCE_FRAME_ID=""
RIGHT_SOURCE_FRAME_ID=""
LEFT_INPUT_TOPIC=/hand_kinematics/left
RIGHT_INPUT_TOPIC=/hand_kinematics/right
INPUT_CONFIG=""
pipeline_args=()
for arg in "$@"; do
  case "$arg" in
    retarget_backend:=*) RETARGET_BACKEND="${arg#*:=}" ;;
    input_source:=*) INPUT_SOURCE="${arg#*:=}" ;;
    sdk_path:=*) SDK_PATH="${arg#*:=}" ;;
    urdf_path:=*) URDF_PATH="${arg#*:=}" ;;
    adapter_config:=*) ADAPTER_CONFIG="${arg#*:=}" ;;
    left_adapter_config:=*) LEFT_ADAPTER_CONFIG="${arg#*:=}" ;;
    right_adapter_config:=*) RIGHT_ADAPTER_CONFIG="${arg#*:=}" ;;
    left_joint_topic:=*) LEFT_JOINT_TOPIC="${arg#*:=}" ;;
    right_joint_topic:=*) RIGHT_JOINT_TOPIC="${arg#*:=}" ;;
    joint_state_layout:=*) JOINT_STATE_LAYOUT="${arg#*:=}" ;;
    left_source_frame_id:=*) LEFT_SOURCE_FRAME_ID="${arg#*:=}" ;;
    right_source_frame_id:=*) RIGHT_SOURCE_FRAME_ID="${arg#*:=}" ;;
    left_input_topic:=*) LEFT_INPUT_TOPIC="${arg#*:=}"; pipeline_args+=("$arg") ;;
    right_input_topic:=*) RIGHT_INPUT_TOPIC="${arg#*:=}"; pipeline_args+=("$arg") ;;
    input_config:=*) INPUT_CONFIG="${arg#*:=}" ;;
    hand_mode:=*|hand_type:=*)
      echo "Select the hand with the first positional argument, not $arg" >&2; exit 2 ;;
    launch_manus_publisher:=*)
      START_MANUS_PUBLISHER="${arg#*:=}"
      case "$START_MANUS_PUBLISHER" in
        true|1) START_MANUS_PUBLISHER=1 ;;
        false|0) START_MANUS_PUBLISHER=0 ;;
        *) echo "launch_manus_publisher must be true or false" >&2; exit 2 ;;
      esac ;;
    *) pipeline_args+=("$arg") ;;
  esac
done
case "$INPUT_SOURCE" in
  manus|humandex|dv1|external) ;;
  *) echo "Unknown input_source: $INPUT_SOURCE" >&2; exit 2 ;;
esac
case "$RETARGET_BACKEND" in
  legacy|vector) ;;
  *) echo "Unknown retarget_backend: $RETARGET_BACKEND" >&2; exit 2 ;;
esac
if [[ "$RETARGET_BACKEND" == vector ]]; then
  if [[ "$INPUT_SOURCE" != manus ]]; then
    echo "Vector requires input_source:=manus (the complete raw MANUS skeleton)." >&2
    exit 2
  fi
  # Reject unsupported mappings/options before starting the hardware driver.
  if [[ -n "$INPUT_CONFIG" ]]; then
    echo "Vector uses per-hand profiles, not input_config." >&2; exit 2
  fi
  for arg in "${pipeline_args[@]}"; do
    case "$arg" in
      left_profile:=*|right_profile:=*|left_glove_id:=*|right_glove_id:=*|cache_dir:=*|calibration_frames:=*|normal_sign:=*|numeric_threads:=*|interpolate:=*|max_velocity_rad_s:=*|max_acceleration_rad_s2:=*|thumb_offset_deg:=*) ;;
      *) echo "Unsupported Vector launch argument: $arg; see the Vector package README." >&2; exit 2 ;;
    esac
  done
fi
START_MANUS_PUBLISHER="${START_MANUS_PUBLISHER:-1}"
case "$START_MANUS_PUBLISHER" in
  true|1) START_MANUS_PUBLISHER=1 ;;
  false|0) START_MANUS_PUBLISHER=0 ;;
  *) echo "START_MANUS_PUBLISHER must be 0, 1, true or false" >&2; exit 2 ;;
esac
if [[ "$INPUT_SOURCE" != manus ]]; then START_MANUS_PUBLISHER=0; fi
START_REVO3_DRIVER="${START_REVO3_DRIVER:-1}"

set +u
source "${SETUP}"
set -u

if [[ "$RETARGET_BACKEND" == vector ]]; then
  vector_share="$(ros2 pkg prefix --share manus_revo3_vector_retarget)" || {
    echo "[teleop_revo3] Build manus_revo3_vector_retarget in the revo_teleop environment first." >&2
    exit 1
  }
  if [[ ! -f "$vector_share/launch/pipeline_launch.py" ]]; then
    echo "[teleop_revo3] Vector pipeline is missing; rebuild manus_revo3_vector_retarget." >&2
    exit 1
  fi
fi

if [[ "$INPUT_SOURCE" == dv1 ]]; then
  if [[ -z "$JOINT_STATE_LAYOUT" ]]; then
    JOINT_STATE_LAYOUT=sdk_single
    if [[ "$MODE" == both ]]; then JOINT_STATE_LAYOUT=sdk_pair; fi
  fi
  case "$JOINT_STATE_LAYOUT" in
    sdk_single)
      LEFT_JOINT_TOPIC="${LEFT_JOINT_TOPIC:-/revohuman/left/joint_states}"
      RIGHT_JOINT_TOPIC="${RIGHT_JOINT_TOPIC:-/revohuman/right/joint_states}" ;;
    sdk_pair)
      LEFT_JOINT_TOPIC="${LEFT_JOINT_TOPIC:-/revohuman/pair/joint_states}"
      RIGHT_JOINT_TOPIC="${RIGHT_JOINT_TOPIC:-/revohuman/pair/joint_states}" ;;
    legacy)
      LEFT_JOINT_TOPIC="${LEFT_JOINT_TOPIC:-/humandex_left/joint_states}"
      RIGHT_JOINT_TOPIC="${RIGHT_JOINT_TOPIC:-/humandex_right/joint_states}" ;;
    *) echo "Unknown joint_state_layout: $JOINT_STATE_LAYOUT" >&2; exit 2 ;;
  esac
  URDF_PATH="${URDF_PATH:-${SDK_PATH}/description/urdf/Revo_Human_DV1_URDF_Bimanual.urdf}"
  if [[ ! -f "$URDF_PATH" ]]; then
    echo "[teleop_revo3] DV1 URDF not found: $URDF_PATH; set sdk_path or urdf_path." >&2
    exit 1
  fi
  for config in "$ADAPTER_CONFIG" "$LEFT_ADAPTER_CONFIG" "$RIGHT_ADAPTER_CONFIG"; do
    if [[ -n "$config" && ! -f "$config" ]]; then
      echo "[teleop_revo3] Adapter config not found: $config" >&2; exit 1
    fi
  done
  adapter_share="$(ros2 pkg prefix --share hand_input_adapters)"
  if [[ ! -f "$adapter_share/launch/dv1_input.launch.py" ]]; then
    echo "[teleop_revo3] Rebuild hand_input_adapters: dv1_input.launch.py is missing." >&2; exit 1
  fi
  if [[ -z "$INPUT_CONFIG" ]]; then
    retarget_share="$(ros2 pkg prefix --share manus_revo3_retarget)"
    INPUT_CONFIG="$retarget_share/config/input_humandex.yaml"
  fi
fi
if [[ -n "$INPUT_CONFIG" && ! -f "$INPUT_CONFIG" ]]; then
  echo "[teleop_revo3] Input mapping not found: $INPUT_CONFIG" >&2; exit 1
fi

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
        local status=0
        wait "${pid}" 2>/dev/null || status=$?
        return "$status"
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

if [[ "$INPUT_SOURCE" == dv1 ]]; then
  sides=("$MODE")
  if [[ "$MODE" == both ]]; then sides=(left right); fi
  for side in "${sides[@]}"; do
    if [[ "$side" == left ]]; then
      joint_topic="$LEFT_JOINT_TOPIC"; output_topic="$LEFT_INPUT_TOPIC"
      config="${LEFT_ADAPTER_CONFIG:-$ADAPTER_CONFIG}"
      source_frame="$LEFT_SOURCE_FRAME_ID"
    else
      joint_topic="$RIGHT_JOINT_TOPIC"; output_topic="$RIGHT_INPUT_TOPIC"
      config="${RIGHT_ADAPTER_CONFIG:-$ADAPTER_CONFIG}"
      source_frame="$RIGHT_SOURCE_FRAME_ID"
    fi
    adapter_args=("hand_mode:=$side" "urdf_path:=$URDF_PATH"
                  "joint_topic:=$joint_topic" "output_topic:=$output_topic"
                  "joint_state_layout:=$JOINT_STATE_LAYOUT")
    if [[ -n "$source_frame" ]]; then adapter_args+=("source_frame_id:=$source_frame"); fi
    if [[ -n "$config" ]]; then adapter_args+=("adapter_config:=$config"); fi
    start_managed "DV1 ${side} adapter/FK" ros2 launch hand_input_adapters dv1_input.launch.py \
      "${adapter_args[@]}"
  done
  INPUT_SOURCE=external
fi
if [[ -n "$INPUT_CONFIG" ]]; then pipeline_args+=("input_config:=$INPUT_CONFIG"); fi

if [[ "${START_REVO3_DRIVER}" == "1" ]]; then
  start_managed "Revo3 driver" "${SCRIPT_DIR}/start_revo3_driver.sh" "${MODE}"
fi

if [[ "$RETARGET_BACKEND" == vector ]]; then
  publish_manus=false
  if [[ "$START_MANUS_PUBLISHER" == 1 ]]; then publish_manus=true; fi
  start_managed "Revo3 vector retarget" ros2 launch manus_revo3_vector_retarget pipeline_launch.py \
    "hand_mode:=$MODE" "launch_manus_publisher:=$publish_manus" "${pipeline_args[@]}"
else
  start_managed "Revo3 retarget" ros2 launch manus_revo3_retarget pipeline_launch.py \
    hand_mode:="${MODE}" \
    launch_manus_publisher:="${START_MANUS_PUBLISHER}" \
    "input_source:=$INPUT_SOURCE" \
    "${pipeline_args[@]}"
fi

wait_for_any
