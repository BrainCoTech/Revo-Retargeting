#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RETARGET_WS="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"
HUMANDEX_WS="$(cd "${RETARGET_WS}/.." && pwd)/BrainCo-HumanDex"

CONDA_ENV="retarget_revo3"
CONDA_SH="${CONDA_SH:-${HOME}/miniforge3/etc/profile.d/conda.sh}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
SIDE="left"
DURATION="60"
ROS_DOMAIN="${ROS_DOMAIN_ID:-11}"
OUTPUT_ROOT="${RETARGET_WS}/artifacts/humandex_raw_records"
STORAGE_ID="sqlite3"
REQUIRE_EEF=0
ALLOW_MISSING_REQUIRED=0
EXTRA_TOPICS=()

usage() {
  cat <<'EOF'
Usage:
  record_humandex_raw.sh [options]

Record only HumanDex-origin ROS topics. This does not require the Revo2 retarget
pipeline, hand input adapters, or Revo2 controllers to be running.

Options:
  --side left|right|both      HumanDex side to record. Default: left
  --duration SEC             Recording duration. Default: 60
  --domain ID                ROS_DOMAIN_ID to use. Default: current env or 11
  --out-dir DIR              Parent output directory. Default: artifacts/humandex_raw_records
  --storage sqlite3          rosbag2 storage id. Default: sqlite3
  --conda-env NAME           Conda environment. Default: retarget_revo3
  --conda-sh PATH            conda.sh path. Default: ~/miniforge3/etc/profile.d/conda.sh
  --retarget-ws PATH         Revo-Retargeting workspace. Default: auto-detected
  --humandex-ws PATH         BrainCo-HumanDex workspace. Default: sibling repo
  --require-eef              Also require /humandex_eef_pose to exist before recording
  --allow-missing-required   Record even if required HumanDex topics are missing
  --extra-topic TOPIC        Add one extra topic. Can be repeated
  -h, --help                 Show this help

Example:
  ./src/brainco_bringup/revo2_teleop_bringup/tools/record_humandex_raw.sh \
    --side left --duration 60 --domain 11
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --side)
      SIDE="$2"
      shift 2
      ;;
    --duration)
      DURATION="$2"
      shift 2
      ;;
    --domain)
      ROS_DOMAIN="$2"
      shift 2
      ;;
    --out-dir)
      OUTPUT_ROOT="$2"
      shift 2
      ;;
    --storage)
      STORAGE_ID="$2"
      shift 2
      ;;
    --conda-env)
      CONDA_ENV="$2"
      shift 2
      ;;
    --conda-sh)
      CONDA_SH="$2"
      shift 2
      ;;
    --retarget-ws)
      RETARGET_WS="$(cd "$2" && pwd)"
      shift 2
      ;;
    --humandex-ws)
      HUMANDEX_WS="$(cd "$2" && pwd)"
      shift 2
      ;;
    --require-eef)
      REQUIRE_EEF=1
      shift
      ;;
    --allow-missing-required)
      ALLOW_MISSING_REQUIRED=1
      shift
      ;;
    --extra-topic)
      EXTRA_TOPICS+=("$2")
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[ERROR] Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ "${SIDE}" != "left" && "${SIDE}" != "right" && "${SIDE}" != "both" ]]; then
  echo "[ERROR] --side must be left, right, or both" >&2
  exit 2
fi
if [[ ! -f "${CONDA_SH}" ]]; then
  echo "[ERROR] conda.sh not found: ${CONDA_SH}" >&2
  exit 1
fi
if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "[ERROR] ROS setup not found: ${ROS_SETUP}" >&2
  exit 1
fi

set +u
source "${CONDA_SH}"
conda activate "${CONDA_ENV}"
source "${ROS_SETUP}"

RETARGET_SETUP="${RETARGET_WS}/install/setup.bash"
HUMANDEX_SETUP="${HUMANDEX_WS}/install/setup.bash"
if [[ -f "${RETARGET_SETUP}" ]]; then
  source "${RETARGET_SETUP}"
fi
if [[ -f "${HUMANDEX_SETUP}" ]]; then
  source "${HUMANDEX_SETUP}"
else
  echo "[INFO] BrainCo-HumanDex install setup not found: ${HUMANDEX_SETUP}" >&2
  echo "       This is OK for raw recording if HumanDex nodes are already running." >&2
fi
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN}"
export ROS2CLI_DISABLE_DAEMON=1

if [[ "${SIDE}" == "both" ]]; then
  SIDES=("left" "right")
else
  SIDES=("${SIDE}")
fi

RUN_ID="$(date +%Y%m%d_%H%M%S)_humandex_raw_${SIDE}_domain${ROS_DOMAIN}"
RUN_DIR="${OUTPUT_ROOT}/${RUN_ID}"
BAG_DIR="${RUN_DIR}/bag"
DIAG_DIR="${RUN_DIR}/diagnostics"
mkdir -p "${DIAG_DIR}"

TOPICS=(/rosout /joint_states /humandex_eef_pose)
REQUIRED_TOPICS=()
if [[ "${REQUIRE_EEF}" -eq 1 ]]; then
  REQUIRED_TOPICS+=(/humandex_eef_pose)
fi

for hand in "${SIDES[@]}"; do
  TOPICS+=(
    "/humandex_${hand}/joint_states"
    "/humandex_${hand}/tactile"
  )
  REQUIRED_TOPICS+=("/humandex_${hand}/joint_states")
done
TOPICS+=("${EXTRA_TOPICS[@]}")

sanitize_name() {
  local value="$1"
  value="${value#/}"
  value="${value//\//_}"
  value="${value//[^A-Za-z0-9_.-]/_}"
  printf '%s' "${value:-root}"
}

run_capture() {
  local output_file="$1"
  shift
  set +e
  timeout 8 "$@" >"${output_file}" 2>&1
  set -e
  return 0
}

{
  echo "run_id=${RUN_ID}"
  echo "date=$(date --iso-8601=seconds)"
  echo "host=$(hostname)"
  echo "side=${SIDE}"
  echo "duration=${DURATION}"
  echo "storage_id=${STORAGE_ID}"
  echo "ros_domain_id=${ROS_DOMAIN_ID}"
  echo "retarget_ws=${RETARGET_WS}"
  echo "humandex_ws=${HUMANDEX_WS}"
  echo "conda_env=${CONDA_ENV}"
  echo "python=$(command -v python)"
  python -V
  echo "ros2=$(command -v ros2)"
  git -C "${RETARGET_WS}" rev-parse HEAD 2>/dev/null | sed 's/^/retarget_git_head=/'
  git -C "${HUMANDEX_WS}" rev-parse HEAD 2>/dev/null | sed 's/^/humandex_git_head=/'
} >"${RUN_DIR}/environment.txt"

git -C "${RETARGET_WS}" status --short >"${RUN_DIR}/retarget_git_status.txt" 2>&1 || true
git -C "${HUMANDEX_WS}" status --short >"${RUN_DIR}/humandex_git_status.txt" 2>&1 || true
printf '%s\n' "${TOPICS[@]}" >"${RUN_DIR}/topics_requested.txt"

cat >"${RUN_DIR}/recording_protocol.txt" <<EOF
Suggested 60s HumanDex raw recording protocol:
  0-10s   Keep the glove still.
  10-30s  Slowly close and open all fingers twice.
  30-45s  Move one finger at a time if possible.
  45-60s  Return to still/open and hold.

This raw recording does not require an input adapter, retargeter, or Revo2 controller.
EOF

run_capture "${DIAG_DIR}/topic_list_before.txt" ros2 topic list -t
run_capture "${DIAG_DIR}/node_list_before.txt" ros2 node list

declare -A AVAILABLE_TOPICS=()
while read -r topic_name _rest; do
  if [[ -n "${topic_name}" ]]; then
    AVAILABLE_TOPICS["${topic_name}"]=1
  fi
done <"${DIAG_DIR}/topic_list_before.txt"

MISSING_REQUIRED=()
for topic in "${REQUIRED_TOPICS[@]}"; do
  if [[ -z "${AVAILABLE_TOPICS[${topic}]+x}" ]]; then
    MISSING_REQUIRED+=("${topic}")
  fi
done
if [[ "${#MISSING_REQUIRED[@]}" -gt 0 && "${ALLOW_MISSING_REQUIRED}" -eq 0 ]]; then
  echo "[ERROR] Required HumanDex topics are not present in ROS_DOMAIN_ID=${ROS_DOMAIN_ID}:" >&2
  printf '  %s\n' "${MISSING_REQUIRED[@]}" >&2
  echo "[ERROR] Start the BrainCo-HumanDex acquisition nodes first, then rerun this script." >&2
  echo "        Diagnostics were written to: ${RUN_DIR}" >&2
  exit 3
fi

RECORD_TOPICS=()
SKIPPED_TOPICS=()
for topic in "${TOPICS[@]}"; do
  if [[ -n "${AVAILABLE_TOPICS[${topic}]+x}" || "${ALLOW_MISSING_REQUIRED}" -eq 1 ]]; then
    RECORD_TOPICS+=("${topic}")
  else
    SKIPPED_TOPICS+=("${topic}")
  fi
done
printf '%s\n' "${RECORD_TOPICS[@]}" >"${RUN_DIR}/topics_recorded_actual.txt"
if [[ "${#SKIPPED_TOPICS[@]}" -gt 0 ]]; then
  {
    echo "Skipped missing optional topics:"
    printf '  %s\n' "${SKIPPED_TOPICS[@]}"
  } >"${RUN_DIR}/topics_skipped_missing.txt"
fi

for topic in "${TOPICS[@]}"; do
  topic_file="$(sanitize_name "${topic}")"
  run_capture "${DIAG_DIR}/topic_info_${topic_file}.txt" ros2 topic info -v "${topic}"
done
for topic in "${RECORD_TOPICS[@]}"; do
  topic_file="$(sanitize_name "${topic}")"
  run_capture "${DIAG_DIR}/topic_echo_once_${topic_file}.txt" ros2 topic echo "${topic}" --once
  run_capture "${DIAG_DIR}/topic_hz_${topic_file}.txt" ros2 topic hz "${topic}"
done

echo "[INFO] Output directory: ${RUN_DIR}"
echo "[INFO] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "[INFO] Recording duration: ${DURATION}s"
echo "[INFO] Recording ${#RECORD_TOPICS[@]} HumanDex topic(s)."
if [[ "${#SKIPPED_TOPICS[@]}" -gt 0 ]]; then
  echo "[INFO] Some optional topics are missing; see ${RUN_DIR}/topics_skipped_missing.txt"
fi
echo "[INFO] Motion protocol: ${RUN_DIR}/recording_protocol.txt"

RECORD_CMD=(ros2 bag record --storage "${STORAGE_ID}" --max-cache-size 268435456 -o "${BAG_DIR}" "${RECORD_TOPICS[@]}")
printf '%q ' "${RECORD_CMD[@]}" >"${RUN_DIR}/record_command.txt"
printf '\n' >>"${RUN_DIR}/record_command.txt"

set +e
"${RECORD_CMD[@]}" >"${RUN_DIR}/record_stdout.log" 2>&1 &
record_pid=$!
set -e

stop_recorder() {
  if [[ -n "${record_pid:-}" ]] && kill -0 "${record_pid}" 2>/dev/null; then
    kill -INT "${record_pid}" 2>/dev/null || true
  fi
}

handle_record_signal() {
  echo "[INFO] Recording interrupted; stopping ros2 bag recorder..." >&2
  stop_recorder
  wait "${record_pid}" 2>/dev/null || true
  exit 130
}

trap handle_record_signal INT TERM

# Starting the ros2 process is not the same as recording data.  In practice,
# discovery and SQLite setup take about half a second.  Start the requested
# duration only after rosbag2 confirms that all subscriptions are active.
recorder_ready=0
for _ in {1..300}; do
  if grep -Fq "[rosbag2_recorder]: Recording..." "${RUN_DIR}/record_stdout.log"; then
    recorder_ready=1
    break
  fi
  if ! kill -0 "${record_pid}" 2>/dev/null; then
    break
  fi
  sleep 0.05
done
if [[ "${recorder_ready}" -ne 1 ]]; then
  echo "[ERROR] ros2 bag recorder did not enter Recording state within 15 seconds." >&2
  stop_recorder
  wait "${record_pid}" 2>/dev/null || true
  tail -40 "${RUN_DIR}/record_stdout.log" >&2 || true
  trap - INT TERM
  exit 4
fi

record_timer_start_ns="$(date +%s%N)"
echo "[INFO] Recorder is ready; starting the ${DURATION}s capture timer."
sleep "${DURATION}"
record_timer_stop_ns="$(date +%s%N)"

echo "[INFO] Stopping ros2 bag recorder..."
stop_recorder
for _ in {1..100}; do
  if ! kill -0 "${record_pid}" 2>/dev/null; then
    break
  fi
  sleep 0.1
done
if kill -0 "${record_pid}" 2>/dev/null; then
  echo "[WARN] ros2 bag recorder did not stop after SIGINT; sending SIGTERM." >&2
  kill -TERM "${record_pid}" 2>/dev/null || true
fi

set +e
wait "${record_pid}"
record_rc=$?
set -e
trap - INT TERM

awk -v requested="${DURATION}" -v start_ns="${record_timer_start_ns}" \
  -v stop_ns="${record_timer_stop_ns}" \
  'BEGIN {printf "requested_duration_sec=%s\ntimer_wall_duration_sec=%.6f\n", requested, (stop_ns - start_ns) / 1000000000}' \
  >"${RUN_DIR}/recording_timing.txt"

if [[ "${record_rc}" -ne 0 && "${record_rc}" -ne 130 && "${record_rc}" -ne 143 ]]; then
  echo "[ERROR] ros2 bag record failed with exit code ${record_rc}" >&2
  exit "${record_rc}"
fi

run_capture "${DIAG_DIR}/topic_list_after.txt" ros2 topic list -t
run_capture "${RUN_DIR}/bag_info.txt" ros2 bag info "${BAG_DIR}"
bag_duration_ns="$(
  awk '$1 == "duration:" {getline; if ($1 == "nanoseconds:") print $2; exit}' \
    "${BAG_DIR}/metadata.yaml"
)"
if [[ "${bag_duration_ns}" =~ ^[0-9]+$ ]]; then
  awk -v duration_ns="${bag_duration_ns}" \
    'BEGIN {printf "bag_duration_sec=%.9f\n", duration_ns / 1000000000}' \
    >>"${RUN_DIR}/recording_timing.txt"
  awk -v duration_ns="${bag_duration_ns}" \
    'BEGIN {printf "[INFO] Recorded bag timestamp span: %.3fs\n", duration_ns / 1000000000}'
fi

echo "[INFO] HumanDex raw recording complete."
echo "[INFO] Send me this path for analysis:"
echo "${RUN_DIR}"
