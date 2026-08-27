#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RETARGET_WS="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"
HUMANDEX_WS="$(cd "${RETARGET_WS}/.." && pwd)/BrainCo-HumanDex"

CONDA_ENV="retarget_revo3"
CONDA_SH="${CONDA_SH:-${HOME}/miniforge3/etc/profile.d/conda.sh}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
SIDE="right"
DURATION="60"
ROS_DOMAIN="${ROS_DOMAIN_ID:-11}"
OUTPUT_ROOT="${RETARGET_WS}/artifacts/teleop_records"
STORAGE_ID="sqlite3"
ALL_TOPICS=0
ALLOW_MISSING_TOPICS=0
EXTRA_TOPICS=()

usage() {
  cat <<'EOF'
Usage:
  record_humandex_teleop.sh [options]

Options:
  --side left|right|both      Hand side to record. Default: right
  --duration SEC             Recording duration passed to timeout. Default: 60
  --domain ID                ROS_DOMAIN_ID to use. Default: current env or 11
  --out-dir DIR              Parent output directory. Default: artifacts/teleop_records
  --storage sqlite3          rosbag2 storage id. Default: sqlite3
  --conda-env NAME           Conda environment. Default: retarget_revo3
  --conda-sh PATH            conda.sh path. Default: ~/miniforge3/etc/profile.d/conda.sh
  --retarget-ws PATH         Revo-Retargeting workspace. Default: auto-detected
  --humandex-ws PATH         BrainCo-HumanDex workspace. Default: sibling repo
  --all-topics               Record every currently/future discovered topic
  --allow-missing-topics     Record even if core teleop topics are not present
  --extra-topic TOPIC        Add one extra topic. Can be repeated
  -h, --help                 Show this help

Example:
  ./src/brainco_capabilities/manus_revo2_retarget/tools/record_humandex_teleop.sh \
    --side left --duration 90 --domain 11
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
    --all-topics)
      ALL_TOPICS=1
      shift
      ;;
    --allow-missing-topics)
      ALLOW_MISSING_TOPICS=1
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
  echo "        Pass --conda-sh or set CONDA_SH." >&2
  exit 1
fi
if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "[ERROR] ROS setup not found: ${ROS_SETUP}" >&2
  echo "        Pass ROS_SETUP=/path/to/setup.bash if needed." >&2
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
else
  echo "[WARN] Revo-Retargeting install setup not found: ${RETARGET_SETUP}" >&2
  echo "       Build/source it before recording if /manus_glove_* types are unavailable." >&2
fi
if [[ -f "${HUMANDEX_SETUP}" ]]; then
  source "${HUMANDEX_SETUP}"
else
  echo "[INFO] BrainCo-HumanDex install setup not found: ${HUMANDEX_SETUP}" >&2
  echo "       This is OK for recording if the HumanDex nodes are already running." >&2
fi
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN}"
export ROS2CLI_DISABLE_DAEMON=1

if [[ "${SIDE}" == "both" ]]; then
  SIDES=("left" "right")
else
  SIDES=("${SIDE}")
fi

RUN_ID="$(date +%Y%m%d_%H%M%S)_humandex_${SIDE}_domain${ROS_DOMAIN}"
RUN_DIR="${OUTPUT_ROOT}/${RUN_ID}"
BAG_DIR="${RUN_DIR}/bag"
DIAG_DIR="${RUN_DIR}/diagnostics"
mkdir -p "${DIAG_DIR}"

TOPICS=(
  /rosout
  /tf
  /tf_static
  /joint_states
  /humandex_eef_pose
)

for hand in "${SIDES[@]}"; do
  glove_index=1
  if [[ "${hand}" == "left" ]]; then
    glove_index=0
  fi
  TOPICS+=(
    "/humandex_${hand}/joint_states"
    "/humandex_${hand}/tactile"
    "/manus_glove_${glove_index}"
    "/revo2_${hand}/retarget/joint_states"
    "/revo2_${hand}/revo2_pid_controller/target_joint_states"
    "/revo2_${hand}/revo2_joint_state/joint_states"
    "/revo2_${hand}/revo2_joint_state/dynamic_joint_states"
    "/revo2_${hand}/joint_forward_pos_controller/commands"
    "/revo2_${hand}/joint_forward_vel_controller/commands"
  )
done
TOPICS+=("${EXTRA_TOPICS[@]}")

REQUIRED_TOPICS=(/humandex_eef_pose)
for hand in "${SIDES[@]}"; do
  glove_index=1
  if [[ "${hand}" == "left" ]]; then
    glove_index=0
  fi
  REQUIRED_TOPICS+=(
    "/humandex_${hand}/joint_states"
    "/manus_glove_${glove_index}"
    "/revo2_${hand}/revo2_pid_controller/target_joint_states"
    "/revo2_${hand}/revo2_joint_state/joint_states"
  )
done

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
  local rc=$?
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
  ros2 --help >/dev/null 2>&1 && echo "ros2_ok=true" || echo "ros2_ok=false"
  git -C "${RETARGET_WS}" rev-parse HEAD 2>/dev/null | sed 's/^/retarget_git_head=/'
  git -C "${HUMANDEX_WS}" rev-parse HEAD 2>/dev/null | sed 's/^/humandex_git_head=/'
} >"${RUN_DIR}/environment.txt"

git -C "${RETARGET_WS}" status --short >"${RUN_DIR}/retarget_git_status.txt" 2>&1 || true
git -C "${HUMANDEX_WS}" status --short >"${RUN_DIR}/humandex_git_status.txt" 2>&1 || true
printf '%s\n' "${TOPICS[@]}" >"${RUN_DIR}/topics_to_record.txt"

cat >"${RUN_DIR}/recording_protocol.txt" <<EOF
Suggested 60-90s recording protocol:
  0-10s   Keep the glove still and the Revo2 hand open.
  10-30s  Slowly close and open all fingers together twice.
  30-50s  Move one finger at a time if possible.
  50-60s  Return to still/open and hold.

Keep the hand in view and avoid intentionally unplugging/restarting nodes during the recording.
EOF

run_capture "${DIAG_DIR}/topic_list_before.txt" ros2 topic list -t
run_capture "${DIAG_DIR}/node_list_before.txt" ros2 node list
run_capture "${DIAG_DIR}/service_list_before.txt" ros2 service list

declare -A AVAILABLE_TOPICS=()
while read -r topic_name _rest; do
  if [[ -n "${topic_name}" ]]; then
    AVAILABLE_TOPICS["${topic_name}"]=1
  fi
done <"${DIAG_DIR}/topic_list_before.txt"

MISSING_TOPICS=()
for topic in "${REQUIRED_TOPICS[@]}"; do
  if [[ -z "${AVAILABLE_TOPICS[${topic}]+x}" ]]; then
    MISSING_TOPICS+=("${topic}")
  fi
done
if [[ "${#MISSING_TOPICS[@]}" -gt 0 && "${ALLOW_MISSING_TOPICS}" -eq 0 ]]; then
  echo "[ERROR] Required teleop topics are not present in ROS_DOMAIN_ID=${ROS_DOMAIN_ID}:" >&2
  printf '  %s\n' "${MISSING_TOPICS[@]}" >&2
  echo "[ERROR] Start/source BrainCo-HumanDex and the Revo-Retargeting teleop pipeline first." >&2
  echo "        If you intentionally want to record before publishers appear, rerun with --allow-missing-topics." >&2
  echo "        Diagnostics were written to: ${RUN_DIR}" >&2
  exit 3
fi

RECORD_TOPICS=()
SKIPPED_TOPICS=()
for topic in "${TOPICS[@]}"; do
  if [[ -n "${AVAILABLE_TOPICS[${topic}]+x}" || "${ALLOW_MISSING_TOPICS}" -eq 1 ]]; then
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

for hand in "${SIDES[@]}"; do
  run_capture \
    "${DIAG_DIR}/controllers_${hand}_before.txt" \
    ros2 control list_controllers -c "/revo2_${hand}/controller_manager"
done

for topic in "${TOPICS[@]}"; do
  topic_file="$(sanitize_name "${topic}")"
  run_capture "${DIAG_DIR}/topic_info_${topic_file}.txt" ros2 topic info -v "${topic}"
done

echo "[INFO] Output directory: ${RUN_DIR}"
echo "[INFO] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "[INFO] Recording duration: ${DURATION}s"
echo "[INFO] Topics: ${#TOPICS[@]} selected"
echo "[INFO] Suggested motion protocol is in: ${RUN_DIR}/recording_protocol.txt"

if [[ "${ALL_TOPICS}" -eq 1 ]]; then
  RECORD_CMD=(ros2 bag record --all --storage "${STORAGE_ID}" --max-cache-size 268435456 -o "${BAG_DIR}")
else
  RECORD_CMD=(ros2 bag record --storage "${STORAGE_ID}" --max-cache-size 268435456 -o "${BAG_DIR}" "${RECORD_TOPICS[@]}")
fi

printf '%q ' "${RECORD_CMD[@]}" >"${RUN_DIR}/record_command.txt"
printf '\n' >>"${RUN_DIR}/record_command.txt"

set +e
"${RECORD_CMD[@]}" >"${RUN_DIR}/record_stdout.log" 2>&1 &
record_pid=$!
set -e

sleep "${DURATION}"

echo "[INFO] Stopping ros2 bag recorder..."
if kill -0 "${record_pid}" 2>/dev/null; then
  kill -INT "${record_pid}" 2>/dev/null || true
fi
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

if [[ "${record_rc}" -ne 0 && "${record_rc}" -ne 130 && "${record_rc}" -ne 143 ]]; then
  echo "[ERROR] ros2 bag record failed with exit code ${record_rc}" >&2
  exit "${record_rc}"
fi

run_capture "${DIAG_DIR}/topic_list_after.txt" ros2 topic list -t
for hand in "${SIDES[@]}"; do
  run_capture \
    "${DIAG_DIR}/controllers_${hand}_after.txt" \
    ros2 control list_controllers -c "/revo2_${hand}/controller_manager"
done
run_capture "${RUN_DIR}/bag_info.txt" ros2 bag info "${BAG_DIR}"

{
  echo "Analyze target/actual timing with:"
  for hand in "${SIDES[@]}"; do
    raw_topic="/humandex_${hand}/joint_states"
    glove_topic="/manus_glove_1"
    if [[ "${hand}" == "left" ]]; then
      glove_topic="/manus_glove_0"
    fi
    echo "python ${RETARGET_WS}/src/brainco_capabilities/manus_revo2_retarget/tools/analyze_revo2_jitter_bag.py ${BAG_DIR} --storage-id ${STORAGE_ID} --side ${hand} --glove-topic ${glove_topic} --raw-angles-topic ${raw_topic} --raw-positions-topic /humandex_eef_pose"
  done
} >"${RUN_DIR}/analysis_commands.txt"

echo "[INFO] Recording complete."
echo "[INFO] Send me this path for analysis:"
echo "${RUN_DIR}"
