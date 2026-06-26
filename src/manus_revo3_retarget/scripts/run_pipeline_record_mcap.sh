#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

find_workspace_root() {
  local dir="$1"
  while [[ "${dir}" != "/" ]]; do
    if [[ -f "${dir}/install/setup.bash" ]]; then
      printf '%s\n' "${dir}"
      return 0
    fi
    dir="$(dirname "${dir}")"
  done
  return 1
}

source_with_nounset_disabled() {
  set +u
  # shellcheck source=/dev/null
  source "$1"
  set -u
}

WORKSPACE_ROOT="${WORKSPACE_ROOT:-}"
if [[ -z "${WORKSPACE_ROOT}" ]]; then
  WORKSPACE_ROOT="$(find_workspace_root "${PACKAGE_DIR}" || true)"
fi

CONDA_ENV_NAME="${CONDA_ENV_NAME:-manusglove}"
if [[ "${CONDA_ENV_NAME}" != "0" && "${CONDA_ENV_NAME}" != "none" && "${CONDA_DEFAULT_ENV:-}" != "${CONDA_ENV_NAME}" ]]; then
  CONDA_SH=""
  for candidate in \
    "${HOME}/miniforge3/etc/profile.d/conda.sh" \
    "${HOME}/miniconda3/etc/profile.d/conda.sh" \
    "${HOME}/anaconda3/etc/profile.d/conda.sh"; do
    if [[ -f "${candidate}" ]]; then
      CONDA_SH="${candidate}"
      break
    fi
  done
  if [[ -z "${CONDA_SH}" ]]; then
    echo "CONDA_ENV_NAME=${CONDA_ENV_NAME}, but conda.sh was not found." >&2
    exit 1
  fi
  source_with_nounset_disabled "${CONDA_SH}"
  conda activate "${CONDA_ENV_NAME}"
fi

if [[ -f "/opt/ros/humble/setup.bash" ]]; then
  source_with_nounset_disabled "/opt/ros/humble/setup.bash"
fi

if [[ -n "${WORKSPACE_ROOT}" && -f "${WORKSPACE_ROOT}/install/setup.bash" ]]; then
  source_with_nounset_disabled "${WORKSPACE_ROOT}/install/setup.bash"
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 was not found. Source ROS 2 and this workspace first." >&2
  exit 1
fi

if ! ros2 bag record --help 2>&1 | grep -Eq -- '--storage \{[^}]*mcap|--storage.*mcap'; then
  cat >&2 <<'EOF'
ros2 bag storage plugin "mcap" is not available in this environment.
Install it, then rerun:

  sudo apt install ros-humble-rosbag2-storage-mcap

EOF
  exit 2
fi

TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
LOG_ROOT="${LOG_ROOT:-${PACKAGE_DIR}/log}"
BAG_NAME="${BAG_NAME:-manus_revo3_retarget_${TIMESTAMP}}"
RUN_DIR="${LOG_ROOT}/${BAG_NAME}"
BAG_DIR="${RUN_DIR}/mcap"
STARTUP_DELAY="${STARTUP_DELAY:-3}"

DEFAULT_TOPICS=(
  "/manus_glove_0"
  "/manus_glove_1"
  "/revo3_left/joint_forward_mit_controller/commands"
  "/revo3_right/joint_forward_mit_controller/commands"
  "/revo3_left/joint_forward_mit_controller/retarget_targets"
  "/revo3_right/joint_forward_mit_controller/retarget_targets"
  "/revo3_left/revo3_joint_state/joint_states_aligned"
  "/revo3_right/revo3_joint_state/joint_states_aligned"
)

if [[ "${RECORD_RAW_JOINT_STATES:-0}" == "1" ]]; then
  DEFAULT_TOPICS+=(
    "/revo3_left/revo3_joint_state/joint_states"
    "/revo3_right/revo3_joint_state/joint_states"
  )
fi

if [[ -n "${BAG_TOPICS:-}" ]]; then
  read -r -a RECORD_TOPICS <<<"${BAG_TOPICS}"
else
  RECORD_TOPICS=("${DEFAULT_TOPICS[@]}")
fi

mkdir -p "${RUN_DIR}"
LAUNCH_LOG="${RUN_DIR}/pipeline_launch.log"
BAG_LOG="${RUN_DIR}/rosbag_record.log"
ALIGNER_LOG="${RUN_DIR}/joint_state_aligner.log"

LAUNCH_ARGS=("$@")
if [[ "${#LAUNCH_ARGS[@]}" == "0" ]]; then
  LAUNCH_ARGS=("hand_mode:=both")
fi

echo "Log directory: ${RUN_DIR}"
echo "MCAP bag directory: ${BAG_DIR}"
printf 'Starting: ros2 launch manus_revo3_retarget pipeline_launch.py'
printf ' %q' "${LAUNCH_ARGS[@]}"
printf '\n'

ros2 launch manus_revo3_retarget pipeline_launch.py "${LAUNCH_ARGS[@]}" \
  > >(tee -a "${LAUNCH_LOG}") \
  2> >(tee -a "${LAUNCH_LOG}" >&2) &
LAUNCH_PID=$!

ALIGNER_PID=""
if [[ "${ENABLE_JOINT_STATE_ALIGNER:-1}" == "1" ]]; then
  ALIGNER_COMMAND=(ros2 run manus_revo3_retarget joint_state_aligner)
  for launch_arg in "${LAUNCH_ARGS[@]}"; do
    case "${launch_arg}" in
      hand_mode:=*)
        ALIGNER_COMMAND+=(--hand-mode "${launch_arg#hand_mode:=}")
        ;;
      hand_type:=*)
        ALIGNER_COMMAND+=(--hand-mode "${launch_arg#hand_type:=}")
        ;;
      use_revo3_namespace:=*)
        ALIGNER_COMMAND+=(--use-revo3-namespace "${launch_arg#use_revo3_namespace:=}")
        ;;
      command_topic_suffix:=*)
        ALIGNER_COMMAND+=(--command-topic-suffix "${launch_arg#command_topic_suffix:=}")
        ;;
    esac
  done
  if [[ -n "${ALIGNER_ARGS:-}" ]]; then
    read -r -a EXTRA_ALIGNER_ARGS <<<"${ALIGNER_ARGS}"
    ALIGNER_COMMAND+=("${EXTRA_ALIGNER_ARGS[@]}")
  fi
  printf 'Starting:'
  printf ' %q' "${ALIGNER_COMMAND[@]}"
  printf '\n'
  "${ALIGNER_COMMAND[@]}" \
    > >(tee -a "${ALIGNER_LOG}") \
    2> >(tee -a "${ALIGNER_LOG}" >&2) &
  ALIGNER_PID=$!
fi

cleanup() {
  local status=$?
  trap - INT TERM EXIT
  echo
  echo "Stopping rosbag and pipeline..."
  if [[ -n "${BAG_PID:-}" ]] && kill -0 "${BAG_PID}" 2>/dev/null; then
    kill -INT "${BAG_PID}" 2>/dev/null || true
  fi
  if [[ -n "${LAUNCH_PID:-}" ]] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill -INT "${LAUNCH_PID}" 2>/dev/null || true
  fi
  if [[ -n "${ALIGNER_PID:-}" ]] && kill -0 "${ALIGNER_PID}" 2>/dev/null; then
    kill -INT "${ALIGNER_PID}" 2>/dev/null || true
  fi
  sleep 2
  if [[ -n "${BAG_PID:-}" ]] && kill -0 "${BAG_PID}" 2>/dev/null; then
    kill -TERM "${BAG_PID}" 2>/dev/null || true
  fi
  if [[ -n "${LAUNCH_PID:-}" ]] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill -TERM "${LAUNCH_PID}" 2>/dev/null || true
  fi
  if [[ -n "${ALIGNER_PID:-}" ]] && kill -0 "${ALIGNER_PID}" 2>/dev/null; then
    kill -TERM "${ALIGNER_PID}" 2>/dev/null || true
  fi
  wait "${BAG_PID:-}" 2>/dev/null || true
  wait "${LAUNCH_PID:-}" 2>/dev/null || true
  wait "${ALIGNER_PID:-}" 2>/dev/null || true
  if [[ -n "${BAG_PID:-}" ]]; then
    echo "Saved MCAP bag under: ${BAG_DIR}"
  else
    echo "MCAP recording did not start."
  fi
  echo "Saved process logs under: ${RUN_DIR}"
  exit "${status}"
}
trap cleanup INT TERM EXIT

sleep "${STARTUP_DELAY}"

if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
  echo "Pipeline launch exited before rosbag recording started. See: ${LAUNCH_LOG}" >&2
  wait "${LAUNCH_PID}" 2>/dev/null || true
  exit 1
fi

if [[ -n "${ALIGNER_PID:-}" ]] && ! kill -0 "${ALIGNER_PID}" 2>/dev/null; then
  echo "Joint state aligner exited before rosbag recording started. See: ${ALIGNER_LOG}" >&2
  wait "${ALIGNER_PID}" 2>/dev/null || true
  exit 1
fi

if [[ "${RECORD_ALL:-0}" == "1" ]]; then
  BAG_COMMAND=(ros2 bag record -s mcap -o "${BAG_DIR}" --all)
else
  BAG_COMMAND=(ros2 bag record -s mcap --include-unpublished-topics -o "${BAG_DIR}" "${RECORD_TOPICS[@]}")
fi

printf 'Recording command:'
printf ' %q' "${BAG_COMMAND[@]}"
printf '\n'

"${BAG_COMMAND[@]}" \
  > >(tee -a "${BAG_LOG}") \
  2> >(tee -a "${BAG_LOG}" >&2) &
BAG_PID=$!

if [[ -n "${ALIGNER_PID:-}" ]]; then
  wait -n "${LAUNCH_PID}" "${BAG_PID}" "${ALIGNER_PID}"
else
  wait -n "${LAUNCH_PID}" "${BAG_PID}"
fi
