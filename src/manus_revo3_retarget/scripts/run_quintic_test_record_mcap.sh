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
BAG_NAME="${BAG_NAME:-revo3_quintic_test_${TIMESTAMP}}"
RUN_DIR="${LOG_ROOT}/${BAG_NAME}"
BAG_DIR="${RUN_DIR}/mcap"
STARTUP_DELAY="${STARTUP_DELAY:-2}"

HAND_MODE="both"
NEXT_IS_HAND_MODE=0
for arg in "$@"; do
  if [[ "${NEXT_IS_HAND_MODE}" == "1" ]]; then
    HAND_MODE="${arg}"
    NEXT_IS_HAND_MODE=0
    continue
  fi
  case "${arg}" in
    --hand-mode=*)
      HAND_MODE="${arg#--hand-mode=}"
      ;;
    --hand-mode)
      NEXT_IS_HAND_MODE=1
      ;;
  esac
done

RECORD_TOPICS=(
  "/revo3_left/joint_forward_mit_controller/commands"
  "/revo3_right/joint_forward_mit_controller/commands"
  "/revo3_left/revo3_joint_state/joint_states_aligned"
  "/revo3_right/revo3_joint_state/joint_states_aligned"
)

if [[ "${HAND_MODE}" == "left" ]]; then
  RECORD_TOPICS=(
    "/revo3_left/joint_forward_mit_controller/commands"
    "/revo3_left/revo3_joint_state/joint_states_aligned"
  )
elif [[ "${HAND_MODE}" == "right" ]]; then
  RECORD_TOPICS=(
    "/revo3_right/joint_forward_mit_controller/commands"
    "/revo3_right/revo3_joint_state/joint_states_aligned"
  )
fi

mkdir -p "${RUN_DIR}"
BAG_LOG="${RUN_DIR}/rosbag_record.log"
ALIGNER_LOG="${RUN_DIR}/joint_state_aligner.log"
TEST_LOG="${RUN_DIR}/quintic_joint_test.log"

cleanup() {
  local status=$?
  trap - INT TERM EXIT
  echo
  echo "Stopping quintic test, rosbag, and aligner..."
  if [[ -n "${TEST_PID:-}" ]] && kill -0 "${TEST_PID}" 2>/dev/null; then
    kill -INT "${TEST_PID}" 2>/dev/null || true
  fi
  if [[ -n "${BAG_PID:-}" ]] && kill -0 "${BAG_PID}" 2>/dev/null; then
    kill -INT "${BAG_PID}" 2>/dev/null || true
  fi
  if [[ -n "${ALIGNER_PID:-}" ]] && kill -0 "${ALIGNER_PID}" 2>/dev/null; then
    kill -INT "${ALIGNER_PID}" 2>/dev/null || true
  fi
  sleep 2
  for pid in "${TEST_PID:-}" "${BAG_PID:-}" "${ALIGNER_PID:-}"; do
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      kill -TERM "${pid}" 2>/dev/null || true
    fi
  done
  wait "${TEST_PID:-}" 2>/dev/null || true
  wait "${BAG_PID:-}" 2>/dev/null || true
  wait "${ALIGNER_PID:-}" 2>/dev/null || true
  echo "Saved MCAP bag under: ${BAG_DIR}"
  echo "Saved process logs under: ${RUN_DIR}"
  exit "${status}"
}
trap cleanup INT TERM EXIT

ALIGNER_COMMAND=(ros2 run manus_revo3_retarget joint_state_aligner --hand-mode "${HAND_MODE}")
printf 'Starting:'
printf ' %q' "${ALIGNER_COMMAND[@]}"
printf '\n'
"${ALIGNER_COMMAND[@]}" \
  > >(tee -a "${ALIGNER_LOG}") \
  2> >(tee -a "${ALIGNER_LOG}" >&2) &
ALIGNER_PID=$!

sleep "${STARTUP_DELAY}"

BAG_COMMAND=(ros2 bag record -s mcap --include-unpublished-topics -o "${BAG_DIR}" "${RECORD_TOPICS[@]}")
printf 'Recording command:'
printf ' %q' "${BAG_COMMAND[@]}"
printf '\n'
"${BAG_COMMAND[@]}" \
  > >(tee -a "${BAG_LOG}") \
  2> >(tee -a "${BAG_LOG}" >&2) &
BAG_PID=$!

sleep "${STARTUP_DELAY}"

TEST_COMMAND=(ros2 run manus_revo3_retarget quintic_joint_test "$@")
printf 'Starting:'
printf ' %q' "${TEST_COMMAND[@]}"
printf '\n'
"${TEST_COMMAND[@]}" \
  > >(tee -a "${TEST_LOG}") \
  2> >(tee -a "${TEST_LOG}" >&2) &
TEST_PID=$!

wait "${TEST_PID}"
