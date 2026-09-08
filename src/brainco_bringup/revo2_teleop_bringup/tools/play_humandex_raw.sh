#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RETARGET_WS="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"

CONDA_ENV="retarget_revo3"
CONDA_SH="${CONDA_SH:-${HOME}/miniforge3/etc/profile.d/conda.sh}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
ROS_DOMAIN="${ROS_DOMAIN_ID:-11}"
STORAGE_ID="sqlite3"
JOINT_SOURCE_TOPIC="/joint_states"
POSE_SOURCE_TOPIC="/humandex_eef_pose"
JOINT_TARGET_TOPIC="/humandex_replay/joint_states"
POSE_TARGET_TOPIC="/humandex_replay/eef_pose"
RATE="1.0"
LOOP=0
START_PAUSED=0
ALLOW_NO_SUBSCRIBER=0
ALLOW_LIVE_CONFLICT=0
BAG_INPUT=""

usage() {
  cat <<'EOF'
Usage: play_humandex_raw.sh BAG_OR_RUN_DIR [options]

Replay the timestamp-paired HumanDex joint and fingertip topics. The default
remap keeps replay data separate from a live HumanDex publisher.

Options:
  --domain ID
  --rate FLOAT
  --storage ID
  --joint-target-topic TOPIC
  --pose-target-topic TOPIC
  --original-topics
  --loop
  --start-paused
  --allow-no-subscriber
  --allow-live-conflict
  --conda-env NAME
  --conda-sh PATH
  --retarget-ws PATH
  -h, --help

Start the read-only pipeline first:
  ros2 launch revo2_teleop_bringup teleop.launch.py \
    profile:=humandex_revo2 hand_mode:=left \
    humandex_joint_topic:=/humandex_replay/joint_states \
    humandex_pose_topic:=/humandex_replay/eef_pose \
    launch_revo2_driver:=false switch_controllers:=false
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --domain) ROS_DOMAIN="$2"; shift 2 ;;
    --rate) RATE="$2"; shift 2 ;;
    --storage) STORAGE_ID="$2"; shift 2 ;;
    --joint-target-topic) JOINT_TARGET_TOPIC="$2"; shift 2 ;;
    --pose-target-topic) POSE_TARGET_TOPIC="$2"; shift 2 ;;
    --original-topics)
      JOINT_TARGET_TOPIC="${JOINT_SOURCE_TOPIC}"
      POSE_TARGET_TOPIC="${POSE_SOURCE_TOPIC}"
      shift
      ;;
    --loop) LOOP=1; shift ;;
    --start-paused) START_PAUSED=1; shift ;;
    --allow-no-subscriber) ALLOW_NO_SUBSCRIBER=1; shift ;;
    --allow-live-conflict) ALLOW_LIVE_CONFLICT=1; shift ;;
    --conda-env) CONDA_ENV="$2"; shift 2 ;;
    --conda-sh) CONDA_SH="$2"; shift 2 ;;
    --retarget-ws) RETARGET_WS="$(cd "$2" && pwd)"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    -*) echo "[ERROR] Unknown argument: $1" >&2; usage >&2; exit 2 ;;
    *)
      if [[ -n "${BAG_INPUT}" ]]; then
        echo "[ERROR] Multiple bag paths provided." >&2
        exit 2
      fi
      BAG_INPUT="$1"
      shift
      ;;
  esac
done

if [[ -z "${BAG_INPUT}" ]]; then
  echo "[ERROR] Missing BAG_OR_RUN_DIR." >&2
  usage >&2
  exit 2
fi
if [[ -f "${BAG_INPUT}/metadata.yaml" ]]; then
  BAG_DIR="$(cd "${BAG_INPUT}" && pwd)"
elif [[ -f "${BAG_INPUT}/bag/metadata.yaml" ]]; then
  BAG_DIR="$(cd "${BAG_INPUT}/bag" && pwd)"
else
  echo "[ERROR] metadata.yaml not found under ${BAG_INPUT}." >&2
  exit 1
fi
if [[ ! -f "${CONDA_SH}" || ! -f "${ROS_SETUP}" ]]; then
  echo "[ERROR] ROS or conda setup file is missing." >&2
  exit 1
fi

set +u
source "${CONDA_SH}"
conda activate "${CONDA_ENV}"
source "${ROS_SETUP}"
if [[ -f "${RETARGET_WS}/install/setup.bash" ]]; then
  source "${RETARGET_WS}/install/setup.bash"
fi
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN}"
export ROS2CLI_DISABLE_DAEMON=1

topic_count() {
  local topic="$1"
  local label="$2"
  ros2 topic info "${topic}" 2>/dev/null | awk -F: -v label="${label}" '
    index($0, label ":") == 1 {gsub(/[[:space:]]/, "", $2); print $2; found=1; exit}
    END {if (!found) print 0}
  '
}

for topic in "${JOINT_TARGET_TOPIC}" "${POSE_TARGET_TOPIC}"; do
  publishers="$(topic_count "${topic}" "Publisher count")"
  subscribers="$(topic_count "${topic}" "Subscription count")"
  if [[ "${ALLOW_NO_SUBSCRIBER}" -eq 0 && "${subscribers}" -eq 0 ]]; then
    echo "[ERROR] Replay target has no subscriber: ${topic}" >&2
    exit 4
  fi
  if [[ "${ALLOW_LIVE_CONFLICT}" -eq 0 && "${publishers}" -gt 0 ]]; then
    echo "[ERROR] Replay target already has ${publishers} publisher(s): ${topic}" >&2
    exit 5
  fi
done

PLAY_CMD=(
  ros2 bag play "${BAG_DIR}"
  --storage "${STORAGE_ID}"
  --topics "${JOINT_SOURCE_TOPIC}" "${POSE_SOURCE_TOPIC}"
  --rate "${RATE}"
  --remap
  "${JOINT_SOURCE_TOPIC}:=${JOINT_TARGET_TOPIC}"
  "${POSE_SOURCE_TOPIC}:=${POSE_TARGET_TOPIC}"
)
if [[ "${LOOP}" -eq 1 ]]; then PLAY_CMD+=(--loop); fi
if [[ "${START_PAUSED}" -eq 1 ]]; then PLAY_CMD+=(--start-paused); fi

echo "[INFO] Bag: ${BAG_DIR}"
echo "[INFO] Joint replay: ${JOINT_SOURCE_TOPIC} -> ${JOINT_TARGET_TOPIC}"
echo "[INFO] Pose replay: ${POSE_SOURCE_TOPIC} -> ${POSE_TARGET_TOPIC}"
exec "${PLAY_CMD[@]}"
