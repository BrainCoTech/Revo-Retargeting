#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RETARGET_WS="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"

CONDA_ENV="retarget_revo3"
CONDA_SH="${CONDA_SH:-${HOME}/miniforge3/etc/profile.d/conda.sh}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
ROS_DOMAIN="${ROS_DOMAIN_ID:-11}"
STORAGE_ID="sqlite3"
SOURCE_TOPIC="/humandex_eef_pose"
TARGET_TOPIC="/humandex_replay/eef_pose"
RATE="1.0"
LOOP=0
START_PAUSED=0
ALLOW_NO_SUBSCRIBER=0
ALLOW_LIVE_CONFLICT=0
BAG_INPUT=""

usage() {
  cat <<'EOF'
Usage:
  play_humandex_raw.sh BAG_OR_RUN_DIR [options]

Replay a raw HumanDex recording into the retarget pipeline.

By default this script replays only /humandex_eef_pose and remaps it to
/humandex_replay/eef_pose. This avoids mixing recorded poses with a live
HumanDex publisher on /humandex_eef_pose.

Options:
  --domain ID                ROS_DOMAIN_ID to use. Default: current env or 11
  --rate FLOAT               Playback rate. Default: 1.0
  --storage sqlite3          rosbag2 storage id. Default: sqlite3
  --source-topic TOPIC       Recorded topic to play. Default: /humandex_eef_pose
  --target-topic TOPIC       Published topic after remap. Default: /humandex_replay/eef_pose
  --original-topic           Publish back to the original topic; requires --allow-live-conflict if a publisher exists
  --loop                     Loop playback
  --start-paused             Start paused
  --allow-no-subscriber      Play even if the target topic currently has no subscribers
  --allow-live-conflict      Play even if the target topic already has publishers
  --conda-env NAME           Conda environment. Default: retarget_revo3
  --conda-sh PATH            conda.sh path. Default: ~/miniforge3/etc/profile.d/conda.sh
  --retarget-ws PATH         Revo-Retargeting workspace. Default: auto-detected
  -h, --help                 Show this help

Expected safe setup:
  ros2 launch manus_revo2_retarget humandex_real_hand_pipeline_launch.py \
    hand_mode:=left \
    eef_pose_topic:=/humandex_replay/eef_pose

Example:
  ./src/brainco_capabilities/manus_revo2_retarget/tools/play_humandex_raw.sh \
    artifacts/humandex_raw_records/20260826_210122_humandex_raw_left_domain11 \
    --domain 11
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --domain)
      ROS_DOMAIN="$2"
      shift 2
      ;;
    --rate)
      RATE="$2"
      shift 2
      ;;
    --storage)
      STORAGE_ID="$2"
      shift 2
      ;;
    --source-topic)
      SOURCE_TOPIC="$2"
      shift 2
      ;;
    --target-topic)
      TARGET_TOPIC="$2"
      shift 2
      ;;
    --original-topic)
      TARGET_TOPIC="${SOURCE_TOPIC}"
      shift
      ;;
    --loop)
      LOOP=1
      shift
      ;;
    --start-paused)
      START_PAUSED=1
      shift
      ;;
    --allow-no-subscriber)
      ALLOW_NO_SUBSCRIBER=1
      shift
      ;;
    --allow-live-conflict)
      ALLOW_LIVE_CONFLICT=1
      shift
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
    -h|--help)
      usage
      exit 0
      ;;
    -*)
      echo "[ERROR] Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
    *)
      if [[ -n "${BAG_INPUT}" ]]; then
        echo "[ERROR] Multiple bag paths provided: ${BAG_INPUT} and $1" >&2
        usage >&2
        exit 2
      fi
      BAG_INPUT="$1"
      shift
      ;;
  esac
done

if [[ -z "${BAG_INPUT}" ]]; then
  echo "[ERROR] Missing BAG_OR_RUN_DIR" >&2
  usage >&2
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

if [[ -f "${BAG_INPUT}/metadata.yaml" ]]; then
  BAG_DIR="$(cd "${BAG_INPUT}" && pwd)"
elif [[ -f "${BAG_INPUT}/bag/metadata.yaml" ]]; then
  BAG_DIR="$(cd "${BAG_INPUT}/bag" && pwd)"
else
  echo "[ERROR] Could not find rosbag metadata under: ${BAG_INPUT}" >&2
  echo "        Pass either the run directory or its bag/ subdirectory." >&2
  exit 1
fi

set +u
source "${CONDA_SH}"
conda activate "${CONDA_ENV}"
source "${ROS_SETUP}"
RETARGET_SETUP="${RETARGET_WS}/install/setup.bash"
if [[ -f "${RETARGET_SETUP}" ]]; then
  source "${RETARGET_SETUP}"
fi
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN}"
export ROS2CLI_DISABLE_DAEMON=1

topic_info() {
  local topic="$1"
  set +e
  ros2 topic info -v "${topic}" 2>&1
  local rc=$?
  set -e
  return "${rc}"
}

extract_count() {
  local label="$1"
  awk -F: -v label="${label}" '
    index($0, label ":") == 1 {
      gsub(/[[:space:]]/, "", $2)
      print $2
      found=1
      exit
    }
    END {
      if (!found) {
        print 0
      }
    }
  '
}

TARGET_INFO="$(topic_info "${TARGET_TOPIC}" || true)"
TARGET_PUBLISHERS="$(printf '%s\n' "${TARGET_INFO}" | extract_count "Publisher count")"
TARGET_SUBSCRIBERS="$(printf '%s\n' "${TARGET_INFO}" | extract_count "Subscription count")"

if [[ "${ALLOW_NO_SUBSCRIBER}" -eq 0 && "${TARGET_SUBSCRIBERS}" -eq 0 ]]; then
  echo "[ERROR] Target topic has no subscribers: ${TARGET_TOPIC}" >&2
  echo "        Playback would not reach the retarget pipeline." >&2
  echo "        Start the pipeline with:" >&2
  echo "          ros2 launch manus_revo2_retarget humandex_real_hand_pipeline_launch.py hand_mode:=left eef_pose_topic:=${TARGET_TOPIC}" >&2
  echo "        Or pass --allow-no-subscriber to force playback." >&2
  exit 4
fi

if [[ "${ALLOW_LIVE_CONFLICT}" -eq 0 && "${TARGET_PUBLISHERS}" -gt 0 ]]; then
  echo "[ERROR] Target topic already has ${TARGET_PUBLISHERS} publisher(s): ${TARGET_TOPIC}" >&2
  echo "        Playback would mix with an existing live publisher." >&2
  echo "        Use the default remapped target /humandex_replay/eef_pose and point the adapter to it," >&2
  echo "        stop the live publisher, or pass --allow-live-conflict if this is intentional." >&2
  exit 5
fi

PLAY_CMD=(
  ros2 bag play "${BAG_DIR}"
  --storage "${STORAGE_ID}"
  --topics "${SOURCE_TOPIC}"
  --rate "${RATE}"
)
if [[ "${SOURCE_TOPIC}" != "${TARGET_TOPIC}" ]]; then
  PLAY_CMD+=(--remap "${SOURCE_TOPIC}:=${TARGET_TOPIC}")
fi
if [[ "${LOOP}" -eq 1 ]]; then
  PLAY_CMD+=(--loop)
fi
if [[ "${START_PAUSED}" -eq 1 ]]; then
  PLAY_CMD+=(--start-paused)
fi

echo "[INFO] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "[INFO] Bag: ${BAG_DIR}"
echo "[INFO] Source topic: ${SOURCE_TOPIC}"
echo "[INFO] Target topic: ${TARGET_TOPIC}"
echo "[INFO] Target subscribers: ${TARGET_SUBSCRIBERS}"
echo "[INFO] Playback rate: ${RATE}"
echo "[INFO] Command: ${PLAY_CMD[*]}"

exec "${PLAY_CMD[@]}"
