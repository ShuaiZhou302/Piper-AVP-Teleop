#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

CAMERA_TOPIC="${CAMERA_TOPIC:-/camera_f/color/image_raw}"
CAMERA_INFO_TOPIC="${CAMERA_INFO_TOPIC:-/camera_f/color/camera_info}"
OUTPUT="${OUTPUT:-${REPO_ROOT}/umi/human_data/episode_0.hdf5}"
TASK_NAME="${TASK_NAME:-umi_human}"
TASK_DESCRIPTION="${TASK_DESCRIPTION:-}"
EPISODE_IDX="${EPISODE_IDX:-0}"
FRAME_RATE="${FRAME_RATE:-30}"
MAX_TIMESTEPS="${MAX_TIMESTEPS:-1500}"
START_IMMEDIATELY="${START_IMMEDIATELY:-0}"
IMAGE_QUALITY="${IMAGE_QUALITY:-30}"
STREAM_FPS="${STREAM_FPS:-24}"
PANEL_HEIGHT="${PANEL_HEIGHT:-2.4}"
STALE_REPEAT_S="${STALE_REPEAT_S:-1.5}"

cd "${REPO_ROOT}"

echo "[human-collect] camera topic      : ${CAMERA_TOPIC}"
echo "[human-collect] camera info topic : ${CAMERA_INFO_TOPIC}"
echo "[human-collect] output            : ${OUTPUT}"
echo "[human-collect] task name         : ${TASK_NAME}"
echo "[human-collect] episode idx       : ${EPISODE_IDX}"
echo "[human-collect] frame rate        : ${FRAME_RATE}"
echo "[human-collect] max timesteps     : ${MAX_TIMESTEPS}"
echo "[human-collect] preview fps       : ${STREAM_FPS}"
echo "[human-collect] preview quality   : ${IMAGE_QUALITY}"
echo "[human-collect] panel height      : ${PANEL_HEIGHT}"
echo "[human-collect] stale repeat s    : ${STALE_REPEAT_S}"

if command -v rostopic >/dev/null 2>&1; then
    if ! rostopic list >/dev/null 2>&1; then
        cat <<'EOF'
[human-collect] ROS master is not running.

Run the normal preflight and camera launch first:
    cd /home/agilex/cobot_magic/aloha-devel/Piper-AVP-Teleop/
    bash multi_arm_launch_tools/can_config_shuai.sh
    source /home/agilex/cobot_magic/Piper_ros_private-ros-noetic/devel/setup.bash
    roslaunch /home/agilex/cobot_magic/aloha-devel/Piper-AVP-Teleop/multi_arm_launch_tools/launch/multi_camera_shuai.launch
EOF
        exit 1
    fi
else
    cat <<'EOF'
[human-collect] WARN: rostopic not found.
Did you source the Piper ROS workspace?
    source /home/agilex/cobot_magic/Piper_ros_private-ros-noetic/devel/setup.bash
EOF
fi

args=(
    "umi/human_data_collect.py"
    "--camera_topic" "${CAMERA_TOPIC}"
    "--camera_info_topic" "${CAMERA_INFO_TOPIC}"
    "--output" "${OUTPUT}"
    "--task_name" "${TASK_NAME}"
    "--task_description" "${TASK_DESCRIPTION}"
    "--episode_idx" "${EPISODE_IDX}"
    "--frame_rate" "${FRAME_RATE}"
    "--max_timesteps" "${MAX_TIMESTEPS}"
    "--image_quality" "${IMAGE_QUALITY}"
    "--stream_fps" "${STREAM_FPS}"
    "--panel_height" "${PANEL_HEIGHT}"
    "--stale_repeat_s" "${STALE_REPEAT_S}"
)

if [[ "${START_IMMEDIATELY}" == "1" ]]; then
    args+=("--start_immediately")
fi

python3 "${args[@]}"
