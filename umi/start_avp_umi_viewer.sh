#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

CAMERA_TOPIC="${CAMERA_TOPIC:-/camera_f/color/image_raw}"
VIEW_MODE="${VIEW_MODE:-panel}"
HOST_IP="${HOST_IP:-}"
HZ="${HZ:-30}"
STREAM_FPS="${STREAM_FPS:-30}"
IMAGE_QUALITY="${IMAGE_QUALITY:-35}"
HUD="${HUD:-minimal}"
PANEL_HEIGHT="${PANEL_HEIGHT:-2.4}"

cd "${REPO_ROOT}"

echo "[umi-avp] camera topic : ${CAMERA_TOPIC}  (default = mid/front camera_f)"
echo "[umi-avp] view mode    : ${VIEW_MODE}"
echo "[umi-avp] hud          : ${HUD}"
echo "[umi-avp] stream fps   : ${STREAM_FPS}"
echo "[umi-avp] jpeg quality : ${IMAGE_QUALITY}"
echo "[umi-avp] panel height : ${PANEL_HEIGHT}"
if [[ -n "${HOST_IP}" ]]; then
    echo "[umi-avp] host ip      : ${HOST_IP}"
fi

if command -v rostopic >/dev/null 2>&1; then
    if ! rostopic list >/dev/null 2>&1; then
        cat <<'EOF'
[umi-avp] ROS master is not running.

Start the existing camera launch first in another terminal:
    conda activate aloha
    roslaunch /home/agilex/cobot_magic/aloha-devel/Piper-AVP-Teleop/multi_arm_launch_tools/launch/multi_camera_shuai.launch

It should publish the mid/front camera as:
    /camera_f/color/image_raw

If your AVP external camera is on another topic, run:
    CAMERA_TOPIC=/your/camera/topic ./umi/start_avp_umi_viewer.sh
EOF
        exit 1
    fi
else
    cat <<'EOF'
[umi-avp] WARN: rostopic not found.
Did you source the Piper ROS workspace?
    source /home/agilex/cobot_magic/Piper_ros_private-ros-noetic/devel/setup.bash

Continuing and letting rospy try to connect.
EOF
fi

args=(
    "umi/avp_umi_viewer.py"
    "--camera_topic" "${CAMERA_TOPIC}"
    "--view_mode" "${VIEW_MODE}"
    "--hud" "${HUD}"
    "--hz" "${HZ}"
    "--stream_fps" "${STREAM_FPS}"
    "--image_quality" "${IMAGE_QUALITY}"
    "--panel_height" "${PANEL_HEIGHT}"
)

if [[ -n "${HOST_IP}" ]]; then
    args+=("--host_ip" "${HOST_IP}")
fi

python3 "${args[@]}"
