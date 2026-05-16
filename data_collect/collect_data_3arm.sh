#!/bin/bash
# 3-arm data collection driver.
# Prereqs (in separate terminals):
#   1. roslaunch ... start_ms_piper_3arm_collect.launch
#   2. roslaunch ... multi_camera_shuai.launch
#   3. python eef_avp_control_singlearm.py --arm m
# Then run this script. For each episode:
#   - It launches collect_data_3arm.py and waits for AVP ENGAGED state.
#   - Do your task while ENGAGED.
#   - Pinch right thumb+middle once to PAUSE (ENGAGED -> LOCKED).
#     Recording stops and the episode is saved.
#   - Bash script sleeps a few seconds, then starts the next episode.

set -e

DATASET_DIR=~/data_shuai_3arm
TASK_NAME=three_arm_pickup
TASK_DESCRIPTION="Three-arm coordinated task: drive mid arm via AVP head pose while operator demos left/right arms via teach mode."

FRAME_RATE=30
EPISODES=10
MAX_TIMESTEPS=1500

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

for ((i=0; i<EPISODES; i++)); do
    echo "============================="
    echo "Collecting episode $i / $((EPISODES - 1))"
    echo "Engage AVP (left pinch -> right pinch twice) to start."
    echo "Pinch right once to stop & save."
    echo "============================="

    python3 "$SCRIPT_DIR/collect_data_3arm.py" \
        --dataset_dir "$DATASET_DIR" \
        --task_name "$TASK_NAME" \
        --task_description "$TASK_DESCRIPTION" \
        --episode_idx "$i" \
        --max_timesteps "$MAX_TIMESTEPS" \
        --frame_rate "$FRAME_RATE"

    sleep 3
done

echo "All episodes collected!"
