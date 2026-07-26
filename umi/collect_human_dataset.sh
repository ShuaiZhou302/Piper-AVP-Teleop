#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

DATASET_DIR="${DATASET_DIR:-${REPO_ROOT}/umi/human_data}"
TASK_NAME="${TASK_NAME:-umi_human}"
TASK_DESCRIPTION="${TASK_DESCRIPTION:-human UMI recording with AVP head/hand pose}"
EPISODES="${EPISODES:-10}"
START_EPISODE="${START_EPISODE:-0}"
MAX_TIMESTEPS="${MAX_TIMESTEPS:-1500}"
FRAME_RATE="${FRAME_RATE:-30}"
CAMERA_TOPIC="${CAMERA_TOPIC:-/camera_f/color/image_raw}"
CAMERA_INFO_TOPIC="${CAMERA_INFO_TOPIC:-/camera_f/color/camera_info}"
START_IMMEDIATELY="${START_IMMEDIATELY:-0}"
STALE_REPEAT_S="${STALE_REPEAT_S:-1.5}"

TASK_DIR="${DATASET_DIR}/${TASK_NAME}"
mkdir -p "${TASK_DIR}"

echo "============================================================"
echo "[human-dataset] task            : ${TASK_NAME}"
echo "[human-dataset] description     : ${TASK_DESCRIPTION}"
echo "[human-dataset] dataset dir     : ${TASK_DIR}"
echo "[human-dataset] episodes        : ${EPISODES}"
echo "[human-dataset] start episode   : ${START_EPISODE}"
echo "[human-dataset] camera topic    : ${CAMERA_TOPIC}"
echo "[human-dataset] max timesteps   : ${MAX_TIMESTEPS}"
echo "[human-dataset] stale repeat s  : ${STALE_REPEAT_S}"
echo "============================================================"

for ((i=START_EPISODE; i<START_EPISODE+EPISODES; i++)); do
    out="${TASK_DIR}/episode_${i}.hdf5"
    if [[ -f "${out}" ]]; then
        echo "[human-dataset] ${out} exists; set START_EPISODE or move it first."
        exit 1
    fi

    echo
    echo "------------------------------------------------------------"
    echo "[human-dataset] episode ${i} -> ${out}"
    echo "Wear AVP, enter Vuer, then hold both pinch to start/stop."
    echo "------------------------------------------------------------"

    OUTPUT="${out}" \
    TASK_NAME="${TASK_NAME}" \
    TASK_DESCRIPTION="${TASK_DESCRIPTION}" \
    EPISODE_IDX="${i}" \
    MAX_TIMESTEPS="${MAX_TIMESTEPS}" \
    FRAME_RATE="${FRAME_RATE}" \
    CAMERA_TOPIC="${CAMERA_TOPIC}" \
    CAMERA_INFO_TOPIC="${CAMERA_INFO_TOPIC}" \
    START_IMMEDIATELY="${START_IMMEDIATELY}" \
    STALE_REPEAT_S="${STALE_REPEAT_S}" \
        "${SCRIPT_DIR}/start_human_data_collect.sh"

    echo "[human-dataset] episode ${i} done."
    if [[ "${i}" -lt $((START_EPISODE + EPISODES - 1)) ]]; then
        echo "[human-dataset] next episode starts in 3s..."
        sleep 3
    fi
done

echo "[human-dataset] all episodes collected."
