#!/bin/bash
# 3-arm data collection driver.
# Prereqs (in separate terminals):
#   1. roslaunch ... start_ms_piper_3arm_collect.launch
#   2. roslaunch ... multi_camera_shuai.launch
#   3. python eef_avp_control_singlearm.py --arm m
# Then run this script. For each episode:
#   - It launches collect_data_3arm.py and waits for AVP ENGAGED state.
#   - Do your task while ENGAGED.
#   - Long-hold both-hand pinch 4s to PAUSE (ENGAGED -> DISARMED -> IDLE).
#     Recording stops and the episode is saved.
#   - If UPLOAD_AFTER_EPISODE=yes, file is queued for background rsync upload.
#   - Bash script sleeps a few seconds, then starts the next episode without
#     waiting for upload to finish.
#
# Auto-upload prereq: `rsync` and `sshpass`.

set -e

# ============ Local collection config ============
DATASET_DIR=~/data_shuai_3arm
TASK_NAME=pick_things_from_backpack
TASK_DESCRIPTION="Took a bottle out of the backpack and put it out on the table"

FRAME_RATE=30
EPISODES=50
MAX_TIMESTEPS=3600        # 2 min @ 30 Hz; gesture stop should kick in well before

# ============ Auto-upload config ============
# Set UPLOAD_AFTER_EPISODE=no to disable; the script then just keeps files locally.
UPLOAD_AFTER_EPISODE=yes
WAIT_FOR_UPLOADS_AT_END=yes
REMOTE_SSH=hpc3_haoangli
REMOTE_SSH_FALLBACK=haoangli@10.120.48.26
REMOTE_BASE=/data/user/haoangli/shuai/active_perception
REMOTE_TASK_DIR="${REMOTE_BASE}/${TASK_NAME}"
# Faster for multi-GB new files on a LAN/HPC link: skip rsync delta checks,
# keep partial files for resume, and avoid SSH compression CPU overhead.
RSYNC_RSH="ssh -T -o Compression=no -x"
RSYNC_EXTRA_OPTS="${RSYNC_EXTRA_OPTS:-}"
REMOTE_PASS="${HPC3_REMOTE_PASS:-}"
# auto = stream with zstd/gzip if available, then decompress on hpc3 into
# the original .hdf5. This is lossless; use "none" to force raw rsync.
LOSSLESS_UPLOAD_COMPRESSION="${LOSSLESS_UPLOAD_COMPRESSION:-auto}"
REMOTE_UPLOAD_COMPRESSION=none

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UPLOAD_QUEUE_FIFO=""
UPLOAD_WORKER_PID=""
UPLOAD_QUEUE_FD=9
REMOTE_EPISODE_NEXT=0

# ============ Init: validate upload setup once before first episode ============
if [ "$UPLOAD_AFTER_EPISODE" = "yes" ]; then
    if ! command -v rsync >/dev/null 2>&1; then
        echo "[upload] rsync not found. Install with: sudo apt install rsync"
        echo "[upload] Or set UPLOAD_AFTER_EPISODE=no at the top of this script."
        exit 1
    fi
    if ! command -v sshpass >/dev/null 2>&1; then
        echo "[upload] sshpass not found. Install with: sudo apt install sshpass"
        echo "[upload] Or set UPLOAD_AFTER_EPISODE=no at the top of this script."
        exit 1
    fi
fi

# Single-quote a local string for the remote shell.
remote_quote() {
    printf "'"
    printf "%s" "$1" | sed "s/'/'\\\\''/g"
    printf "'"
}

resolve_remote_ssh() {
    local cfg_host
    local cfg_user
    cfg_host="$(ssh -G "$REMOTE_SSH" 2>/dev/null | awk '$1 == "hostname" {print $2; exit}')"
    cfg_user="$(ssh -G "$REMOTE_SSH" 2>/dev/null | awk '$1 == "user" {print $2; exit}')"
    if [ "$REMOTE_SSH" = "hpc3_haoangli" ] && \
            { [ "$cfg_host" != "10.120.48.26" ] || [ "$cfg_user" != "haoangli" ]; }; then
        echo "[upload] ssh alias hpc3_haoangli is not configured for this user; using ${REMOTE_SSH_FALLBACK}"
        REMOTE_SSH="$REMOTE_SSH_FALLBACK"
    fi
}

ensure_remote_password() {
    if [ "$UPLOAD_AFTER_EPISODE" != "yes" ] || [ -n "$REMOTE_PASS" ]; then
        return
    fi
    if [ -t 0 ]; then
        read -rsp "[upload] Password for ${REMOTE_SSH}: " REMOTE_PASS
        echo
    else
        echo "[upload] no terminal for password prompt. Set HPC3_REMOTE_PASS before running."
        UPLOAD_AFTER_EPISODE=no
    fi
}

remote_ssh() {
    SSHPASS="$REMOTE_PASS" sshpass -e ssh \
        -o ConnectTimeout=5 -o StrictHostKeyChecking=accept-new \
        "$REMOTE_SSH" "$@"
}

choose_upload_compression() {
    REMOTE_UPLOAD_COMPRESSION=none
    if [ "$LOSSLESS_UPLOAD_COMPRESSION" = "none" ]; then
        echo "[upload] lossless stream compression disabled; using raw rsync."
        return
    fi

    if command -v zstd >/dev/null 2>&1 && \
            remote_ssh "command -v zstd >/dev/null 2>&1"; then
        REMOTE_UPLOAD_COMPRESSION=zstd
    elif command -v gzip >/dev/null 2>&1 && \
            remote_ssh "command -v gzip >/dev/null 2>&1"; then
        REMOTE_UPLOAD_COMPRESSION=gzip
    fi

    if [ "$REMOTE_UPLOAD_COMPRESSION" = "none" ]; then
        echo "[upload] no shared zstd/gzip found; using raw rsync."
    else
        echo "[upload] using lossless ${REMOTE_UPLOAD_COMPRESSION} stream compression."
    fi
}

get_remote_next_episode_idx() {
    local remote_task_q
    local max_idx
    remote_task_q="$(remote_quote "$REMOTE_TASK_DIR")"
    max_idx="$(remote_ssh \
        "find ${remote_task_q} -maxdepth 1 -type f -name 'episode_*.hdf5' -printf '%f\n' 2>/dev/null | sed -n 's/^episode_\\([0-9][0-9]*\\)\\.hdf5$/\\1/p' | sort -n | tail -1")"
    if [ -z "$max_idx" ]; then
        echo 0
    else
        echo $((max_idx + 1))
    fi
}

init_remote_upload() {
    if [ "$UPLOAD_AFTER_EPISODE" != "yes" ]; then
        return
    fi

    resolve_remote_ssh
    ensure_remote_password
    if [ "$UPLOAD_AFTER_EPISODE" != "yes" ]; then
        return
    fi

    local remote_task_q
    remote_task_q="$(remote_quote "$REMOTE_TASK_DIR")"
    echo "[upload] testing ssh to ${REMOTE_SSH}..."
    if remote_ssh \
            "mkdir -p ${remote_task_q} && echo connect-ok" >/dev/null 2>&1; then
        REMOTE_EPISODE_NEXT="$(get_remote_next_episode_idx)"
        choose_upload_compression
        echo "[upload] ssh ok. target dir: ${REMOTE_SSH}:${REMOTE_TASK_DIR}/"
        echo "[upload] remote next episode index: ${REMOTE_EPISODE_NEXT}"
    else
        echo "[upload] ssh connection FAILED. Disabling auto-upload for this run."
        echo "[upload] (Files will accumulate locally in ${DATASET_DIR}/${TASK_NAME}/)"
        UPLOAD_AFTER_EPISODE=no
    fi
}

upload_raw_rsync() {
    local file="$1"
    local remote_file="$2"
    SSHPASS="$REMOTE_PASS" rsync -av --whole-file --partial --info=progress2 \
        ${RSYNC_EXTRA_OPTS} \
        -e "sshpass -e ${RSYNC_RSH} -o StrictHostKeyChecking=accept-new" \
        "$file" "${REMOTE_SSH}:${remote_file}"
}

upload_lossless_stream() {
    local file="$1"
    local remote_file="$2"
    local remote_file_q
    local remote_tmp_q
    local remote_tmp="${remote_file}.uploading.$$"
    remote_file_q="$(remote_quote "$remote_file")"
    remote_tmp_q="$(remote_quote "$remote_tmp")"

    case "$REMOTE_UPLOAD_COMPRESSION" in
        zstd)
            zstd -1 -T0 -c "$file" | remote_ssh \
                "zstd -d -c > ${remote_tmp_q} && mv -f ${remote_tmp_q} ${remote_file_q}"
            ;;
        gzip)
            gzip -1 -c "$file" | remote_ssh \
                "gzip -d -c > ${remote_tmp_q} && mv -f ${remote_tmp_q} ${remote_file_q}"
            ;;
        *)
            return 2
            ;;
    esac
}

# Upload a file to remote/<TASK_NAME>/episode_<remote_idx>.hdf5,
# delete local on success.
upload_and_delete() {
    local file="$1"
    local remote_idx="$2"
    local remote_file="${REMOTE_TASK_DIR}/episode_${remote_idx}.hdf5"
    local base
    base="$(basename "$file")"
    echo "[upload] ${base} -> ${REMOTE_SSH}:${remote_file}"
    if [ "$REMOTE_UPLOAD_COMPRESSION" = "none" ]; then
        upload_raw_rsync "$file" "$remote_file"
    else
        upload_lossless_stream "$file" "$remote_file"
    fi
    if [ "$?" -eq 0 ]; then
        echo "[upload] success, removing local ${file}"
        rm -f "$file"
    else
        echo "[upload] FAILED for ${file}. Keeping local file; will NOT retry automatically."
    fi
}

start_upload_worker() {
    if [ "$UPLOAD_AFTER_EPISODE" != "yes" ]; then
        return
    fi

    UPLOAD_QUEUE_FIFO="$(mktemp -u /tmp/piper_collect_upload_queue.XXXXXX)"
    mkfifo "$UPLOAD_QUEUE_FIFO"

    (
        while IFS=$'\t' read -r file remote_idx; do
            if [ -z "$file" ]; then
                continue
            fi
            if [ -f "$file" ]; then
                upload_and_delete "$file" "$remote_idx"
            else
                echo "[upload] queued file missing, skip: $file"
            fi
        done < "$UPLOAD_QUEUE_FIFO"
        echo "[upload] background worker stopped."
    ) &
    UPLOAD_WORKER_PID=$!

    # Keep a persistent writer FD open. Without this, the FIFO reader exits
    # after the first enqueue closes its write side.
    eval "exec ${UPLOAD_QUEUE_FD}>\"\$UPLOAD_QUEUE_FIFO\""
    rm -f "$UPLOAD_QUEUE_FIFO"
    echo "[upload] background worker started (pid=${UPLOAD_WORKER_PID})."
}

enqueue_upload() {
    local file="$1"
    local base
    local remote_idx
    base="$(basename "$file")"

    if [ "$UPLOAD_AFTER_EPISODE" != "yes" ] || [ -z "$UPLOAD_WORKER_PID" ]; then
        return
    fi

    remote_idx=$REMOTE_EPISODE_NEXT
    REMOTE_EPISODE_NEXT=$((REMOTE_EPISODE_NEXT + 1))
    echo "[upload] queued ${base} -> remote episode_${remote_idx}.hdf5; collection will continue while upload runs."
    printf '%s\t%s\n' "$file" "$remote_idx" >&${UPLOAD_QUEUE_FD}
}

finish_upload_worker() {
    if [ "$UPLOAD_AFTER_EPISODE" != "yes" ] || [ -z "$UPLOAD_WORKER_PID" ]; then
        return
    fi

    local pid="$UPLOAD_WORKER_PID"
    UPLOAD_WORKER_PID=""
    eval "exec ${UPLOAD_QUEUE_FD}>&-"
    if [ "$WAIT_FOR_UPLOADS_AT_END" = "yes" ]; then
        echo "[upload] waiting for queued uploads to finish..."
        wait "$pid"
    else
        echo "[upload] queued uploads are still running in background (pid=${pid})."
    fi
}

cleanup_on_interrupt() {
    echo
    echo "[collect] interrupted. Closing upload queue."
    finish_upload_worker
    exit 130
}

trap cleanup_on_interrupt INT TERM

init_remote_upload
start_upload_worker

# ============ Episode loop ============
for ((i=0; i<EPISODES; i++)); do
    echo "============================="
    echo "Collecting episode $i / $((EPISODES - 1))"
    echo "Hold both-hand pinch 2s to ENGAGE; long-hold both-hand pinch 4s to STOP."
    echo "============================="

    # collect_data_3arm.py exits 1 if no actions were recorded (e.g., script killed
    # before engage). We want to keep the loop alive in that case.
    set +e
    python3 "$SCRIPT_DIR/collect_data_3arm.py" \
        --dataset_dir "$DATASET_DIR" \
        --task_name "$TASK_NAME" \
        --task_description "$TASK_DESCRIPTION" \
        --episode_idx "$i" \
        --max_timesteps "$MAX_TIMESTEPS" \
        --frame_rate "$FRAME_RATE"
    PY_STATUS=$?
    set -e

    EPISODE_FILE="${DATASET_DIR}/${TASK_NAME}/episode_${i}.hdf5"
    if [ "$PY_STATUS" -eq 0 ] && [ -f "$EPISODE_FILE" ]; then
        if [ "$UPLOAD_AFTER_EPISODE" = "yes" ]; then
            enqueue_upload "$EPISODE_FILE"
        fi
    else
        echo "[collect] episode $i did not produce $EPISODE_FILE (status=$PY_STATUS); skipping upload."
    fi

    sleep 3
done

finish_upload_worker

echo "All episodes collected!"
