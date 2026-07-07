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
TASK_NAME=pick_things_from_shelft_and_put_in_the_box_v4
TASK_DESCRIPTION="Open the bottom drawer on the right, take out the fruit inside, place it on the brown plate at the center of the tabletop, and then close the drawer"

FRAME_RATE=30
EXPECTED_FRAME_RATE=30
ALLOW_NON_30HZ="${ALLOW_NON_30HZ:-no}"
EPISODES=3
MAX_TIMESTEPS=3600        # 2 min @ 30 Hz; gesture stop should kick in well before

# Episode-index ranges for language labels. Format: "START-END|description".
# The first matching range is used; if nothing matches, TASK_DESCRIPTION is used.
TASK_DESCRIPTION_RANGES=(
    "0-1|Pick up the red box from the top shelf of the shelving unit on the left and place it on the blue box on the right."
    "2-3|Pick up the brown bottle from the top shelf of the shelving unit on the left and place it on the blue box on the right."

    )
    # "0-14|Pick up the red box from the top shelf of the shelving unit on the left and place it on the blue box on the right."

    # "15-29|Pick up the brown bottle from the top shelf of the shelving unit on the left and place it on the brown box on the left."
    # "0-1|Pick up the red box from the top shelf of the shelving unit on the left and place it on the brown box on the left."
    # "0-14|Pick up the yellow canister from the middle shelf of the shelving unit on the left and place it on the brown box on the left."

    # "15-29|Pick up the yellow canister from the middle shelf of the shelving unit on the left and place it on the blue box on the right."
    #  "0-14|Pick up the silver can from the bottom shelf of the shelving unit on the left and place it on the brown box on the left."

    # "15-29|Pick up the silver can from the bottom shelf of the shelving unit on the left and place it on the blue box on the right."

    # "30-44|Pick up the blue cookie box from the middle shelf of the shelving unit on the left and place it on the brown box on the left."

    # "45-59|Pick up the blue cookie box from the middle shelf of the shelving unit on the left and place it on the blue box on the right."
    # Pick up the green bell pepper from the left shelf, place it into the pot on the middle table, and then cover the pot with the lid.
    # Pick up the onion from the left shelf, place it into the pot on the middle table, and then cover the pot with the lid.
    # Pick up the yellow bell pepper from the left shelf, place it into the pot on the middle table, and then cover the pot with the lid.
    # Pick up the red bell pepper from the left shelf, place it into the pot on the middle table, and then cover the pot with the lid.
    # Pick up the white radish from the left shelf, place it into the pot on the middle table, and then cover the pot with the lid.
    # "30-59|Pick up the carrot from underneath the table, and place it into its corresponding labeled bin."
    # "60-89|Pick up the corn from underneath the table, and place it into its corresponding labeled bin."
    # "0-19|Pick up the carrot from underneath the table and place it into the corresponding labeled bin. Then, pick up the corn from underneath the table and place it into the corresponding labeled bin."
    # "20-39|Pick up the potato from underneath the table and place it into the corresponding labeled bin. Then, pick up the corn from underneath the table and place it into the corresponding labeled bin."
    # "40-59|Pick up the potato from underneath the table and place it into the corresponding labeled bin. Then, pick up the carrot from underneath the table and place it into the corresponding labeled bin."
    # "20-39|Pick up the potato from underneath the table, and place it into its corresponding labeled bin."
    # "40-59|Pick up the carrot from underneath the table, and place it into its corresponding labeled bin."
    # "60-79|Pick up the corn from underneath the table, and place it into its corresponding labeled bin."

# ============ Auto-upload config ============
# Set UPLOAD_AFTER_EPISODE=no to disable; the script then just keeps files locally.
UPLOAD_AFTER_EPISODE=no
WAIT_FOR_UPLOADS_AT_END=yes
REMOTE_SSH=hpc3_haoangli
REMOTE_SSH_FALLBACK=haoangli@10.120.48.26
REMOTE_BASE=/data/user/haoangli/shuai/active_perception/cobot_teleop_finetune/
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
CONFIRM_TASK_DESCRIPTION="${CONFIRM_TASK_DESCRIPTION:-no}"
UPLOAD_QUEUE_FIFO=""
UPLOAD_WORKER_PID=""
UPLOAD_QUEUE_FD=9
REMOTE_EPISODE_NEXT=0

trim_ws() {
    local s="$1"
    s="${s#"${s%%[![:space:]]*}"}"
    s="${s%"${s##*[![:space:]]}"}"
    printf "%s" "$s"
}

description_for_episode() {
    local episode_idx="$1"
    local entry
    local range
    local desc
    local start
    local end

    for entry in "${TASK_DESCRIPTION_RANGES[@]}"; do
        range="${entry%%|*}"
        desc="${entry#*|}"
        start="${range%-*}"
        end="${range#*-}"
        if [[ "$start" =~ ^[0-9]+$ ]] && [[ "$end" =~ ^[0-9]+$ ]] && \
                (( episode_idx >= start && episode_idx <= end )); then
            printf "%s" "$desc"
            return
        fi
    done

    printf "%s" "$TASK_DESCRIPTION"
}

description_range_contains_episode() {
    local episode_idx="$1"
    local entry
    local range
    local start
    local end

    for entry in "${TASK_DESCRIPTION_RANGES[@]}"; do
        range="${entry%%|*}"
        start="${range%-*}"
        end="${range#*-}"
        if [[ "$start" =~ ^[0-9]+$ ]] && [[ "$end" =~ ^[0-9]+$ ]] && \
                (( episode_idx >= start && episode_idx <= end )); then
            return 0
        fi
    done
    return 1
}

validate_frame_rate() {
    if [ "$FRAME_RATE" -eq "$EXPECTED_FRAME_RATE" ]; then
        return
    fi

    echo "[collect] ERROR: FRAME_RATE=${FRAME_RATE}, expected ${EXPECTED_FRAME_RATE} Hz." >&2
    echo "[collect] This changes dataset timing and HDF5 frame_rate metadata." >&2
    echo "[collect] Fix FRAME_RATE=${EXPECTED_FRAME_RATE}, or explicitly run with ALLOW_NON_30HZ=yes if you truly want a non-30Hz dataset." >&2
    if [ "$ALLOW_NON_30HZ" = "yes" ]; then
        echo "[collect] WARNING: ALLOW_NON_30HZ=yes set; continuing with FRAME_RATE=${FRAME_RATE}." >&2
        return
    fi
    exit 2
}

validate_description_ranges_for_run() {
    local missing=()
    local idx
    local end_idx=$((EPISODES - 1))

    for ((idx=0; idx<=end_idx; idx++)); do
        if ! description_range_contains_episode "$idx"; then
            missing+=("$idx")
        fi
    done

    if [ "${#missing[@]}" -eq 0 ]; then
        return
    fi

    echo "[collect] ERROR: TASK_DESCRIPTION_RANGES does not cover this run." >&2
    echo "[collect] Planned local episode range: 0-${end_idx}" >&2
    echo "[collect] Missing description ranges for episode(s): ${missing[*]}" >&2
    echo "[collect] Fix TASK_DESCRIPTION_RANGES or EPISODES. Refusing to use TASK_DESCRIPTION fallback silently." >&2
    exit 2
}

confirm_episode_description() {
    local episode_idx="$1"
    local desc="$2"
    local replacement

    if [ "$CONFIRM_TASK_DESCRIPTION" != "yes" ]; then
        printf "%s" "$desc"
        return
    fi

    if [ ! -t 0 ]; then
        echo "[collect] episode ${episode_idx} description: ${desc}" >&2
        printf "%s" "$desc"
        return
    fi

    echo "[collect] episode ${episode_idx} description:" >&2
    echo "  ${desc}" >&2
    read -rp "[collect] Press ENTER to use, or type replacement: " replacement
    replacement="$(trim_ws "$replacement")"
    if [ -n "$replacement" ]; then
        desc="$replacement"
    fi
    printf "%s" "$desc"
}

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
    local upload_status
    base="$(basename "$file")"
    echo "[upload] ${base} -> ${REMOTE_SSH}:${remote_file}"
    set +e
    if [ "$REMOTE_UPLOAD_COMPRESSION" = "none" ]; then
        upload_raw_rsync "$file" "$remote_file"
    else
        upload_lossless_stream "$file" "$remote_file"
    fi
    upload_status=$?
    set -e
    if [ "$upload_status" -eq 0 ]; then
        echo "[upload] success, removing local ${file}"
        rm -f "$file"
    else
        echo "[upload] FAILED for ${file} (status=${upload_status}). Keeping local file; will NOT retry automatically."
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
    local remote_idx="$2"
    local base
    base="$(basename "$file")"

    if [ "$UPLOAD_AFTER_EPISODE" != "yes" ] || [ -z "$UPLOAD_WORKER_PID" ]; then
        return
    fi

    if [ -z "$remote_idx" ]; then
        remote_idx=$REMOTE_EPISODE_NEXT
    fi
    if [ "$remote_idx" -ge "$REMOTE_EPISODE_NEXT" ]; then
        REMOTE_EPISODE_NEXT=$((remote_idx + 1))
    fi
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

validate_frame_rate
validate_description_ranges_for_run
init_remote_upload
start_upload_worker

# ============ Episode loop ============
for ((i=0; i<EPISODES; i++)); do
    EPISODE_DESCRIPTION="$(description_for_episode "$i")"
    EPISODE_DESCRIPTION="$(confirm_episode_description "$i" "$EPISODE_DESCRIPTION")"
    echo "============================="
    echo "Collecting local episode $i / $((EPISODES - 1))"
    echo "Description: ${EPISODE_DESCRIPTION}"
    echo "Hold both-hand pinch 2s to ENGAGE; long-hold both-hand pinch 4s to STOP."
    echo "============================="

    # collect_data_3arm.py exits 1 if no actions were recorded (e.g., script killed
    # before engage). We want to keep the loop alive in that case.
    set +e
    python3 "$SCRIPT_DIR/collect_data_3arm.py" \
        --dataset_dir "$DATASET_DIR" \
        --task_name "$TASK_NAME" \
        --task_description "$EPISODE_DESCRIPTION" \
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
