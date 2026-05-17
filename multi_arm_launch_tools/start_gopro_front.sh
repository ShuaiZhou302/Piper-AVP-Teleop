#!/usr/bin/env bash
# 启动 "左/右 Dabai + GoPro 替代前相机" 的相机组合。
#
# 三步骤包在一个脚本里:
#   1. sudo gopro webcam  - 把 GoPro Hero 12 切到 webcam 模式 (SuperView FOV)
#   2. ffmpeg            - 把 GoPro 的 UDP 流转到 v4l2loopback /dev/video42
#   3. roslaunch         - 启动 2 个 Dabai + 1 个 usb_cam_node (读 /dev/video42)
#
# 前置条件:
#   - GoPro 已经通过 USB-C 数据线连接到主机
#   - GoPro 已经开机, 处于待机状态
#   - 已经 sudo bash gopro_as_webcam_on_linux/install.sh 装好 gopro 命令
#     (或者本脚本直接调用 submodule 里的 ./gopro)
#   - ROS master 已经在跑 (例如已经先起了机械臂 launch); 没起的话本脚本自己起 roscore
#
# 用法:
#   bash multi_arm_launch_tools/start_gopro_front.sh
#
# 退出: Ctrl+C 会同时结束 ffmpeg 和 roslaunch.

set -e

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
GOPRO_BIN="$REPO_ROOT/gopro_as_webcam_on_linux/gopro"
LAUNCH_FILE="$REPO_ROOT/multi_arm_launch_tools/launch/multi_camera_gopro_front.launch"

# 可调参数 - 命令行覆盖
RESOLUTION="${RESOLUTION:-720}"          # GoPro 内部分辨率请求 (Hero 12 通常忽略, 实际还是 1080p)
FOV="${FOV:-superview}"                  # linear / wide / narrow / superview
FFMPEG_WARMUP_SEC="${FFMPEG_WARMUP_SEC:-5}"

FFMPEG_PID=""

cleanup() {
    echo ""
    echo "[start_gopro_front] 正在清理..."
    if [ -n "$FFMPEG_PID" ] && kill -0 "$FFMPEG_PID" 2>/dev/null; then
        kill -INT "$FFMPEG_PID" 2>/dev/null || true
        wait "$FFMPEG_PID" 2>/dev/null || true
        echo "[start_gopro_front] ffmpeg 已停止"
    fi
}
trap cleanup EXIT INT TERM

# ---- Step 1: 启动 GoPro webcam 模式 ----
echo "==========================================="
echo " 1/3 启动 GoPro webcam 模式 (FOV=$FOV)"
echo "==========================================="
if [ ! -x "$GOPRO_BIN" ]; then
    echo "[start_gopro_front] 错误: 找不到 $GOPRO_BIN" >&2
    echo "[start_gopro_front] 请先 git submodule update --init" >&2
    exit 1
fi
sudo "$GOPRO_BIN" webcam -r "$RESOLUTION" -f "$FOV" -n
# -n = non-interactive (不等用户按 Return)
# 这里 -r 720 在 Hero 12 上一般会被相机固件忽略, 仍然吐 1080p, 见 GOPRO_README.md.

# ---- Step 2: 后台启动 ffmpeg, 把 UDP 流转到 /dev/video42 ----
echo ""
echo "==========================================="
echo " 2/3 启动 ffmpeg → /dev/video42"
echo "==========================================="
ffmpeg -nostdin -threads 0 -loglevel warning -stats \
    -fflags nobuffer+discardcorrupt -flags low_delay \
    -i 'udp://@0.0.0.0:8554?overrun_nonfatal=1&fifo_size=1000000' \
    -f:v mpegts -fflags nobuffer \
    -vf format=yuyv422 \
    -f v4l2 /dev/video42 &
FFMPEG_PID=$!
echo "[start_gopro_front] ffmpeg PID = $FFMPEG_PID"
echo "[start_gopro_front] 等待 ${FFMPEG_WARMUP_SEC}s 让 ffmpeg 渡过 SPS/PPS 启动噪声..."
sleep "$FFMPEG_WARMUP_SEC"

# 健康检查: ffmpeg 还活着吗?
if ! kill -0 "$FFMPEG_PID" 2>/dev/null; then
    echo "[start_gopro_front] 错误: ffmpeg 已经死了, 检查 GoPro 状态" >&2
    exit 1
fi

# ---- Step 3: roslaunch ----
echo ""
echo "==========================================="
echo " 3/3 roslaunch (2 Dabai + 1 GoPro usb_cam)"
echo "==========================================="
echo "  Launch file: $LAUNCH_FILE"
echo "  Ctrl+C 退出会同时停掉 ffmpeg"
echo ""
roslaunch "$LAUNCH_FILE"
