# UMI AVP Viewer

Purpose: use AVP as the pose source and display an external camera stream inside
the AVP Vuer scene.

The AVP does not expose a raw video stream through WebXR. For UMI, mount/connect
the external camera separately on the PC side and publish it as a ROS image
topic. This viewer pushes that ROS image back into AVP and overlays the AVP
headset relative pose.

## Run

Every fresh boot/reboot needs the same ROS/CAN environment preflight used by the
multi-arm stack. Run this first:

```bash
# 每次开机 / 重启后必须做;脚本里以 USB bus-info 识别 4 个 CAN 模块
# (左臂 / 右臂 / 中间臂 / 底盘)
cd /home/agilex/cobot_magic/aloha-devel/Piper-AVP-Teleop/
bash multi_arm_launch_tools/can_config_shuai.sh
source /home/agilex/cobot_magic/Piper_ros_private-ros-noetic/devel/setup.bash
```

Start the existing 3-camera launch first. In this repo, the UMI/mid/front camera
is the third camera in `multi_camera_shuai.launch`, with prefix `f`, so its color
topic is `/camera_f/color/image_raw`.

```bash
conda activate aloha
roslaunch /home/agilex/cobot_magic/aloha-devel/Piper-AVP-Teleop/multi_arm_launch_tools/launch/multi_camera_shuai.launch
```

Then start the AVP viewer:

```bash
conda activate aloha
cd /home/agilex/cobot_magic/aloha-devel/Piper-AVP-Teleop
./umi/start_avp_umi_viewer.sh
```

If your UMI/AVP camera is published under a different topic, replace
`/camera_f/color/image_raw`:

```bash
CAMERA_TOPIC=/your/camera/topic ./umi/start_avp_umi_viewer.sh
```

By default the camera feed is a lower-center panel with a minimal one-line HUD.
The defaults favor lower latency:

- `VIEW_MODE=panel`
- `HUD=minimal`
- `STREAM_FPS=30`
- `IMAGE_QUALITY=35`
- `PANEL_HEIGHT=2.4`

For no text overlay:

```bash
HUD=none ./umi/start_avp_umi_viewer.sh
```

For an even larger panel:

```bash
PANEL_HEIGHT=3.0 ./umi/start_avp_umi_viewer.sh
```

For a full debugging view:

```bash
VIEW_MODE=full ./umi/start_avp_umi_viewer.sh
```

For lower latency, try lowering JPEG quality further:

```bash
IMAGE_QUALITY=25 STREAM_FPS=24 ./umi/start_avp_umi_viewer.sh
```

To move/resize the panel:

```bash
python3 umi/avp_umi_viewer.py \
    --camera_topic /camera_f/color/image_raw \
    --panel_height 0.7 \
    --panel_x 0.0 --panel_y -0.7 --panel_z -2.2
```

On AVP Safari, open the URL printed by the script:

```text
https://<PC_LAN_IP>:8012?ws=wss://<PC_LAN_IP>:8012
```

Then tap `Enter VR`.

## Pose Display

- `AVP rel xyz`: raw WebXR delta from the locked origin.
- `UMI rel xyz`: the same delta remapped to `x forward, y left, z up`.
- `UMI rpy deg`: relative headset orientation in the remapped UMI frame.

The first valid AVP pose is used as the origin. Hold both-hand pinch for 1 second
to reset the origin from inside AVP.

The script also publishes:

- `/umi/avp/head_pose`: raw AVP headset pose, frame `avp_world`.
- `/umi/avp/head_relative_pose`: relative remapped pose, frame `umi_head_origin`.

## Human Data Collect

`human_data_collect.py` records the human side only: external camera RGB plus
AVP WebXR head/hand tracking. It does not command robot arms.

After the same preflight and `multi_camera_shuai.launch`, run:

```bash
./umi/start_human_data_collect.sh
```

Default output:

```text
umi/human_data/episode_0.hdf5
```

Override output or episode length:

```bash
OUTPUT=/tmp/human_ep0.hdf5 MAX_TIMESTEPS=900 ./umi/start_human_data_collect.sh
```

For a quick debug run without the start gesture:

```bash
START_IMMEDIATELY=1 MAX_TIMESTEPS=300 ./umi/start_human_data_collect.sh
```

For a standard multi-episode dataset interface:

```bash
TASK_NAME=pick_cup_human \
TASK_DESCRIPTION="human UMI cup pickup" \
EPISODES=10 \
MAX_TIMESTEPS=1500 \
    ./umi/collect_human_dataset.sh
```

This writes:

```text
umi/human_data/<TASK_NAME>/episode_<i>.hdf5
```

Gesture lifecycle:

- Hold both-hand pinch for 1 second: start recording.
- Hold both-hand pinch for 2 seconds while recording: stop and save.
- Release during the stop hold: cancel stop and keep recording.

HDF5 layout:

```text
observations/images/cam_front                     (T,480,640,3)
observations/avp/head/matrix                      (T,4,4)
observations/avp/head/relative_matrix             (T,4,4)
observations/avp/left_hand/matrix                 (T,4,4)
observations/avp/right_hand/matrix                (T,4,4)
observations/avp/left_landmarks                   (T,25,3)
observations/avp/right_landmarks                  (T,25,3)
observations/avp/tracking/left_hand_fresh         (T,)
observations/avp/tracking/right_hand_fresh        (T,)
observations/avp/tracking/pinch_distance          (T,2) left,right
observations/avp/tracking/stale_count             (T,2) left,right
observations/avp/tracking/all_zero                (T,2) left,right
observations/avp/tracking/gesture_state           (T,) 0/1/2
timestamp/wall_time                               (T,)
timestamp/camera_ros_stamp                        (T,)
camera_info/cam_front/{K,D,R,P}                   optional
```

Tracking masks are derived by the collector. In the current Vuer/WebXR path we
do not receive an explicit confidence bool from AVP. A hand is marked stale when
its landmarks are all zeros or exactly repeated for `STALE_REPEAT_S` seconds
(default `1.5`). This uses bitwise equality, not a low-motion threshold: a real
hand that is merely holding still should still have tiny WebXR jitter and remain
fresh. If WebXR/Vuer repeats the exact same packet because tracking stopped or
events stopped arriving, the mask flips false.

Tune the conservative stale detector:

```bash
STALE_REPEAT_S=2.0 ./umi/start_human_data_collect.sh
```

Human collect preview stream defaults:

- `STREAM_FPS=24`
- `IMAGE_QUALITY=30`
- saved HDF5 images are raw ROS frames resized to `480x640`; preview JPEG
  quality does not affect saved data quality

Visualize a saved episode:

```bash
python3 umi/visualize_human_data.py umi/human_data/episode_0.hdf5
```

This writes:

```text
umi/human_data/episode_0_preview.mp4
```

For a shorter preview:

```bash
python3 umi/visualize_human_data.py umi/human_data/episode_0.hdf5 --max_frames 180
```
