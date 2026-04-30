# Piper Dual-Arm Teleop

## Startup Sequence

```bash
# 1. CAN init (required after every power cycle / reboot)
cd /home/agilex/cobot_magic/Piper_ros_private-ros-noetic/
bash can_config.sh
source devel/setup.bash
```

```bash
# 2. Power-cycle the arms BEFORE launching the slaves:
#    turn off the power strip → unplug the master arms' aviation connectors → power on the strip
conda activate aloha
roslaunch piper start_ms_piper.launch mode:=1 auto_enable:=true
```

```bash
# 3. Start cameras
conda activate aloha
roslaunch astra_camera multi_camera.launch
```

```bash
# 4. Control script (pick one)
cd aloha-devel
bash act/inference.sh                                                                       # inference
python /home/agilex/cobot_magic/aloha-devel/Piper-AVP-Teleop/teleop/joint_keyboard_control.py  # joint-space keyboard
python /home/agilex/cobot_magic/aloha-devel/Piper-AVP-Teleop/teleop/eef_keyboard_control.py    # end-effector keyboard
```

> ⚠️ Stay alert during inference. If the arm misbehaves, stop the script or cut power immediately.

## Joint Dimension Map

| Left arm dim | Right arm dim | Description |
| --- | --- | --- |
| 0 | 7  | base z-axis rotation |
| 1 | 8  | base joint |
| 2 | 9  | upper-arm joint |
| 3 | 10 | forearm (rotation about its own axis) |
| 4 | 11 | forearm joint |
| 5 | 12 | wrist (rotation about its own axis) |
| 6 | 13 | gripper (0–0.1, closed–open) |

## EEF Dimension Map

End-effector pose = translation (xyz, m) + orientation (rpy, rad, sxyz / extrinsic convention), expressed in base → EE.

| Left dim | Right dim | Description | Unit | Default step |
| --- | --- | --- | --- | --- |
| 0  | 7  | x — EE X in base frame &nbsp;&nbsp;**(+x forward)** | m | 0.002 |
| 1  | 8  | y — EE Y in base frame &nbsp;&nbsp;**(+y left)**    | m | 0.002 |
| 2  | 9  | z — EE Z in base frame &nbsp;&nbsp;**(+z up)**      | m | 0.002 |
| 3  | 10 | roll  — rotation about base X (extrinsic, sxyz step 1) &nbsp;&nbsp;**(+roll tilts right)** | rad | 0.01 |
| 4  | 11 | pitch — rotation about base Y (extrinsic, sxyz step 2) &nbsp;&nbsp;**(+pitch tilts down / nods down)** | rad | 0.01 |
| 5  | 12 | yaw   — rotation about base Z (extrinsic, sxyz step 3) &nbsp;&nbsp;**(+yaw turns left)** | rad | 0.01 |
| 6  | 13 | gripper opening (0–0.1, closed–open) | m | 0.002 |

> Sign conventions above are verified empirically by jogging each axis in `eef_keyboard_control.py` and observing the real arm.

> rpy follows the ROS / aerospace `sxyz` convention. Equivalent readings: "rotate about the fixed base axes in order X→Y→Z" or, equivalently, "rotate about the current body axes in order Z→Y→X (reversed)". When pitch approaches ±90° you enter gimbal lock and the roll / yaw values become coupled.

## EEF Frame Convention

`/puppet/end_pose_*` reports the pose of the **joint6 frame** in the URDF — NOT the physical gripper / camera frame. `eef_keyboard_control.py` adds a static offset inside the IK model to switch the controlled frame to the **camera pose** (not the gripper):

| Component | Offset (in joint6 local frame) |
| --- | --- |
| Translation | `0.05 m along -X` |
| Rotation    | `Ry(-90°)` |

Code location: [eef_keyboard_control.py:37-50](teleop/eef_keyboard_control.py#L37-L50), the `addFrame` call.

**To find joint6 on the real robot**: run `joint_keyboard_control.py`, select dim **5** (left wrist) or **12** (right wrist), press w/s — the joint that moves is joint6.

**To switch to the gripper pose**: the gripper shares the same rotational offset (`Ry(-90°)`) but has no 5 cm translation. Just set the translation vector at [eef_keyboard_control.py:47](teleop/eef_keyboard_control.py#L47) to zero; leave the rotation line (line 40) untouched.

```python
# Current (camera pose)
ee_off_quat = quaternion_from_euler(0.0, -np.pi / 2.0, 0.0)  # rotation: keep as-is
# ...
np.array([-0.05, 0.0, 0.0]),                                  # ← change this line

# After (gripper pose)
ee_off_quat = quaternion_from_euler(0.0, -np.pi / 2.0, 0.0)  # rotation: keep as-is
# ...
np.array([0.0, 0.0, 0.0]),                                    # ← translation set to zero
```

After editing, verify visually with the `frame_visualize.py` command below (`--offset_xyz 0 0 0`) before running on the real robot.

## Frame Visualization Tool

Before changing the xyz / rpy in `addFrame`, **use `frame_visualize.py` to verify visually** that the resulting frame matches the target you want.

It writes an interactive HTML file (drag to rotate, scroll to zoom in browser):

```bash
# Example: verify the current addFrame offset (camera pose)
# --xyz / --rpy = current joint6 pose reported by the driver (read from /puppet/end_pose_*)
# --offset_xyz / --offset_rpy = the offset you plan to put in addFrame
python /home/agilex/cobot_magic/aloha-devel/Piper-AVP-Teleop/teleop/frame_visualize.py \
    --xyz 0.061058 -0.003505 0.20627 \
    --rpy -0.0364 1.5377 -0.0939 \
    --offset_xyz -0.05 0 0 \
    --offset_rpy 0 -90 0 --offset_deg \
    --html /tmp/frame_right_offset.html
xdg-open /tmp/frame_right_offset.html
```

Frames in the figure:

| Frame | Meaning |
| --- | --- |
| `base` | Robot base |
| `target` | **Current joint6 pose** (just `--xyz/--rpy`, no offset applied) |
| `main*(R-then-T) [SE3]` | **Final pose with offset applied** (this is the IK's internal ee frame) |
| `main*(T-then-R)` | Translate-then-rotate variant (coincides with the above when translation and rotation commute) |

Compare `main*(R-then-T)` against the real robot to check whether it matches the pose you expect (camera / gripper).

**Switching to the gripper** (set translation to zero first, then visually verify the rotation):
```bash
# --offset_xyz 0 0 0 → offset reduces to rotation only
python ... --offset_xyz 0 0 0 --offset_rpy 0 -90 0 --offset_deg --html /tmp/gripper.html
```

## URDF Mesh Setup (one-time)

`piper_description_new.urdf` references `gripper_base.STL`, which the private `Piper_ros_private-ros-noetic` package does **not** ship. Pinocchio's `BuildFromURDF` will fail with

```
ValueError: Mesh package://piper_description/meshes/gripper_base.STL could not be found.
```

until you drop that mesh in. The file lives in agilexrobotics' upstream `piper_ros`:

```bash
git clone https://github.com/agilexrobotics/piper_ros.git
cp piper_ros/src/piper_description/meshes/gripper_base.STL \
   /home/agilex/cobot_magic/Piper_ros_private-ros-noetic/src/piper_description/meshes/
```

Only `gripper_base.STL` is needed — all other meshes (`base_link.STL`, `link1-8.STL`) are already in the private package. After copying, you can delete the cloned `piper_ros/` directory if you don't need it for anything else.

## Runtime Environment

All scripts run inside the conda env `aloha`:

| Component | Version |
| --- | --- |
| Python | 3.8.19 |
| Pinocchio | 3.2.0 |
| CasADi | 3.6.5 |
| NumPy | 1.24.4 |
| SciPy | 1.10.1 |
| ROS | Noetic (system install) |

Always `conda activate aloha` before running anything. `frame_visualize.py` additionally needs `plotly` for the `--html` output — install with `pip install plotly`.
