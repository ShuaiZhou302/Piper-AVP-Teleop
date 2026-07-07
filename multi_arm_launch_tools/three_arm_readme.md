

# 下电之前 一定要扶着五个臂！！！！！

中臂通过puppet/joint mid 控制 确保不用发错 
中臂控制can口固定于左三

inference 模式

```bash
# 每次开机/重启后必须做;脚本里以 USB bus-info 识别 4 个 CAN 模块
# (左臂 / 右臂 / 中间臂 / 底盘)
cd /home/agilex/cobot_magic/aloha-devel/Piper-AVP-Teleop/
bash multi_arm_launch_tools/can_config_shuai.sh
source /home/agilex/cobot_magic/Piper_ros_private-ros-noetic/devel/setup.bash
```

```bash
# 启动前确认: 排插已上电 + 第三个臂的航空插头已接好
conda activate aloha
roslaunch /home/agilex/cobot_magic/aloha-devel/Piper-AVP-Teleop/multi_arm_launch_tools/launch/start_ms_piper_3arm.launch mode:=1 auto_enable:=true
```


```bash
# 启动前确认: 排插已上电 + 第三个臂的航空插头已接好 启动三路相机
conda activate aloha
roslaunch /home/agilex/cobot_magic/aloha-devel/Piper-AVP-Teleop/multi_arm_launch_tools/launch/multi_camera_shuai.launch
```


```bash
# 启动中路相机 到预设位置
conda activate aloha
cd /home/agilex/cobot_magic/aloha-devel/Piper-AVP-Teleop
python teleop/fix_mid_camera_pose.py
```

## 训练动作表示约定

我们现在推荐给 VLA/LeRobot loader 使用的动作表示是统一坐标系下的 hybrid target:

```text
left  arm action: observations/ee_pose_in_unified/left/quat
right arm action: observations/ee_pose_in_unified/right/quat
mid   arm action: observations/camera_pose_in_unified/cam_front/quat
gripper action : action[:, 6] 和 action[:, 13]  # 只用左右 gripper,不用中臂 gripper
```

所有 pose 都在 unified frame 下:

```text
unified frame = mid_base 朝向,原点在 mid_base 下方 25cm
quat format   = x, y, z, qx, qy, qz, qw
quaternion    = xyzw
unit          = m, rad
```

也就是说左右两只手监督“EEF 去哪里”,中间臂监督“前置相机看哪里”。中臂执行时需要用 hand-eye 把 camera pose 反推出 mid EEF pose,再走 IK:

```text
T_unified_from_mid_ee = T_unified_from_cam_front @ inverse(T_mid_ee_from_cam_front)
```

本地真机检查脚本:

```bash
conda activate aloha
cd /home/agilex/cobot_magic/aloha-devel/Piper-AVP-Teleop
python data_collect/play_data_hybrid_unified_ik.py /path/to/episode_0.hdf5 \
    --pose_source quat \
    --execute \
    --max_joint_step 0.05
```

LeRobot 转换会保留这些训练需要的信息:

```text
action.hybrid.left_ee_pose_in_unified.quat
action.hybrid.right_ee_pose_in_unified.quat
action.hybrid.mid_camera_pose_in_unified.quat
action.hybrid.gripper
observation.camera_pose_in_unified.{quat,rpy,matrix}.{cam_front,cam_left,cam_right}
meta/camera_info.json  # K,D,R,P,width,height,FOV,distortion_model
```


数据收集模式


```bash
# 每次开机/重启后必须做;脚本里以 USB bus-info 识别 4 个 CAN 模块
# (左臂 / 右臂 / 中间臂 / 底盘)
cd /home/agilex/cobot_magic/aloha-devel/Piper-AVP-Teleop/
bash multi_arm_launch_tools/can_config_shuai.sh
source /home/agilex/cobot_magic/Piper_ros_private-ros-noetic/devel/setup.bash
```

```bash
# 启动机械臂
conda activate aloha
roslaunch /home/agilex/cobot_magic/aloha-devel/Piper-AVP-Teleop/multi_arm_launch_tools/launch/start_ms_piper_3arm_collect.launch
```

```bash
#启动相机
conda activate aloha
roslaunch /home/agilex/cobot_magic/aloha-devel/Piper-AVP-Teleop/multi_arm_launch_tools/launch/multi_camera_shuai.launch
```

```bash
# 启动中路相机 到预设位置
conda activate aloha
cd /home/agilex/cobot_magic/aloha-devel/Piper-AVP-Teleop
python teleop/fix_mid_camera_pose.py
```

