# multi_arm_launch_tools

shuai 版启动脚本/launch 文件,支持 **3 臂 + 底盘 + 3 相机(新前相机)** 的硬件配置。
每个文件都是原文件的复制 + 微量修改,原文件保持不动。

---

## 启动流程

### 1. CAN 初始化 + ROS 环境

```bash
# 每次开机/重启后必须做;脚本里以 USB bus-info 识别 4 个 CAN 模块
# (左臂 / 右臂 / 中间臂 / 底盘)
cd /home/agilex/cobot_magic/aloha-devel/Piper-AVP-Teleop/
bash multi_arm_launch_tools/can_config_shuai.sh
source /home/agilex/cobot_magic/Piper_ros_private-ros-noetic/devel/setup.bash
```

### 2. 启动 3 个机械臂(inference 模式)

```bash
# 启动前确认: 排插已上电 + 第三个臂的航空插头已接好
conda activate aloha
roslaunch /home/agilex/cobot_magic/aloha-devel/Piper-AVP-Teleop/multi_arm_launch_tools/launch/start_ms_piper_3arm.launch mode:=1 auto_enable:=true
```

成功标志:三行 `使能状态: True`(每个臂一行),无 `SEND_MESSAGE_FAILED` 报错。

### 3. 启动相机(自动用新前相机 SN)

```bash
conda activate aloha
roslaunch /home/agilex/cobot_magic/aloha-devel/Piper-AVP-Teleop/multi_arm_launch_tools/launch/multi_camera_shuai.launch
```


# 4. 机械臂单臂 eef pose track 测试

```bash
python3 teleop/eef_keyboard_control_singlearm.py             # 启动时交互式问选哪个臂
python3 teleop/eef_keyboard_control_singlearm.py --arm m     # 直接指定 mid
python3 teleop/eef_keyboard_control_singlearm.py --arm left  # 直接指定 left
```

# 5. 机械臂单臂 joint pose track 测试

```bash
python3 teleop/joint_keyboard_control_singlearm.py             # 交互式问选哪个臂
python3 teleop/joint_keyboard_control_singlearm.py --arm m     # mid
python3 teleop/joint_keyboard_control_singlearm.py --arm l     # left
python3 teleop/joint_keyboard_control_singlearm.py --arm r     # right
```


# 6. 机械臂单臂 apple vision pro 测试
```bash
conda activate aloha
cd /home/agilex/cobot_magic/aloha-devel/Piper-AVP-Teleop/teleop
python eef_avp_control_singlearm.py --arm m
```