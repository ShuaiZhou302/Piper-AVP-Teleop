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

---

## 数据采集模式 (左右从动 / 中间主动)

替代上面第 2 步的"统一 mode/auto_enable"启动方式。给三个臂**分别**配置:

- **左 / 右臂**:`mode=0`(读物理主臂消息转发)、`auto_enable=false`(motor 不上电,手动拖动)。
  操作者用双手拖动左右主臂,puppet 自动跟随,作为数据采集的演示源。
- **中间臂**:`mode=1`(订阅 `/master/joint_mid` 当指令驱动 puppet)、`auto_enable=true`
  (motor 自动使能)。`eef_avp_control_singlearm.py --arm m` 发命令到 `/master/joint_mid`,
  中臂跟着头动。

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

#另外终端启动相机
conda activate aloha
roslaunch /home/agilex/cobot_magic/aloha-devel/Piper-AVP-Teleop/multi_arm_launch_tools/launch/multi_camera_shuai.launch
```

启动后另开终端跑 AVP teleop 控制中臂:

```bash
conda activate aloha
cd /home/agilex/cobot_magic/aloha-devel/Piper-AVP-Teleop/teleop
python eef_avp_control_singlearm.py --arm m --smooth_alpha 0.5
```

参数都有默认值,通常不需要传。若需调整:

```bash
# 例如:暂时关闭中臂自动使能(motor 不上电、安全调试):
roslaunch ... start_ms_piper_3arm_collect.launch mid_auto_enable:=false
```

**注意**:`mode` 和 `auto_enable` 的耦合在 `piper_start_ms_node.py:58` 强制写死 ——
**只有 mode=1 时 auto_enable 才会真正生效**。这就是为什么左右臂的 `lr_auto_enable=false`
其实是冗余的(mode=0 下无论传 true/false 都不会自动使能)。

---

## 完整数据采集流程(4 终端)

修改 `data_collect/collect_data_3arm.sh` 顶部的参数(任务名、描述、保存路径、episode 数、最大帧数)。
然后**按下面顺序起 4 个终端**:

```bash
# 终端 1 — 启动 3 臂(左右 teach、中间 ROS 驱动)
conda activate aloha
roslaunch /home/agilex/cobot_magic/aloha-devel/Piper-AVP-Teleop/multi_arm_launch_tools/launch/start_ms_piper_3arm_collect.launch
```

```bash
# 终端 2 — 启动 3 路相机
conda activate aloha
roslaunch /home/agilex/cobot_magic/aloha-devel/Piper-AVP-Teleop/multi_arm_launch_tools/launch/multi_camera_shuai.launch
```

```bash
# 终端 3 — AVP teleop 控制中间臂
conda activate aloha
cd /home/agilex/cobot_magic/aloha-devel/Piper-AVP-Teleop/teleop
python eef_avp_control_singlearm.py --arm m
```

```bash
# 终端 4 — 数据采集(订阅 /teleop/state,等 ENGAGED 才开始录)
conda activate aloha
cd /home/agilex/cobot_magic/aloha-devel/Piper-AVP-Teleop/data_collect
bash collect_data_3arm.sh
```

戴 AVP,Safari 进 immersive,按下面**每条 episode 重复**:

触发手势统一改成**双手同时捏**(左右手各自拇指+中指一起捏),单手动作不会触发任何状态切换 —— 这样 teleop 时单手做夹爪/抓取动作就不会误触发暂停。

| 步骤 | 手势 | HUD 状态 | 含义 |
|---|---|---|---|
| 1 | **双手** 拇指+中指 同时捏一下 | `IDLE → ENGAGED` (绿) | 锁定头部原点,开始 teleop **+ 数据采集自动开始** |
| 2 | (做任务,头/手动)| `ENGAGED` 持续 | 录帧中 |
| 3 | **双手** 同时捏一下 | `ENGAGED → DISARMED` (橙) | 进入暂停待确认,启动 4 秒长按计时 |
| 4a | 保持双手捏住满 4 秒 | `DISARMED → IDLE` (灰) | 确认暂停 + 数据采集自动停止 + 存盘 |
| 4b | 4 秒内任一手松开 | `DISARMED → ENGAGED` (绿) | 误触撤销,继续 teleop(帧不丢) |
| 5 | sh 脚本 sleep 3 秒后自动启下一条 |  |  |

**特点**:
- 中臂(mid)随头动,左右 puppet 跟人手动主臂,自动录入
- HDF5 单文件每条 episode,在 `$DATASET_DIR/$TASK_NAME/episode_<i>.hdf5`
- 录到的字段:joint state × 3 臂、EE pose(四元数 + RPY)× 3 臂、相机 × 3、master action × 3 臂
- 暂停用"双手捏 + 按住 4 秒"两道门:双手捏避免单手夹爪动作误触,4 秒长按防止两手在 teleop 中偶然靠近被读成双手捏

**手势检测的额外保护**:
- pinch 距离用 Schmitt-trigger hysteresis(close < 0.03 m, open > 0.04 m)防抖
- 当手出 AVP 视野 → 系统检测 landmarks 冻结 → 把 pinch 距离顶到 "open" 防误触发(HUD 上对应行显示红色 `(stale)`)
- 详细见 `avp/avp_gesture_test.py` 里的 `GestureStateMachine` 和 `HandFreshness` 类

---

## 数据回放(验证采集数据)

把录下来的 HDF5 喂给三个 puppet 臂回放,检查录到的 action 序列是不是真的能复现任务。

**前提**:必须用 **inference 模式 launch**(三臂都 `mode=1` + `auto_enable=true`),不能用 collect 模式那个 —— collect 里的左右臂 `mode=0`,不接受 ROS 命令。

```bash
# 终端 1 — 启动 3 臂(inference 模式,三臂都接受 /master/joint_<arm> 命令)
conda activate aloha
roslaunch /home/agilex/cobot_magic/aloha-devel/Piper-AVP-Teleop/multi_arm_launch_tools/launch/start_ms_piper_3arm.launch mode:=1 auto_enable:=true
```

```bash
# 终端 2 — 回放某条 episode
conda activate aloha
cd /home/agilex/cobot_magic/aloha-devel/Piper-AVP-Teleop/data_collect
python play_data.py /home/agilex/data_shuai_3arm/three_arm_pickup/episode_0.hdf5
```

**流程**:

| 步骤 | 操作 | 行为 |
|---|---|---|
| 1 | 启动后打印 task / shape / frame_rate | 自检 |
| 2 | 显示三臂"当前位姿 vs 第一帧目标" | 让你看清楚要往哪儿挪 |
| 3 | **回车确认** | 同时把 3 臂以最大 0.3 rad/s 速度 ramp 到起点位姿 |
|   |   | - mid → `INITIAL_ARM_JOINTS`(已知安全锚点) |
|   |   | - left / right → `action[0]` 对应切片 |
| 4 | **再回车** | 按录制时的 frame_rate 重播 actions[0..T-1] |
| 5 | 播放完 / Ctrl-C | 退出 |

**可调参数**:
```bash
python play_data.py path/to/ep.hdf5 \
    --ramp_speed_rad_s 0.5     # 默认 0.3 rad/s,觉得慢可以加快
    --frame_rate 15            # 覆盖回放帧率(默认从 HDF5 attrs 读)
    --no_confirm               # 跳过两次回车确认(确认安全后再用)
```

**安全注意**:
- 桌面上的物体最好和录制时一致,不然胳膊会"穿过去"或者撞到
- ramp 速度故意慢(0.3 rad/s ≈ 17°/s),距离越远耗时越长但永远不会超速
- 首次回放新数据时建议手放急停旁边