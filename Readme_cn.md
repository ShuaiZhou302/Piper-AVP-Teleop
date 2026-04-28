# Piper 双臂遥操作

## 启动流程

```bash
# 1. CAN 初始化(每次开机/重启后必须做)
cd /home/agilex/cobot_magic/Piper_ros_private-ros-noetic/
bash can_config.sh
source devel/setup.bash
```

```bash
# 2. 启动从臂前先把机械臂断电重启
#    关掉供电插排 → 拔掉主臂航空插头 → 插排上电
conda activate aloha
roslaunch piper start_ms_piper.launch mode:=1 auto_enable:=true
```

```bash
# 3. 启动相机
conda activate aloha
roslaunch astra_camera multi_camera.launch
```

```bash
# 4. 控制脚本(选一个)
cd aloha-devel
bash act/inference.sh                                                                       # 推理
python /home/agilex/cobot_magic/aloha-devel/Piper-AVP-Teleop/teleop/joint_keyboard_control.py  # 关节键盘控制
python /home/agilex/cobot_magic/aloha-devel/Piper-AVP-Teleop/teleop/eef_keyboard_control.py    # 末端键盘控制
```

> ⚠️ 推理时注意安全,行为不正常立即中断代码或断电。

## Joint维度对照表

| 左臂 dim | 右臂 dim | 含义 |
| --- | --- | --- |
| 0 | 7 | 底座 z 轴旋转 |
| 1 | 8 | 底座 joint |
| 2 | 9 | 大臂 joint |
| 3 | 10 | 小臂(以臂为轴旋转) |
| 4 | 11 | 小臂 joint |
| 5 | 12 | 腕部(以臂为轴旋转) |
| 6 | 13 | 夹爪(0–0.1, 关–开) |

## EEF维度对照表

末端执行器位姿 = 平移 (xyz, m) + 姿态 (rpy, rad,sxyz/外旋约定),坐标系是 base → EE。

| 左臂 dim | 右臂 dim | 含义 | 单位 | step 默认值 |
| --- | --- | --- | --- | --- |
| 0  | 7  | x —— EE 在 base frame 下的 X 位置(前后) | m | 0.002 |
| 1  | 8  | y —— EE 在 base frame 下的 Y 位置(左右) | m | 0.002 |
| 2  | 9  | z —— EE 在 base frame 下的 Z 位置(上下) | m | 0.002 |
| 3  | 10 | roll  —— 绕 base X 轴旋转(外旋,sxyz 第 1 步) | rad | 0.01 |
| 4  | 11 | pitch —— 绕 base Y 轴旋转(外旋,sxyz 第 2 步) | rad | 0.01 |
| 5  | 12 | yaw   —— 绕 base Z 轴旋转(外旋,sxyz 第 3 步) | rad | 0.01 |
| 6  | 13 | 夹爪开合(0–0.1, 关–开) | m | 0.002 |

> rpy 是 ROS / 航空标准的 sxyz 约定 —— 等价说法是"绕固定 base 轴依次 X→Y→Z 旋转",或"绕本体当前轴依次 Z→Y→X 旋转(顺序反过来)"。pitch 接近 ±90° 会进入万向锁,届时 roll/yaw 数值会退化耦合。

## EEF 坐标系约定

`/puppet/end_pose_*` 报的位姿是 URDF 里 **joint6 frame**,不是物理夹爪/相机的 frame。eef_keyboard_control.py 在 IK 模型里加了一个静态偏移,把控制对象切换成了**摄像头位姿**(不是夹爪):

| 维度 | 偏移(joint6 局部系下) |
| --- | --- |
| 平移 | `-X 方向 0.05 m` |
| 旋转 | `Ry(-90°)` |

代码位置:[eef_keyboard_control.py:37-50](eef_keyboard_control.py#L37-L50) 的 `addFrame`。

**想看 joint6 在真机上具体在哪儿**:跑 `joint_keyboard_control.py`,选 dim **5** (左臂腕部) 或 **12** (右臂腕部),按 w/s,转动的那个关节就是 joint6。

**想换成夹爪的位姿**:夹爪和摄像头共用同一个旋转偏移(`Ry(-90°)`),只是没有那 5cm 平移。把 [eef_keyboard_control.py:47](eef_keyboard_control.py#L47) 的平移向量改为零即可,旋转那行(line 40)保持不动。

```python
# 当前(摄像头位姿)
ee_off_quat = quaternion_from_euler(0.0, -np.pi / 2.0, 0.0)  # 旋转保持不变
# ...
np.array([-0.05, 0.0, 0.0]),                                  # ← 改这一行

# 改后(夹爪位姿)
ee_off_quat = quaternion_from_euler(0.0, -np.pi / 2.0, 0.0)  # 旋转保持不变
# ...
np.array([0.0, 0.0, 0.0]),                                    # ← 平移置 0
```

改完先用下面的 frame_visualize 命令(`--offset_xyz 0 0 0`)视觉确认一遍再跑真机。

## Frame 可视化工具

修改 `addFrame` 的 xyz/rpy 之前,**先用 `frame_visualize.py` 视觉验证**最终位姿是不是你要的目标。

输出一个交互式 HTML(浏览器拖拽旋转、滚轮缩放):

```bash
# 例:验证当前 addFrame 偏移(摄像头位姿)是否正确
# --xyz / --rpy 是当前驱动报的 joint6 位姿(从 /puppet/end_pose_* 读)
# --offset_xyz / --offset_rpy 是 addFrame 里要填的偏移
python /home/agilex/cobot_magic/aloha-devel/Piper-AVP-Teleop/teleop/frame_visualize.py \
    --xyz 0.061058 -0.003505 0.20627 \
    --rpy -0.0364 1.5377 -0.0939 \
    --offset_xyz -0.05 0 0 \
    --offset_rpy 0 -90 0 --offset_deg \
    --html /tmp/frame_right_offset.html
xdg-open /tmp/frame_right_offset.html
```

图中各 frame 的含义:

| frame | 含义 |
| --- | --- |
| `base` | 机器人基座 |
| `target` | **joint6 当前位姿**(纯 `--xyz/--rpy`,不带偏移) |
| `main*(R-then-T) [SE3]` | **加偏移后的最终位姿**(就是 IK 内部 ee frame) |
| `main*(T-then-R)` | 先平移再旋转的对照(平移与旋转可交换时与上面重合) |

对照真机看 `main*(R-then-T)` 是不是和你期望的位姿(摄像头/夹爪)一致。

**换夹爪示例**(如果以后想控夹爪,先把平移置 0,再视觉调旋转):
```bash
# --offset_xyz 0 0 0 → 偏移退化为只有旋转
python ... --offset_xyz 0 0 0 --offset_rpy 0 -90 0 --offset_deg --html /tmp/gripper.html
```

## 运行环境

所有脚本依赖 conda 的 `aloha` 环境:

| 组件 | 版本 |
| --- | --- |
| Python | 3.8.19 |
| Pinocchio | 3.2.0 |
| CasADi | 3.6.5 |
| NumPy | 1.24.4 |
| SciPy | 1.10.1 |
| ROS | Noetic(系统装) |

启动前记得 `conda activate aloha`。`frame_visualize.py` 额外依赖 `plotly`(用于 `--html` 输出),`pip install plotly` 即可。


