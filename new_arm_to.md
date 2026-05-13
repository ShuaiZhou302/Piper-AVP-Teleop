# 第三个臂(mid)接入 TODO

## 当前硬件状态(2026-05-09)

| 项 | SN | bus 位置 | 说明 |
|---|---|---|---|
| 中间臂 CAN(新) | `0033002A5246571220393733` | `1-13.1.4:1.0` | 分线器第 4 口 |
| 新前相机 | `CC1B25100JX` | bus 1-3.2 | 替代原 `CC1T353011F`,物理位置不变 |

详细基线 → [HARDWARE_BASELINE.md](HARDWARE_BASELINE.md)

---

## 设计原则

1. **不破坏原有 `bash can_config.sh` / `multi_camera.launch` / `start_ms_piper.launch` 的 3 臂工作流**
   - 别人 / 别的机器(只有 2 臂 + 3 旧相机)按 Readme 原流程仍可正常启动
2. **以 SN 为单位识别硬件**,而非依赖物理 USB 端口编号
3. **新硬件存在 → 用新配置;不存在 → 回退到原有 3 臂行为**(自适应)
4. **我们的扩展脚本和原脚本并存**,我们走自己的入口

---

## Phase A — 让新硬件在 ROS 里可见(优先做,本次)

提议架构:**新增 3 个自己的脚本/launch 文件,原文件不动**。

### A.1 CAN 初始化

- 新增 `tools/can_config_shuai.sh`(或类似名字),功能:
  - 按 SN 列表识别 4 个 CAN(包含新中间臂),自适应 3 / 4 模式
  - 兼容旧脚本逻辑(本质就是改一下 EXPECTED_CAN_COUNT 和 USB_PORTS,但更智能)
- 原 `Piper_ros_private-ros-noetic/can_config.sh` **完全不动**

### A.2 相机启动

- 新增 `tools/start_cameras_shuai.sh`(或类似名字),功能:
  - 探测 3 个相机的 SN:左固定 `CC1T353010Z`,右固定 `CC1T35300D1`,前**先查 `CC1T353011F` → 不在则用 `CC1B25100JX`**
  - 用 `roslaunch ... camera3_serila_number:=<动态值>` 的方式传参,**不修改原 launch 文件**
- 原 `multi_camera.launch` **完全不动**

### A.3 机械臂启动

- 新增 `piper/launch/start_ms_piper_3arm.launch`(或类似名字),内容:
  - 复制原 launch,加上第 3 个 piper 节点(can_mid)
  - 节点话题:`/master/joint_mid` `/puppet/joint_mid` `/puppet/end_pose_mid`
- 原 `start_ms_piper.launch` **完全不动**

### A.4 验收

- [ ] 跑 `rostopic list` 看到 6 个 joint topic(左右中各 master/puppet)+ 3 个相机 topic
- [ ] `rostopic echo /puppet/joint_mid` 摆动中间臂能看到数据变化

---

## Phase B — 接入数据采集(晚些时候)

- [ ] 阅读 `collect_data/collect_data_shuai.sh` 现有逻辑
- [ ] 自己重写采集脚本,增加 mid 臂状态 + 新前相机的数据通道
- [ ] 验证录下的数据包含 mid 关节角 / 末端位姿

## Phase C — 接入遥操作 / 推理(晚些时候)

- [ ] 决定 mid 臂遥操作方式(键盘 / AVP 跟手 / 主臂从动 / ...)
- [ ] 自己写 teleop 脚本(参考现有 `joint_keyboard_control.py` / `eef_keyboard_control.py`)
- [ ] 决定推理时 mid 臂的角色(被动 / 主动控制)
- [ ] 自己写 inference 脚本扩展

---

## 待对齐的命名决策

- 第 3 个臂的 CAN 接口名 → `can_mid`(默认推荐) / 其他?
- 第 3 个臂的 ROS topic 前缀 → `/master/joint_mid` `/puppet/joint_mid` / 其他?
- 新前相机的 topic prefix → 仍叫 `f`(默认推荐,因为位置没变) / 其他?
