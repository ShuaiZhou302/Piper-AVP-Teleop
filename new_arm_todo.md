# Piper-AVP-Teleop TODO / 状态

## 硬件基线(2026-05-09)

| 项 | SN | bus 位置 | 说明 |
|---|---|---|---|
| 中间臂 CAN | `0033002A5246571220393733` | `1-13.1.4:1.0` | 分线器第 4 口 |
| 新前相机 | `CC1B25100JX` | bus 1-3.2 | 替代原 `CC1T353011F`,物理位置不变 |

详细基线 → [HARDWARE_BASELINE.md](HARDWARE_BASELINE.md)

---

## 设计原则

1. **不破坏原有 3 臂工作流** —— 只有 2 臂 + 3 旧相机的机器仍按原 Readme 跑
2. **以 SN 为单位识别硬件**,不依赖物理 USB 端口编号
3. **新硬件存在 → 用新配置;不存在 → 回退原有 3 臂行为**(自适应)
4. **我们的扩展脚本和原脚本并存**,从 [multi_arm_launch_tools/](multi_arm_launch_tools/) 入口

---

## Phase A — 让新硬件在 ROS 里可见 ✅ 完成

- [x] **A.1 CAN 初始化**:[multi_arm_launch_tools/can_config_shuai.sh](multi_arm_launch_tools/can_config_shuai.sh) 按 SN 识别 4 个 CAN
- [x] **A.2 相机启动**:[multi_arm_launch_tools/launch/multi_camera_shuai.launch](multi_arm_launch_tools/launch/multi_camera_shuai.launch),自适应前相机 SN
- [x] **A.3 机械臂启动**:[multi_arm_launch_tools/launch/start_ms_piper_3arm.launch](multi_arm_launch_tools/launch/start_ms_piper_3arm.launch)(推理模式)+ [start_ms_piper_3arm_collect.launch](multi_arm_launch_tools/launch/start_ms_piper_3arm_collect.launch)(数据采集混合模式,左右 teach + 中间 ROS 驱动)
- [x] **A.4 验收**:`rostopic list` 看到 6 个 joint topic + 3 个 end_pose + 3 个相机 ✓

---

## Phase B — 数据采集 ✅ 完成

- [x] 单臂键盘 EE / Joint 控制:[teleop/eef_keyboard_control_singlearm.py](teleop/eef_keyboard_control_singlearm.py)、[teleop/joint_keyboard_control_singlearm.py](teleop/joint_keyboard_control_singlearm.py)
- [x] 3 臂数据采集脚本:[data_collect/collect_data_3arm.py](data_collect/collect_data_3arm.py) + [data_collect/collect_data_3arm.sh](data_collect/collect_data_3arm.sh)
- [x] HDF5 schema:joint × 3 臂 + EE pose 四元数 / RPY × 3 臂 + 相机 × 3 + action(master)× 3 + base_action
- [x] **AVP 手势驱动 episode 生命周期**:ENGAGED 入则开始,LOCKED 退则存盘
- [x] task_name / task_description / dataset_dir 全部从 .sh 参数化,不需要交互输入

---

## Phase C — AVP 遥操作

- [x] AVP ↔ PC 通信:Vuer / WebXR ([avp/](avp/) 文件夹整套)
- [x] 手势状态机 4 态:IDLE → LOCKED → ARMED → ENGAGED(`avp/avp_gesture_test.py:GestureStateMachine`)
- [x] **双 pinch 防误触发**:LOCKED → ARMED → 2 秒内再 pinch → ENGAGED;单 pinch 退出
- [x] **手出 FOV 检测**:`HandFreshness` 类,landmarks 冻结 333 ms 自动失效
- [x] 头位姿驱动 mid 臂:[teleop/eef_avp_control_singlearm.py](teleop/eef_avp_control_singlearm.py)
- [x] 控制层轴重排 `R_AVP_TO_PIPER`,世界系叠加
- [x] **关节空间锚点**:`INITIAL_ARM_JOINTS` 直接定关节,FK 算 EE pose 作 delta tracking 锚点(避免 IK 多解)
- [x] IK seed 用上一帧命令(非反馈),配 `--max_joint_step` 关节速度限位,大幅减小跳变
- [x] Boot ramp:启动时自动从当前关节角线性插值到 `INITIAL_ARM_JOINTS`
- [x] 不回 echo 反馈,持续 publish target_q(防 fight-back loop)
- [x] HUD 显示 state / 目标 pose / IK clip 状态 / 相机回传

### 已知 / 待优化(非阻塞)
- [ ] **相机画质**:`/camera_f` 当前画质偏差,后续接更好的相机
- [ ] **camera passthrough 延迟**:在 HKUSTGZ 公共 Wi-Fi 时偶有积压,有空再调 fps / quality
- [ ] **操作者朝向自动对齐**:目前需操作者站位正对 base 前方;egox 风格的 head-yaw 自动对齐目前没实现
- [ ] **旋转叠加方式**:V1 用世界系,如果复合动作下 EE 转得反直觉可切本体系(一行修改)

---

## Phase D — 推理 / 模型部署(未开始)

- [ ] 决定推理时 mid 臂的角色(被动观察 / 主动控制)
- [ ] 扩展 inference 脚本接入 3 臂 + 3 相机数据
- [ ] 利用 `collect_data_3arm.py` 产出的数据集训练模型

---

## Phase E — 眼动追踪(搁置)

详见之前讨论,简要结论:
- AVP + WebXR **不能拿持续 gaze**(Apple 隐私限制,只在 pinch 时给方向)
- 若必须连续 gaze,**Quest Pro**(可在 HKUSTGZ CMA lab 借)是更好的选择,WebXR 原生支持,代码改动 < 100 行
- 当前 head 方向已作为"注意力代理",对 teleop 场景足够用

---

## 命名决策(已敲定)

| 项 | 决策 |
|---|---|
| 第 3 个臂的 CAN 接口名 | `can_mid` ✓ |
| ROS topic 前缀 | `/master/joint_mid` / `/puppet/joint_mid` / `/puppet/end_pose_mid` ✓ |
| 新前相机 topic | 沿用 `/camera_f`(物理位置没变) ✓ |
