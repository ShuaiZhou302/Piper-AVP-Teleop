# AVP → Piper EE Teleop 实现计划

输入:AVP `head_matrix`(Vuer / WebXR)
输出:Piper 右臂 EE pose,以 `INITIAL_ARM_POSE` 为锚点
进程结构:单 Python 进程,Vuer 子进程 + 主线程跑 rospy / IK / publish

---

## 已锁定决定

- [x] **头位姿**驱动 EE,不是手位姿
- [x] **只控右臂**。左臂 echo `/puppet/joint_left` → `/master/joint_left`
- [x] **夹爪固定 0.1**(全开),不进 teleop
- [x] **位置缩放 1:1**,做成 `--scale` 超参,默认 1.0
- [x] **姿态跟随**:头怎么转 EE 就怎么转
- [x] **启动归零**:开机先把右臂自动挪到 `INITIAL_ARM_POSE`,再等手势
- [x] **手势只用于状态机**:
  - 左手 拇指+中指 捏(rising edge) → 锁原点(IDLE → LOCKED)
  - 右手 拇指+中指 捏住 → engage(LOCKED ↔ ENGAGED,deadman)
- [x] **操作者朝向**:站姿正对 base 前方,Readme 里写约束
- [x] **旋转叠加**:**世界系**(`ΔR_world @ T_initial`)
- [x] **轴重排放在控制层**:`eef_avp_control.py` 自己做,`avp_min_test.py` 保持 raw

### `R_avp_to_piper`

```
[[ 0,  0, -1],
 [-1,  0,  0],
 [ 0,  1,  0]]
```

把 AVP 世界系(右/上/后)映射到 Piper 世界系(前/左/上)。

---

## Step A — 在 `avp_min_test.py` 里加手势状态机

**目标**:把手势检测调到可靠后再接机械臂,免得 Step B 调试时同时面对手势 + IK 两个变量。

- [ ] 读 `vr.left_landmarks`、`vr.right_landmarks`
- [ ] 计算 `‖[4] - [14]‖`(拇指 ↔ 中指)左右两个值
- [ ] 状态枚举:`IDLE` / `LOCKED` / `ENGAGED`
- [ ] 状态转移:
  - `IDLE → LOCKED`:左手距离 < 0.03 m(rising edge)
  - `LOCKED → ENGAGED`:右手距离 < 0.03 m(持续触发)
  - `ENGAGED → LOCKED`:右手距离 > 0.04 m(松开,留点 hysteresis 防抖)
- [ ] HUD 上把当前 STATE 字符串渲染出来
- [ ] 戴头显验证 3 个状态切换都稳

---

## Step B — 新建 `eef_avp_control.py`

**目标**:头驱动 EE 的端到端 teleop 闭环。

### Bootstrap
- [ ] Vuer / `OpenTeleVision`(复用 `avp_min_test.py` 的初始化方式)
- [ ] `rospy.init_node('eef_avp_teleop')`
- [ ] 两个 `PinocchioIKSolver`(左 URDF + 右 URDF)
- [ ] Subscribers:`/puppet/joint_left`、`/puppet/joint_right`(IK seed + 左臂 echo 源)
- [ ] Publishers:`/master/joint_left`、`/master/joint_right`

### 启动归零
- [ ] 拿到第一帧 joint feedback 后,IK:**当前 → `INITIAL_ARM_POSE`**
- [ ] 关节空间线性插值,30 Hz 发,挪个几秒到位
- [ ] 归零未完成不允许进 LOCKED 状态

### 主循环 30 Hz
- [ ] 读 `vr.head_matrix`
- [ ] 跑 Step A 的状态机
- [ ] 如果 `STATE == ENGAGED`:
  ```
  delta_pos_avp     = head_pos_now - head_pos_initial
  delta_R_world_avp = R_head_now @ R_head_initial.T

  delta_pos_piper   = R @ delta_pos_avp * scale
  delta_R_piper     = R @ delta_R_world_avp @ R.T

  target_pos = INITIAL_ARM_POSE.pos + delta_pos_piper
  target_R   = delta_R_piper @ R_initial_arm
  ```
  IK → 关节角 → publish
- [ ] `IDLE` / `LOCKED`:发当前右臂 joint(机械臂保持原位)
- [ ] 始终:echo `/puppet/joint_left` → `/master/joint_left`,gripper 两侧都填 0.1

### 安全
- [ ] IK 失败 → 跳过本帧 publish(参考 keyboard control 现有处理)
- [ ] Vuer 断连 / `head_matrix` 超过 0.5 s 没更新 → 强制掉回 LOCKED

---

## 待定 / 暂不处理

### 1. 旋转叠加:世界系 vs 本地系
V1 用**世界系**(`ΔR_world @ T_initial`)。

- 单一动作:两种写法**完全等价**
- 复合动作(roll + pitch 之类不同轴叠加):**会差几度**,看哪个更"跟手"
- 世界系直觉:"我头怎么动,Piper 世界里 EE 就怎么动"
- 本地系直觉:"EE 从自己的视角去转",当 EE 已经偏离初始姿态后会越发别扭

V1 跑通后实测,觉得歪头 → 抬头那种动作 EE 转得反直觉,就把那一行改成
`T_target = T_initial @ ΔR_body`。一行的事。

### 2. 相机回传(原 TODO #2)
订阅 Piper 右臂相机的 ROS 话题(`astra_camera multi_camera.launch` 起的),把
帧 push 到 `OpenTeleVision` 共享内存,VR 里就能看到 EE 第一视角而不是空白
HUD。和 teleop 控制完全独立,Step B 跑通后再做。

### 3. 缩放调参
`--scale` 默认 1.0。真实工作空间测下来可能太挤或太跳,计划做一次 0.5 / 1.0 / 1.5 的扫一下。

### 4. 操作者朝向自动对齐
当前假设站姿正对 base 前方。egox 用 head-yaw 自动对齐(开机时拿头部 yaw 把
世界系绕 z 转到"操作者前 = +x"),如果以后要"操作者随便站",移植那段逻辑。
