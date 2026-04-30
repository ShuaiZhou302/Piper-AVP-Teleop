# Apple Vision Pro 通信 Setup(Vuer / OpenTeleVision 方案)

不需要 Mac、不需要 Xcode、不需要 Apple Developer 账号。
PC 起一个 HTTPS+WSS 的 Vuer 服务,AVP 用 Safari 进 WebXR 沉浸式 session,
PC 端通过 Vuer 的 `HAND_MOVE` / `CAMERA_MOVE` 事件拿到头戴和双手的 4×4 位姿。

```
[Apple Vision Pro]                                     [PC (Linux)]
 Safari (WebXR session)  ◄── HTTPS:8012 + WSS:8012 ──► Vuer server
   └─ 戴上头显进入 immersive                              └── OpenTeleVision
       └─ 头/手位姿事件                                        ├── .head_matrix    (4,4)
                                                              ├── .left_hand      (4,4)
                                                              ├── .right_hand     (4,4)
                                                              ├── .left_landmarks (25,3)
                                                              └── .right_landmarks(25,3)
```

参考实现:[../egox/data_collect/tele_vision.py](../egox/data_collect/tele_vision.py)。

---

## 1. PC 端环境

```bash
conda create -n avp python=3.10 -y
conda activate avp
pip install vuer==0.0.31rc7 aiohttp==3.9.5 aiohttp_cors==0.7.0 numpy
```

`vuer` 版本要按 egox `requirements.txt` 锁定,新版本 schema 改过,事件结构会对不上。

> **Python 3.8 兼容(本机 aloha env 走的路线)**
> 默认安装会拉到 `params_proto-3.x`,3.x 用了 PEP 585 的 `tuple[Any, bool]`,3.8 上 import 直接炸。
> 解决:`pip install 'params_proto<3.0'`(实测 2.13.2 在 Python 3.8 + vuer 0.0.31rc7 上 import OK)。

---

## 2. 生成自签证书(mkcert)

WebXR 在 Safari 里只对 **HTTPS 起源**启用,所以 PC 这边必须有合法的 TLS 证书。
自签 CA + 给本机 IP 签一张证书是最简方案,工具用 [mkcert](https://github.com/FiloSottile/mkcert)。

### 2.1 装 mkcert

```bash
# Ubuntu / Debian
sudo apt install -y libnss3-tools
curl -L https://github.com/FiloSottile/mkcert/releases/latest/download/mkcert-v1.4.4-linux-amd64 \
     -o /usr/local/bin/mkcert
sudo chmod +x /usr/local/bin/mkcert
```

### 2.2 生成根 CA + 服务器证书

```bash
cd /home/agilex/cobot_magic/aloha-devel/Piper-AVP-Teleop/avp
mkcert -install                                # 在 PC 系统信任链里装根 CA
mkcert -cert-file cert.pem -key-file key.pem \
       <PC_LAN_IP> localhost 127.0.0.1         # 把 PC 在路由器里的 IP 也签进去
```

完成后该目录会有 `cert.pem` / `key.pem`,Vuer 启动时用它们。
mkcert 根 CA 文件在 `$(mkcert -CAROOT)/rootCA.pem`,**下一步要把它装到 AVP**。

> 注意:`<PC_LAN_IP>` 必须是 AVP 实际能访问到 PC 的那个 IP(同一 Wi-Fi 段)。证书绑死 IP,PC 换网络后要重签。

---

## 3. 把 mkcert 根 CA 装到 AVP(一次性)

这一步完成后 AVP Safari 才会信任你 PC 的 HTTPS。流程类似 iOS。

1. PC 启动一个临时 HTTP 服务托管根 CA:
   ```bash
   cd "$(mkcert -CAROOT)"
   python3 -m http.server 8000
   ```
2. AVP Safari 打开 `http://<PC_LAN_IP>:8000/rootCA.pem` → 弹出 "下载配置描述文件"
3. AVP `Settings → General → VPN & Device Management → 已下载描述文件 → 安装`(可能要输 AVP 密码两次)
4. AVP `Settings → General → About → Certificate Trust Settings` → 找到刚装的 mkcert CA → 打开"启用对此根证书的完全信任"

> visionOS 的菜单文案可能略有出入,认 "Profile" / "Certificate Trust" 即可。

---

## 4. 在 AVP 上启用 WebXR

某些 visionOS 版本默认关掉 WebXR,要手动开:

`Settings → Apps → Safari → Advanced → Feature Flags` →
- `WebXR Device API` → On
- `WebXR Hand Input Module` → On
- `WebXR Augmented Reality Module` → On(可选)

打开后**重启 Safari**(完全退出再开)。

---

## 5. 起 Vuer 服务 + AVP 进 WebXR

PC 端跑最小测试脚本(下一节给):

```bash
conda activate aloha     # 或者你装了 vuer 的那个 env
cd /home/agilex/cobot_magic/aloha-devel/Piper-AVP-Teleop/avp
python avp_min_test.py
```

启动后会在 `0.0.0.0:8012` 监听 HTTPS+WSS。

AVP Safari 打开:

```
https://<PC_LAN_IP>:8012?ws=wss://<PC_LAN_IP>:8012
```

页面加载后右下/中央会有 **"Enter VR"** 按钮(WebXR 标准 UI),点它进入沉浸式 session。
进了 immersive 之后 PC 端日志会开始打印头部/手部 4×4 位姿。

---

## 6. 取 pose 数据(API)

`OpenTeleVision` 把 Vuer 事件存进多进程共享数组,主进程通过 property 读:

```python
vr.head_matrix        # (4,4) 头戴位姿,AVP 世界系
vr.left_hand          # (4,4) 左手腕
vr.right_hand         # (4,4) 右手腕
vr.left_landmarks     # (25,3) 左手 25 个关键点(关节)
vr.right_landmarks    # (25,3) 右手 25 个关键点
vr.aspect             # AVP 视场宽高比(标量)
```

字段定义见 [tele_vision.py:180-202](../egox/data_collect/tele_vision.py#L180-L202)。

### 6.1 坐标系
- AVP / WebXR 世界系:**y 朝上**,原点 = 戴头显完成空间定位时锁定的位置。
- 与 IsaacGym / 实机常用的 **z 朝上 / x 前**不同,需要做轴重排。
  egox 里的换轴见 [egox_data_collect.py:1454-1472](../egox/data_collect/egox_data_collect.py#L1454-L1472)。
- 单位:米 / 弧度。

### 6.2 "相对初始位姿"
Vuer 给的是绝对位姿。在用户摆好开始姿势那一刻锁一帧:
```python
T0 = vr.head_matrix.copy()
# 之后每帧:
T_rel = np.linalg.inv(T0) @ vr.head_matrix
```
egox 的精细做法:只用 reset 时刻 head 的 yaw 把世界系绕 z 转成"人脸朝向 = +x",再以 reset 时 head 位置作为原点做减法,见 [egox_data_collect.py:1579-1637](../egox/data_collect/egox_data_collect.py#L1579-L1637)。

---

## 7. 验证流程

1. PC `python avp_min_test.py` 启动,日志看到 `Vuer server running on https://0.0.0.0:8012`。
2. AVP Safari 打开 `https://<PC_LAN_IP>:8012?ws=wss://<PC_LAN_IP>:8012`,**没有证书警告**(有警告说明第 3 步根 CA 没装好)。
3. 点 "Enter VR" 进 immersive。
4. 转头、动手,PC 终端打印的 `head_matrix` / `left_hand` / `right_hand` 4×4 应当跟着变。

---

## 8. 常见坑

- **Safari 一直提示证书不安全 / 进不去**:`mkcert -CAROOT` 那个根 CA 没装到 AVP,或者装了没在 Certificate Trust Settings 里"完全信任"。
- **页面打开但没有 "Enter VR" 按钮**:WebXR feature flag 没开(第 4 节),或者 Safari 没重启。
- **进了 VR 但 `head_matrix` 全是 0 / 单位阵**:Safari WebXR 没真正发事件,常见原因是没进 immersive(只是打开网页)。一定要点 Enter VR。
- **`left_hand` 偶尔跳变到奇怪姿态**:WebXR hand tracking 在手出视野时会回退到上一帧或乱跳,egox 用 `safe_mat_update` 拿上一帧顶替,见 [egox_data_collect.py:1421-1444](../egox/data_collect/egox_data_collect.py#L1421-L1444)。
- **PC 换 IP 后连不上**:证书绑死 IP,要重新 `mkcert -cert-file ... <新IP> ...` 并重启服务。
- **延迟大 / 卡顿**:Wi-Fi 走 5G 频段、PC 和 AVP 同一路由器同段,关掉其他大流量设备。
- **看不到外部世界**:Vuer 是 VR 模式,不是 pass-through。你能在 PC 端把摄像头画面 push 进 Vuer 场景作为背景(参考 [tele_vision.py:88-171](../egox/data_collect/tele_vision.py#L88-L171) 的 `ImageBackground` 用法),但看不到自己房间的真实画面。

---

## 9. AVP 世界系实测约定

本机上根据 `avp_min_test.py` 的 HUD 面板,实际走动 + 摇头测出来的结果。参考姿态是
进 immersive 那一刻、操作者正面朝前的中立位。

### 位置(右手系,AVP 世界系)
- **+x**:操作者**右侧**
- **+y**:**上**(竖直向上)
- **+z**:**后**(指向操作者背后)

这就是 WebXR 标准约定:`-z` 朝前,`+y` 朝上,右手系。

### 姿态(yaw / pitch / roll,见 `mat_to_yaw_pitch_roll_yup`,YXZ 内禀分解)
- **+yaw**:头**向左转**
- **+pitch**:**抬头**(下巴朝上)
- **+roll**:头**向左倾**(向右倾为负)

`left_hand` / `right_hand` 的姿态在同一个 AVP 世界系下,符号约定一致。

要把这些位姿喂给 z-up / x-forward 的机器人世界系(IsaacGym、大多数机械臂控制器),
用 [egox_data_collect.py:1454-1472](../egox/data_collect/egox_data_collect.py#L1454-L1472) 那段轴重排即可。
