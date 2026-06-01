# 硬件基线档

**当前生效状态**: 使用分线器(2026-05-08 切换并验证 OK)
**用途**: 任何 USB 接线变动后,按本文档对照 bus-info 即可恢复;两份基线分别记录"直插主机"和"分线器"两套配置

---

## 0. 当前(分线器)配置 — 2026-05-08 验证通过

**物理拓扑**:
- 分线器**上行线**插主机 **Port 13**(原左臂位置)— 这个口已贴标签"分线器专用,勿动"
- **左臂 CAN** 插分线器 → 经过分线器内部级联 hub → bus-info `1-13.1.2:1.0`
- **右臂 CAN** 插分线器 → 经过分线器内部级联 hub → bus-info `1-13.1.3:1.0`
- **底盘 CAN** 未动,仍直插主机 Port 4 → bus-info `1-4:1.0`
- **三个相机** 未动,仍各自直插原主机口(Port 3/5/6),靠序列号识别,不受影响

| CAN 接口名 | bus-info | 物理位置 | 模块序列号 | 波特率 | 用途 |
|---|---|---|---|---|---|
| `can_left`  | `1-13.1.2:1.0` | 分线器(上行→主机 Port 13) | `004A00405443570F20393433` | 1 Mbps | 左臂 |
| `can_right` | `1-13.1.3:1.0` | 分线器(上行→主机 Port 13) | `001700355443570A20393433` | 1 Mbps | 右臂 |
| `can0`      | `1-4:1.0`      | 主机 Port 4(直连) | `004500265631511820313857` | 500 kbps | 松灵 AGV 底盘 |

**can_config.sh 当前生效行**(第 111-119 行附近):
```bash
USB_PORTS["1-13.1.2:1.0"]="can_left:1000000"
USB_PORTS["1-13.1.3:1.0"]="can_right:1000000"
USB_PORTS["1-4:1.0"]="can0:500000"
```

**未来扩展第三个臂**:把第三根 CAN 线插分线器空闲口,跑 `sudo ethtool -i canX` 拿新 bus-info(预期 `1-13.1.X:1.0`),在 can_config.sh 加一行 + `EXPECTED_CAN_COUNT=4`。

**回滚记录**: can_config.sh 内部已把"分线器之前"的两行硬编码作为注释保留,需要回滚时直接把那两行注释解开、新两行注释掉即可

---

## 1. 旧配置(直插主机)— 2026-05-07 快照

**用途**: 万一分线器有任何问题,可以拔掉分线器把 6 根线直接插回主机原位,然后把 can_config.sh 恢复成下面的硬编码即可回到已知工作状态

### CAN 模块映射(直插主机版)

| CAN 接口名 | bus-info | 物理 USB 端口 | 模块序列号 | 波特率 | 用途 |
|---|---|---|---|---|---|
| `can_left`  | `1-13:1.0` | Bus 1 Port 13 (root hub 直连) | `004A00405443570F20393433` | 1 Mbps | 左臂 |
| `can_right` | `1-12:1.0` | Bus 1 Port 12 (root hub 直连) | `001700355443570A20393433` | 1 Mbps | 右臂 |
| `can0`      | `1-4:1.0`  | Bus 1 Port 4  (root hub 直连) | `004500265631511820313857` | 500 kbps | **松灵 AGV 底盘**(经 candump 验证,见 ID 0x221/0x240/0x251/0x311) |

> **注意**: 三个 candleLight CAN 适配器**全部直连主机 root hub**,bus-info 都是 `1-X:1.0` 单层格式。
> 一旦经过 USB 分线器,bus-info 会变成 `1-X.Y:1.0` 这种多一段的格式。

### can_config.sh 当前硬编码(第 111-113 行)
```bash
USB_PORTS["1-13:1.0"]="can_left:1000000"
USB_PORTS["1-12:1.0"]="can_right:1000000"
USB_PORTS["1-4:1.0"]="can0:500000"
```
`EXPECTED_CAN_COUNT=3`(第 95 行)— 三个全在位才不报错。

---

## 2. 相机映射(靠序列号识别,不依赖 USB 端口)

| 角色 | topic 前缀 | 序列号 | 当前 USB 路径 |
|---|---|---|---|
| 前 (front) | `f` | `CC1T353011F` | Bus 1 DevPath 3.2 |
| 右 (right) | `r` | `CC1T35300D1` | Bus 1 DevPath 5.2 |
| 左 (left)  | `l` | `CC1T353010Z` | Bus 1 DevPath 6.2 |

> 三台 Dabai DC1 各占一个 root hub 端口(Port 3 / 5 / 6),内部自带 hub(所以 DevPath 是 `X.2`)。
> 启动文件 `multi_camera.launch` 按序列号匹配,**换 USB 口不影响**。

---

## 3. 完整 USB 拓扑(`lsusb -t` 快照)

```
/:  Bus 02.Port 1: Dev 1, Class=root_hub, Driver=xhci_hcd/10p, 20000M
/:  Bus 01.Port 1: Dev 1, Class=root_hub, Driver=xhci_hcd/16p, 480M
    |__ Port 2: Dev 2, If 0, Class=Hub, Driver=hub/4p, 480M
        |__ Port 1: Dev 4, If 0, Class=Hub, Driver=hub/4p, 480M
        |__ Port 4: Dev 6, If 0, Class=Vendor Specific Class, Driver=cp210x, 12M  # CP2102N USB-UART
    |__ Port 3: Dev 3, If 0, Class=Hub, Driver=hub/4p, 480M                       # 前置相机 hub
        |__ Port 4: Dev 9, If 0, Class=Vendor Specific Class
        |__ Port 2: Dev 7, Class=Video                                            # CC1T353011F (前)
    |__ Port 4: Dev 5, Class=Vendor Specific Class, Driver=gs_usb, 12M            # can0 (1-4:1.0)
    |__ Port 5: Dev 8, If 0, Class=Hub, Driver=hub/4p, 480M                       # 右相机 hub
        |__ Port 4: Dev 13, Class=Vendor Specific Class
        |__ Port 2: Dev 11, Class=Video, Driver=uvcvideo                          # CC1T35300D1 (右)
    |__ Port 6: Dev 10, If 0, Class=Hub, Driver=hub/4p, 480M                      # 左相机 hub
        |__ Port 4: Dev 16, Class=Vendor Specific Class
        |__ Port 2: Dev 14, Class=Video, Driver=uvcvideo                          # CC1T353010Z (左)
    |__ Port 9: Dev 12, Class=Hub, Driver=hub/4p, 480M                            # (空 hub)
    |__ Port 11: Dev 19, Class=Wireless, Driver=btusb                             # 蓝牙
    |__ Port 12: Dev 17, Class=Vendor Specific Class, Driver=gs_usb, 12M          # can_right (1-12:1.0)
    |__ Port 13: Dev 18, Class=Vendor Specific Class, Driver=gs_usb, 12M          # can_left  (1-13:1.0)
```

---

## 4. 其他 USB 设备

| 设备 | 序列号 | USB 路径 | 备注 |
|---|---|---|---|
| CP2102N USB-UART Bridge | `b6e253cf6fd9ee11b4cdad4c37b89984` | Bus 1 DevPath 2.4 | 串口桥(用途待定 — 可能是夹爪/IMU) |

---

## 5. 出问题时的恢复步骤

1. **物理恢复**: 把所有 USB 线全部从分线器拔下,直接插回主机机箱
   - 三个 CAN 模块:位置不必精确,但必须三个都直插 root hub(不经过 hub),且总数 = 3 个
   - 一旦插好后,跑下面这条命令,bus-info 必须能匹配上表 1 的三行
2. **验证 bus-info**:
   ```bash
   for iface in $(ip -br link show type can | awk '{print $1}'); do
       echo -n "$iface -> "; sudo ethtool -i "$iface" | grep "bus-info"
   done
   ```
   期望输出:
   ```
   canX -> bus-info: 1-13:1.0   # → can_left
   canX -> bus-info: 1-12:1.0   # → can_right
   canX -> bus-info: 1-4:1.0    # → can0
   ```
   (重启或重插后接口名可能是 can0/can1/can2,不重要,can_config.sh 会按 bus-info 重命名)
3. **恢复 can_config.sh**: 当前已是基线值,无需修改。如果改坏了,把第 111-113 行恢复成:
   ```bash
   USB_PORTS["1-13:1.0"]="can_left:1000000"
   USB_PORTS["1-12:1.0"]="can_right:1000000"
   USB_PORTS["1-4:1.0"]="can0:500000"
   ```
4. **重跑初始化**:
   ```bash
   cd /home/agilex/cobot_magic/Piper_ros_private-ros-noetic/
   bash can_config.sh
   ```
   看到 "所有 CAN 接口已成功重命名并激活" 即恢复完成。

---

## 6. 切到分线器后需要做的事(参考)

- 相机:不用改任何东西
- 机械臂 launch:不用改(只看 `can_left` / `can_right` 名称)
- **唯一要改**:`can_config.sh` 第 111-113 行的 bus-info key,换成新拓扑下的实际值
  - 用 `sudo ethtool -i canX` 一个一个查新 bus-info
  - 区分哪根线是左/右/第三个 CAN,可以靠序列号(见上表 1 第 4 列)
