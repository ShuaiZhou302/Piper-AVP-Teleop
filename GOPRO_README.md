# GoPro Hero 12 接入指南

通过 USB 把 GoPro Hero 12 作为额外相机视角接到 Linux,经 `v4l2loopback` 暴露为 `/dev/video42`,后续可接入 ROS 数据采集流程。

依赖第三方脚本 [gopro_as_webcam_on_linux](https://github.com/jschmid1/gopro_as_webcam_on_linux),已克隆到本仓库下 `gopro_as_webcam_on_linux/` 目录。

---

## ⚠️ 关于延迟(必须提前知道)

GoPro webcam 模式有**先天延迟 150-200ms**,这是相机内部 H.264 编码 + UDP 推流的协议开销,**消除不了**。即便我们这边参数调到最优,端到端总延迟也在 ~150-220ms,**远比 Dabai DC1(UVC 直出,~50-100ms)高**。

| 用途 | 是否适合 GoPro |
|---|---|
| 数据采集 + 时间戳后对齐 | ✅ 可接受(后处理减去固定偏移即可) |
| 录制 POV 视频参考 | ✅ 可接受 |
| 实时遥操作(盯着画面操作) | ⚠️ 200ms 滞后明显 |
| 视觉伺服 / VIO | ❌ 太慢 |

时间戳对齐技巧:GoPro 的 `header.stamp` 来自 `usb_cam` 读 `/dev/video42` 那一刻,**漏掉了相机内部那段固定延迟**。测量一次偏移(挥手对比 vs Dabai)后,后处理时把 GoPro stamp 减去这个偏移即可与其他相机对齐。

---

## 启动流程(3 个终端)

### 终端 1 — 把 GoPro 切到 webcam 模式

```bash
cd /home/agilex/cobot_magic/aloha-devel/Piper-AVP-Teleop/gopro_as_webcam_on_linux

# 启动前: GoPro 用 USB-C 数据线接到电脑,开机
# (注意是数据线不是充电线,GoPro 原装线常常只能充电)

sudo gopro webcam -r 720 -f superview
# -r 720      分辨率(480 / 720 / 1080,1080p 容易卡顿,720p 实测最稳)
# -f superview 视角(linear / wide / superview / narrow)
```

脚本会:
1. 提示 "Please plug in your GoPro and switch it on" → 看 GoPro 屏幕出现充电图标后回车
2. 检测 USB-Ethernet 接口(类似 `enxXXXXXX`)
3. 发 HTTP 命令切相机模式
4. 输出 "Successfully started the GoPro Webcam mode"

### 终端 2 — 起 ffmpeg 把 UDP 流转成 /dev/video42

**等终端 1 跑完上面那条**,然后:

```bash
ffmpeg -nostdin -threads 0 -loglevel warning -stats \
    -fflags nobuffer+discardcorrupt -flags low_delay \
    -i 'udp://@0.0.0.0:8554?overrun_nonfatal=1&fifo_size=1000000' \
    -f:v mpegts -fflags nobuffer -vf format=yuv420p \
    -f v4l2 /dev/video42
```

关键参数(低延迟优化):
- `-threads 0` 多核解码(脚本默认给的 `-threads 1` 单线程会卡)
- `-fflags nobuffer+discardcorrupt` 不缓冲、坏帧丢弃
- `-flags low_delay` ffmpeg 内部低延迟模式
- `fifo_size=1000000` UDP 缓冲 1MB(原 50MB 太大、加延迟)
- `-stats` 强制显示 `frame= ...` 进度行(配合 `-loglevel warning` 看清干净)

**前 1-2 秒会刷一堆 `SPS unavailable / no frame` 错误,是正常的**(等下一个关键帧),撑过去就开始 `frame=N fps=30 ...` 滚动。

### 终端 3 — 验证画面

```bash
ffplay -fflags nobuffer -flags low_delay -framedrop \
       -probesize 500K -analyzeduration 500000 \
       /dev/video42
```

弹窗显示 GoPro 实时画面就成功了。`-framedrop` 关键 —— 解码不上就丢帧,不让画面堆积导致延迟累积。

---

## 关键路径速查

| 内容 | 值 |
|---|---|
| 第三方脚本路径 | `gopro_as_webcam_on_linux/` |
| GoPro USB-Ethernet 接口 | `enxXXXXXXXXXXXX`(每次重启变,脚本自动找) |
| GoPro 控制 IP(脚本里) | `172.22.155.51`(网段固定,后缀可能不同) |
| UDP 流端口 | `8554` |
| v4l2loopback 设备号 | `/dev/video42` |
| GoPro 物理位置(USB) | Bus 01 Port 13 子口(同分线器,走 USB 2.0 CDC-NCM 协议) |

---

## 已知约束

1. **必须用 USB-C 数据线**。GoPro 出厂可能附充电线,Linux 不认。
2. **GoPro 在 webcam 模式下走 USB 2.0 CDC-NCM**,不是 USB 3.0(Hero 12 固件设计如此)。但 1080p H.264 ~30Mbps,USB 2.0 的 480Mbps 完全够。
3. **重启脚本**需要 sudo,因为脚本会改虚拟网卡配置 + 加载 `v4l2loopback` 内核模块。
4. **建议分辨率 720p**,1080p 在我们这台机器上即使开多线程也偶尔卡。720p 实测足够清晰。
5. **GoPro 进 webcam 模式后不能用屏幕操作**(切 FOV、分辨率都不行)。要改设置就 Ctrl+C 重启脚本带新参数。
6. **关相机前先 Ctrl+C** 两个终端,让 ffmpeg 优雅退出,否则下次启动可能 `/dev/video42` 被占。

---

## TODO: 接入 ROS

下一步把 `/dev/video42` 桥接到 ROS topic(类似 Dabai 相机):

```bash
# 草稿,待验证
sudo apt install ros-noetic-usb-cam

rosrun usb_cam usb_cam_node \
    _video_device:=/dev/video42 \
    _image_width:=1280 _image_height:=720 \
    _pixel_format:=yuyv \
    _io_method:=mmap \
    _framerate:=30 \
    _camera_name:=gopro
```

待办:
- [ ] 验证 `usb_cam` 节点能稳定订阅 `/dev/video42` 并发布 `/usb_cam/image_raw`
- [ ] 测量 ROS 层延迟(`rostopic delay`),记录 GoPro 相对 Dabai 的固定偏移量
- [ ] 把 GoPro topic 加入数据采集脚本(`collect_data_3arm.sh` 之类)
- [ ] 确定 GoPro 物理安装位置 / 用途(POV 视角?额外角度?)
- [ ] 决定是否需要相机内参标定(超广角 superview 模式需要)

---

## 故障排查

| 现象 | 可能原因 |
|---|---|
| 脚本说"Plug in GoPro"按回车没反应 | USB 线是充电线 / GoPro 没开机 / 屏幕没充电图标 |
| 脚本找不到 `enxXXXXX` 网络接口 | 同上 / 内核模块未加载(`sudo modprobe cdc_ncm`) |
| ffmpeg 一直刷 `SPS unavailable / no frame` 超过 5 秒 | UDP 流没真正过来 — 检查 GoPro 屏幕是不是 USB 模式图标 |
| ffmpeg 解码慢 / 画面卡顿 | 没加 `-threads 0`,或者分辨率太高(降到 720p) |
| `frame=0` 后无进度 | 用了 `-loglevel warning` 但没加 `-stats`,实际可能已在跑 |
| 画面延迟累积越来越大(看起来"漂移") | 没加 `-framedrop` 给 ffplay 或下游 |
| Ctrl+C ffmpeg 后再启动 v4l2 报忙 | 上次 ffmpeg 没完全释放,等 5 秒再启动,或 `sudo modprobe -r v4l2loopback && sudo modprobe v4l2loopback exclusive_caps=1 video_nr=42` |
