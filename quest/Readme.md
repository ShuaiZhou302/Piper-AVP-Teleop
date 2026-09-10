# Quest 2 Tri-Arm + Base Teleop (WebXR -> SSH -> Piper + Slate base)

Cross-machine sibling of [`../avp/`](../avp/Readme.md): same idea (headset
pose -> Pinocchio IK -> `/master/joint_*`), different transport, because the
robot (`cobot_magic`) is only reachable over SSH/VPN, not on the same LAN as
the Windows box driving the headset.

```
[Quest 2 headset]                    [Windows PC]                        [cobot_magic, over SSH/VPN]
 Meta Quest Browser        HTTPS/WSS   webxr_server.py                     quest_server.py
   (WebXR session,        same LAN     (serves index.html,                  ArmChannel x3 (mid/left/right)
    XRInputSource.gamepad) ────────►    relays JSON -> wire        SSH -L   PinocchioIKSolver (../teleop/)
                                        protocol frames)  ───────► tunnel   -> /master/joint_mid
                                                                             -> /master/joint_left
                                                                             -> /master/joint_right
                                                                             -> /cmd_vel (Twist, base)
                                                                                      │
                                                                                      ▼
                                                                          piper driver + slate_base_node
```

Head drives the **mid** arm, left controller drives the **left** arm, right
controller drives the **right** arm — same delta-pose-composition math as
`eef_avp_control_singlearm.py`'s head→arm mapping, applied three times. Left
X/Y turn the mobile base left/right and right A/B drive it backward/forward
(`interbotix_slate_driver`'s `/cmd_vel`). See
[Controls](#6-controls) for the full mapping.

**All coordinate-frame math and ALL safety policy (clutch, staleness
watchdog, e-stop ramp-home, base stop) live on the robot side**
(`quest_server.py`). Neither the Quest's browser nor the Windows bridge ever
decides anything about the robot — if either crashes, hangs, or the tunnel
drops, `quest_server.py` notices on its own and freezes / ramps the arms
home / stops the base. Treat this as load-bearing: don't move safety
decisions upstream when adding features.

### Why WebXR instead of SteamVR/OpenVR

An earlier version of this module used SteamVR + `pyopenvr` (still present,
see [Appendix: OpenVR path](#appendix-openvr-path) — it works and is kept as
a fallback). Switched to WebXR because:

- The Quest can join directly over LAN WiFi, same as the AVP flow — no
  Link/Air Link cable, no SteamVR install, one fewer moving part.
- Standard `XRInputSource.gamepad.buttons`/`.axes` gives trigger/grip/face
  buttons directly. The OpenVR path needed real reverse-engineering to get
  there (see the Appendix) because SteamVR's legacy controller-state API
  doesn't work for a background app on this driver version, and the pinned
  `vuer==0.0.31rc7`'s `Gamepads` scene component turned out to be an
  unimplemented stub in the bundled frontend (confirmed by grepping the
  installed package — not worth fighting a half-finished dependency).

---

## 1. One-time cert setup (Windows)

Quest's browser needs to trust an HTTPS cert for your PC's LAN IP, same
mkcert pattern as the AVP setup:

```powershell
winget install --id FiloSottile.mkcert -e
mkcert -install
cd quest\webxr
mkcert -cert-file cert.pem -key-file key.pem <this PC's LAN IP> localhost 127.0.0.1
```

Find your LAN IP with `ipconfig` (the `WLAN` adapter's IPv4 address — both
the PC and the Quest must be on that same WiFi network). Re-run the
`mkcert -cert-file ...` line if the PC's IP ever changes.

Then trust the mkcert root CA **on the Quest** (one-off, mirrors
`avp/Readme.md` section 3):

1. On the PC: `cd "$(mkcert -CAROOT)"; python -m http.server 8000`
2. On the Quest, in the Meta Quest Browser: open
   `http://<PC LAN IP>:8000/rootCA.pem` → it downloads a config profile.
3. Quest `Settings → System → Device → Install Certificate` (wording varies
   by firmware) → install the downloaded profile.

## 2. Windows side setup

```powershell
winget install --id Python.Python.3.12 -e
python -m pip install websockets numpy
```

(If `python --version` still resolves to the Microsoft Store stub after
installing, disable the `python.exe` / `python3.exe` app execution aliases
under Settings -> Apps -> Advanced app settings -> App execution aliases.)

## 3. SSH tunnel

`cobot_magic` is already configured in `~/.ssh/config` with the VPN
`ProxyCommand`; this just adds a local port forward on top of it. Keep this
running in its own terminal for the whole session:

```powershell
powershell -File scripts\start_tunnel.ps1
```

It retries automatically if the VPN blips. Verify the plain SSH path works
first if you've never used `cobot_magic` from this machine:

```powershell
ssh cobot_magic "echo ok"
```

## 4. Robot side setup (cobot_magic, in `aloha` conda env)

Uses the existing 3-arm launch tooling in
[`../multi_arm_launch_tools/`](../multi_arm_launch_tools/Readme.md) --
`start_ms_piper_3arm.launch mode:=1 auto_enable:=true` already gives all
three arms `/master/joint_<arm>` command acceptance with no leader arm
needed, which is exactly what `quest_server.py` expects (its
`--{mid,left,right}_joint_topic` / `--..._cmd_topic` defaults match this
launch file's topics as-is -- no ROS-side changes needed):

```bash
# 1. CAN init -- 4 modules this time (left/right/mid arms + base), NOT the
#    2-arm can_config.sh from the AVP flow.
cd /home/agilex/cobot_magic/aloha-devel/Piper-AVP-Teleop/
bash multi_arm_launch_tools/can_config_shuai.sh
source /home/agilex/cobot_magic/Piper_ros_private-ros-noetic/devel/setup.bash

# 2. Power-cycle the arms (confirm power strip on + all 3 arms' aviation
#    connectors seated), then launch all 3 arms auto-enabled -- no physical
#    leader arm needed, Quest2 replaces that role entirely.
conda activate aloha
roslaunch /home/agilex/cobot_magic/aloha-devel/Piper-AVP-Teleop/multi_arm_launch_tools/launch/start_ms_piper_3arm.launch mode:=1 auto_enable:=true
# success = three lines of "使能状态: True", no SEND_MESSAGE_FAILED

# 3. Base driver -- NOT part of any existing launch file (confirmed by
#    grepping this repo's launch files: it's built but unused elsewhere).
#    Publishes odom/battery_state, subscribes /cmd_vel (geometry_msgs/Twist,
#    linear.x + angular.z only), has its own 300ms cmd_vel timeout baked in.
rosrun interbotix_slate_driver slate_base_node

# 4. Cameras (optional for pose-only teleop)
roslaunch /home/agilex/cobot_magic/aloha-devel/Piper-AVP-Teleop/multi_arm_launch_tools/launch/multi_camera_shuai.launch

# 5. Quest teleop server
cd /home/agilex/cobot_magic/aloha-devel/Piper-AVP-Teleop/quest
python quest_server.py
```

Gripper torque is already hardware-limited independent of anything in this
repo: every `/master/joint_<arm>` message's gripper value passes through
`piper_start_ms_node.py`'s `joint_callback()`, which calls
`piper_sdk`'s `GripperCtrl(angle, effort=1000, ...)` -- `effort` is a real
firmware-level torque limit sent over CAN (0.001 N*m units, so 1000 = 1.0
N*m), not something simulated in software. The motor will not keep
tightening past that torque even while still being commanded further
closed, which is the same mechanism (and the same hardcoded 1.0 N*m limit)
the AVP teleop path already relies on -- nothing extra was needed here to
get ALOHA-style "stops when it grasps something" gripper behavior.

It binds `127.0.0.1:8770` only — reachable exclusively through the SSH
tunnel above, never from the open network. It waits up to 10s for
`/puppet/joint_mid`, `/puppet/joint_left`, `/puppet/joint_right` feedback,
then ramps all three arms to `INITIAL_ARM_JOINTS` (same anchor pose as the
AVP script) before accepting teleop input.

## 5. Run the WebXR bridge + enter VR

On Windows:

```powershell
cd quest\webxr
python webxr_server.py
```

On the Quest, in the Meta Quest Browser:

```
https://<this PC's LAN IP>:8443
```

Tap **Enter VR**. The 2D page (visible before/after entering VR, and on the
PC if you load the same URL there) shows live `ws:`/`xr:`/`head:`/`left:`/
`right:` status lines — useful for confirming tracking before committing to
the headset. Once `quest_server.py` logs `[teleop] ready.`, the system is
live.

## 6. Controls

| Input | Effect |
| --- | --- |
| Press **both grips together** (once) | **Toggle ON** — engage all 3 arms at once: locks head + left + right poses as the delta-tracking origins, all three start following from there. This is a toggle, not hold-to-track — letting go of the grips afterward does nothing |
| Press **both grips together** again | **Toggle OFF** — freeze all 3 arms at their last commanded pose (no drift, no auto-return) |
| **Left/right trigger** (analog, while engaged) | That hand's arm gripper — released (resting) = open, fully squeezed = closed (squeeze to grab). Mid (head) arm's gripper stays fixed open (head has no trigger) |
| **Left X** | Base turn left (counter-clockwise) |
| **Left Y** | Base turn right (clockwise) |
| **Right A** | Base backward |
| **Right B** | Base forward |
| **Thumbstick click** (either hand) | Panic: all 3 arms ramp home at `--return_speed_rad_s`, base stops, everything force-disengaged (overrides the toggle) |

Base drive is independent of the arm clutch — grip is squeezed with the
middle/ring fingers, X/Y/A/B with the thumb, so driving the base while the
arms track is possible (and allowed) on purpose.

A staleness blip or panic always forces disengage regardless of the toggle's
current state — the toggle only holds while the link is fresh (see
[Safety watchdog details](#7-safety-watchdog-details)). Re-engaging does
**not** re-anchor to a stale pose — the grip-together rising edge always
relocks to the current head/hand poses *at that instant*, so small drift
between engagements is expected and matches the AVP script's per-episode
anchoring behavior.

## 7. Safety watchdog details

| Condition (measured on the robot) | Effect |
| --- | --- |
| No packet for > `--stale_freeze_sec` (default 0.2s) | All 3 arms forced to frozen/disengaged, base commanded to zero, regardless of last known button state |
| No packet for > `--stale_estop_sec` (default 1.0s) | All 3 arms ramp home at `--return_speed_rad_s`, base zeroed; requires a fresh both-grips press after reconnect to re-engage |
| Thumbstick click on either controller | Same ramp-home + base stop as above, immediate, regardless of staleness |
| Base driver's own `CMD_TIME_OUT` (300ms, in `slate_base.h`, not adjustable from here) | A second, independent layer: zeros base velocity if `quest_server.py` itself stops publishing `/cmd_vel` for any reason |

Tune `--stale_freeze_sec` / `--stale_estop_sec` to the tunnel's actual RTT —
run `ping` / `ssh cobot_magic "echo ok"` a few times and leave headroom
(freeze threshold should be a few multiples of typical jitter, not just
mean RTT).

## 8. Coordinate frame

WebXR's reference-space frame (`local-floor`/`local`) is right-handed,
**+Y up, +Z back** (-Z is the direction you faced when the session started)
— the same convention `avp/Readme.md` section 9 documents for AVP. That's
why `R_QUEST_TO_PIPER` in `quest_server.py` is numerically identical to
`R_AVP_TO_PIPER` in `eef_avp_control_singlearm.py`. `index.html` converts
WebXR's column-major `XRRigidTransform.matrix` to the row-major `rot` array
`protocol.py` expects — see the comment on `poseToWire()` there if this
math is ever touched.

## 9. Known gaps / follow-ups

- **No visual feedback loop yet.** The AVP flow pushes the robot's camera
  feed back into the headset via Vuer's `ImageBackground`; `index.html`
  renders nothing (a bare WebXR session with a cleared framebuffer, which is
  all the spec requires to keep `requestAnimationFrame` firing). Today the
  operator tele-operates blind, or by watching a monitor separately. If this
  needs solving, the natural next step is pushing camera frames into
  `index.html`'s WebGL context as a background quad — same idea as Vuer's
  `ImageBackground`, just hand-rolled.
- **Single WebSocket client at a time**, and separately, **single TCP
  bridge-to-robot client at a time**. `webxr_server.py` will happily accept
  a second browser tab, but `quest_server.py`'s accept loop only serves one
  TCP connection; a second bridge will hang until the first disconnects.
  Fine for one operator, revisit if that changes.
- **Base speed defaults are conservative and untested on hardware**
  (`--base_linear_speed 0.15`, `--base_angular_speed 0.3`) — the driver's
  own ceiling is 1.0/1.0. Tune up carefully after confirming the e-stop
  paths actually work.
- **`slate_base_node` isn't in any existing launch file** — it's started
  standalone in step 4 above. If this becomes permanent, fold it into
  `multi_arm_launch_tools`.

---

## Appendix: OpenVR path

The original SteamVR/`pyopenvr` implementation (`openvr_reader.py`,
`quest_client.py`, `openvr_actions/`) is still in this directory and still
works for pose+button capture, but is **not** wired to the tri-arm/base
`quest_server.py` above (it predates the mid-arm and base additions, and its
`quest_client.py` only ever sent head/left/right with no engage semantics
beyond what the OpenVR Action system's buttons carry — the wire protocol is
identical, so it would need equivalent client-side changes to work with the
current server, but none were made since WebXR replaced it as the primary
path). Kept for reference and as a fallback if WebXR ever becomes
unavailable (e.g. a firmware update breaks WebXR in the Quest browser).

If reviving it: `python openvr_reader.py` for a standalone pose+button
smoke test (needs SteamVR running, headset on Link/Air Link), then
`python quest_client.py` to stream to `quest_server.py` exactly like
`webxr_server.py` does. See git history on this file for the two real
OpenVR-specific bugs that had to be fixed (legacy `GetControllerState` not
working for background apps, and a shared `system.generated.python.exe` app
key silently poisoning the Action system across unrelated scripts) if
picking this path back up.
