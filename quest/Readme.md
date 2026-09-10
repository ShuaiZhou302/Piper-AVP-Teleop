# Quest 2 Dual-Arm Teleop (SteamVR -> SSH -> Piper)

Cross-machine sibling of [`../avp/`](../avp/Readme.md): same idea (headset
pose -> Pinocchio IK -> `/master/joint_*`), different transport, because the
robot (`cobot_magic`) is only reachable over SSH/VPN, not on the same LAN as
the Windows box running the headset.

```
[Windows PC]                                      [cobot_magic, over SSH/VPN]
 SteamVR (Link/Air Link)                            quest_server.py
   |  openvr_reader.py                                |  ArmChannel x2 (left/right)
   v                                                   |  PinocchioIKSolver (reused
 quest_client.py  ---TCP, raw OpenVR poses--->  SSH -L |   from ../teleop/)
   (dumb sensor relay,                          tunnel |  -> /master/joint_left
    no frame math,                                     |  -> /master/joint_right
    no safety logic)                                   v
                                                  piper driver -> real arms
```

Left controller drives the **left** arm's end-effector, right controller
drives the **right** arm's. Head pose is captured and sent but nothing
consumes it yet (candidate future use: camera gimbal).

**All coordinate-frame math and ALL safety policy (clutch, staleness
watchdog, e-stop ramp-home) live on the robot side** (`quest_server.py`).
The Windows client never decides anything about the robot — if it crashes,
hangs, or the tunnel drops, `quest_server.py` notices on its own and freezes
/ ramps the arms home. Treat this as load-bearing: don't move safety
decisions to the client when adding features.

---

## 1. Windows side setup

```powershell
winget install --id Python.Python.3.12 -e
python -m pip install openvr numpy
```

(If `python --version` still resolves to the Microsoft Store stub after
installing, disable the `python.exe` / `python3.exe` app execution aliases
under Settings -> Apps -> Advanced app settings -> App execution aliases.)

Install SteamVR (Steam -> `steam://install/250820`), then:

1. Open the Meta Horizon app, connect the headset via Link or Air Link.
2. Enter the headset, launch Link -> SteamVR should auto-start; confirm the
   HMD and both controllers show as tracked in the SteamVR status window.
3. Sanity check the raw poses AND buttons with no networking involved:
   ```powershell
   python openvr_reader.py
   ```
   Hold each controller and squeeze grip/trigger -- you should see
   `trig=`/`grip=` values update live next to that controller's pose. If
   they stay at 0.00 no matter what you press, see "Button input gotchas"
   below before assuming the hardware is broken.

### Button input gotchas (read this if trig/grip read 0.00)

Two real bugs were hit and fixed getting controller buttons working on this
setup; both are baked into `openvr_reader.py` now, but if you ever see
button values stuck at zero while poses track fine, these are the first two
things to suspect:

1. **`IVRSystem::GetControllerState` (the "legacy" input API) does not work
   for a background utility app on this SteamVR version.** It returns
   `success=False` for Quest 2 Touch controllers bridged through SteamVR's
   `oculus` driver unless the calling app currently has scene focus, which a
   plain pose-reader script never does. This is also Valve's own documented
   deprecated path. Fix: read buttons via the **Action system** (`IVRInput`)
   instead -- see `openvr_actions/actions.json` +
   `openvr_actions/binding_oculus_touch.json`. Input paths there (`/input/
   trigger`, `/input/grip`, `/input/a`, `/input/b`, `/input/x`, `/input/y`)
   were verified against the installed driver's own profile:
   `SteamVR/drivers/oculus/resources/input/touch_profile.json`.

2. **Every OpenVR script run via the same `python.exe` shares one
   auto-generated app key** (`system.generated.python.exe`, derived from the
   executable name, not the script path). Confirmed by reading
   `Steam/logs/vrserver.txt`: the FIRST script to touch the legacy
   `GetControllerState` API under that shared key gets it permanently pinned
   to legacy-only mode, silently starving the Action system for every
   *other* script that later runs under that same key -- including a
   correctly-written one. Fix: give this script its own stable identity
   before touching input, via `openvr_actions/app.vrmanifest` +
   `IVRApplications.identifyApplication()` (done automatically in
   `QuestPoseReader.__init__`). `app.vrmanifest`'s `binary_path_windows` is
   regenerated on every run from `sys.executable` -- a stale/relative path
   there makes SteamVR silently skip the whole manifest entry (you'll see
   `... binary_path ... is invalid. Skipping` in `vrserver.txt`), and
   `identifyApplication()` then fails with `UnknownApplication`.

If poses AND buttons both read as dead/zero, check `Steam\logs\vrserver.txt`
(tail it) for `[Input]` lines around the time you launched the script --
it is very explicit about which binding file loaded for which app key.

## 2. SSH tunnel

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

## 3. Robot side setup (cobot_magic, in `aloha` conda env)

Same startup sequence as the AVP flow ([`../Readme.md`](../Readme.md)) up
through camera launch, then instead of the AVP/keyboard scripts:

```bash
# 1. CAN init
cd /home/agilex/cobot_magic/Piper_ros_private-ros-noetic/
bash can_config.sh
source devel/setup.bash

# 2. Power-cycle the arms, then launch the driver for BOTH arms you're
#    teleoperating (left + right -- adjust to your actual launch setup).
conda activate aloha
roslaunch piper start_ms_piper.launch mode:=1 auto_enable:=true

# 3. Cameras (optional for phase 1, pose-only teleop doesn't need them)
roslaunch astra_camera multi_camera.launch

# 4. Quest teleop server
cd /home/agilex/cobot_magic/aloha-devel/Piper-AVP-Teleop/quest
python quest_server.py
```

It binds `127.0.0.1:8770` only — reachable exclusively through the SSH
tunnel above, never from the open network. It waits up to 10s for
`/puppet/joint_left` and `/puppet/joint_right` feedback, then ramps both
arms to `INITIAL_ARM_JOINTS` (same anchor pose as the AVP script) before
accepting teleop input.

## 4. Run the client

```powershell
python quest_client.py
```

Prints a status line every second (`hz=... head=OK left=OK right=OK`). Once
`quest_server.py` logs `[teleop] ready.`, the system is live.

## 5. Controls

| Input | Effect |
| --- | --- |
| Hold **grip** (either controller) | Engage that arm — locks the current hand pose as the delta-tracking origin, arm follows hand motion from there |
| Release **grip** | Freeze that arm at its last commanded pose (no drift, no auto-return) |
| **Trigger** (analog, while engaged) | Gripper opening — released = open, fully squeezed = closed |
| **A/X** (either controller) | Panic: both arms ramp home at `--return_speed_rad_s`, both disengage |

Re-engaging after a release does **not** re-anchor to a stale pose — grip
press always relocks to the hand's pose *at that instant*, so small drift
between engagements is expected and matches the AVP script's per-episode
anchoring behavior.

## 6. Safety watchdog details

| Condition (measured on the robot) | Effect |
| --- | --- |
| No packet for > `--stale_freeze_sec` (default 0.2s) | Both arms forced to frozen/disengaged, regardless of last known button state |
| No packet for > `--stale_estop_sec` (default 1.0s) | Both arms ramp home at `--return_speed_rad_s`, requires fresh grip press after reconnect to re-engage |
| A/X pressed on either controller | Same ramp-home as above, immediate, regardless of staleness |

Tune `--stale_freeze_sec` / `--stale_estop_sec` to the tunnel's actual RTT —
run `ping` / `ssh cobot_magic "echo ok"` a few times and leave headroom
(freeze threshold should be a few multiples of typical jitter, not just
mean RTT).

## 7. Coordinate frame

OpenVR's `TrackingUniverseStanding` world frame is right-handed, **+Y up,
+Z back** (-Z is the direction you faced when Guardian was set up) — the
same convention `avp/Readme.md` section 9 documents for AVP/WebXR. That's
why `R_QUEST_TO_PIPER` in `quest_server.py` is numerically identical to
`R_AVP_TO_PIPER` in `eef_avp_control_singlearm.py`. If this is ever
re-verified empirically and turns out subtly different for SteamVR
controllers specifically (vs. the HMD), update the matrix here and note the
discrepancy — don't assume it silently, the delta math is only correct if
this remap is right.

## 8. Known gaps / follow-ups

- **No visual feedback loop yet.** The AVP flow pushes the robot's camera
  feed back into the headset via Vuer's `ImageBackground`; this OpenVR path
  has no equivalent (SteamVR doesn't have a lightweight "put this image in
  front of the user" primitive the way a WebXR canvas does). Today the
  operator tele-operates blind, or by watching a monitor separately. Options
  if/when this matters: a small SteamVR overlay app, or piping camera frames
  to a 2D window mirrored into the headset's Desktop view.
- **Head pose is unused.** Wired through the protocol and available in
  `quest_server.py` (`frame["head"]`) but nothing consumes it.
- **Single TCP client at a time.** `quest_server.py`'s accept loop only
  serves one connection; a second `quest_client.py` instance will hang until
  the first disconnects. Fine for one operator, revisit if that changes.
