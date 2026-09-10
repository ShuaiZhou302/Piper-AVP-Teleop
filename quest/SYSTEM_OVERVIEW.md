# Quest 2 Tri-Arm + Mobile Base Teleoperation System

Internal technical reference for the Quest 2 -> cobot_magic teleoperation
module (`quest/` in `Piper-AVP-Teleop`), written to be pulled from directly
when drafting the paper's system/methods section. Reflects the
implementation as of commit `f51227e` on branch `quest2-teleop`.

**Verification status disclosure (read before citing numbers below as
"demonstrated"):** pose/button capture, the coordinate remap, the SSH
transport, and the tri-arm/base control logic have been verified on real
Quest 2 hardware and (for `quest_server.py`/`camera_streamer.py`) via
import/dependency checks in the actual robot-side ROS environment. The
control loop has **not yet been exercised against the powered, physical
robot arms** -- that run is scheduled but has not happened as of this
writing. Do not describe the tri-arm/base motion as hardware-validated
until that session completes; the pose-capture and software-architecture
claims are independently solid.

---

## 1. Motivation and relation to prior work

The lab's existing teleoperation stack for this platform (`avp/`,
`teleop/`) uses an Apple Vision Pro: hand-tracking (no physical
controllers) drives one arm's end-effector via head motion, with pinch
gestures as the engage/disengage gesture, over a Vuer/WebXR bridge running
directly on the robot host (AVP and the robot are on the same LAN).

This module extends that design in two ways that motivated most of the
architectural differences below:

1. **Physical controllers instead of hand-tracking.** Quest 2's Touch
   controllers give real analog triggers, grip squeeze, and face buttons
   via the standard WebXR Gamepad API, in place of AVP's pinch-distance
   heuristic. This allows independent per-hand end-effector control (not
   just a head-driven single arm) and a continuous (not binary) gripper
   command.
2. **Cross-machine deployment.** The operator's Windows PC and the robot
   host (`cobot_magic`) are not on the same network; the robot is reachable
   only over SSH through an internal VPN. The AVP flow's assumption of a
   direct LAN connection between headset and robot does not hold here, so
   the pose-capture, transport, and control stages are split across three
   processes on two machines (see Section 3).

## 2. Hardware setup

- **Headset:** Meta Quest 2, standalone, connected to the operator's
  Windows PC only via shared WiFi (no USB/Link cable, no SteamVR -- an
  earlier OpenVR/SteamVR-based implementation was built and verified
  working, then superseded by the WebXR approach in this document; see
  Appendix in `Readme.md` if the OpenVR path is relevant to describe as a
  design alternative that was tried).
- **Robot:** `cobot_magic`, an AgileX mobile manipulation platform with:
  - Three AgileX Piper 6-DoF arms + parallel grippers (referred to as
    `left`, `mid`, `right` throughout), each on its own CAN bus.
  - A differential-drive AGV base (Trossen "Slate"-compatible, driven by
    `interbotix_slate_driver`), on a fourth, separate CAN bus.
  - Three RGB cameras, one wrist/front-facing per arm position.
- **Compute:** the operator's Windows PC (pose capture + network bridge)
  and `cobot_magic`'s onboard PC (ROS Noetic, Pinocchio-based IK, all
  robot I/O), connected via SSH through an internal VPN (no direct LAN
  path between them).

## 3. Software architecture

Three processes across two machines, connected by two independent,
unidirectional TCP streams tunneled over one SSH connection:

```
Quest 2 (Meta Quest Browser, WebXR session)
    |  pose (head, left, right) + button state, 50 Hz
    v
webxr_server.py (Windows)  <-- HTTPS+WSS server + bidirectional bridge
    |  pose: browser -> quest_server.py           (TCP, port 8770)
    |  camera: camera_streamer.py -> browser        (TCP, port 8771)
    v (both ports tunneled over one `ssh -L` connection)
quest_server.py (cobot_magic)          camera_streamer.py (cobot_magic)
  - Pinocchio IK x3 (left/mid/right)     - 3x ROS Image subscriber
  - tri-arm clutch + safety logic        - JPEG encode, 5 Hz, standalone
  - /master/joint_{mid,left,right}         process (kept off the 50 Hz
  - /cmd_vel (base)                        control loop on purpose)
```

Design rationale for the two-port split: the pose stream is latency-
critical and small (~1 KB/frame); the camera stream is bandwidth-heavy and
latency-tolerant (JPEG frames, base64-encoded). Sharing one TCP connection
or one process between them would let a large image frame sit in the send
queue in front of a time-critical pose/control frame. Both directions use
the same length-prefixed JSON wire framing (`protocol.py`).

`webxr_server.py` and the browser page (`index.html`) are intentionally
"dumb": no coordinate transform, no safety decision is made outside
`quest_server.py`. If the network drops or the Windows process crashes,
`quest_server.py`'s own staleness watchdog (Section 6) is what keeps the
robot safe, not anything upstream.

### Why WebXR instead of reusing the existing Vuer/WebXR bridge (`avp/`)

The existing `avp/tele_vision.py` wraps `vuer` (pinned to `0.0.31rc7` for
schema-compatibility reasons already documented in `avp/Readme.md`).
`vuer`'s Python API declares a `Gamepads` scene component, but the bundled
client-side JS in that pinned version does not implement it (confirmed by
inspecting the installed package's compiled frontend -- every "gamepad"
reference resolves to Three.js's generic controller-model rendering code,
not an input event emitter). Since AVP's hand-tracking has no equivalent
notion of "physical button," this gap was never exercised before. Rather
than depend on an unfinished, version-pinned component, `index.html` reads
controller input directly via the standard `XRInputSource.gamepad` API
(`buttons[]`/`axes[]`), which every WebXR-capable browser implements per
spec regardless of any particular JS framework.

### Why a hand-rolled WebGL page instead of a framework

`index.html` renders through raw WebGL calls (custom shaders, manual
matrix math) rather than a scene-graph library. This was a scope decision,
not a technical constraint: the page needs to (a) read controller/head
poses, (b) draw a handful of boxes and textured quads for the operator HUD
and camera panels, and (c) relay data over WebSocket. Given the second-by-
second engineering time available before the target demo, avoiding a
framework dependency (and the associated CSP/CDN-loading constraints of
some deployment contexts) was judged lower-risk than it added value.

## 4. Coordinate frames

WebXR's reference space (`local-floor`, falling back to `local`) is
right-handed with **+Y up**; the operator's forward direction at session
start is **-Z**. This is numerically identical to the world-frame
convention already documented for the AVP/Vuer flow (`avp/Readme.md`
section 9), which is what makes the axis remap below a direct reuse rather
than a new derivation.

The robot's IK layer (shared with the AVP flow, `teleop/
eef_keyboard_control_singlearm.py`) uses **+X forward, +Y left, +Z up** in
each arm's base frame. The fixed rotation

```
R_QUEST_TO_PIPER = [[ 0,  0, -1],
                     [-1,  0,  0],
                     [ 0,  1,  0]]
```

maps Quest-world vectors into this frame (`piper_x = -quest_z`,
`piper_y = -quest_x`, `piper_z = quest_y`); the same matrix, `R_AVP_TO_
PIPER`, already exists in `eef_avp_control_singlearm.py` for the AVP flow.
Poses are transmitted over the wire in the raw Quest frame; the remap is
applied server-side (`quest_server.py`'s `remap_to_piper()`), keeping the
transport and safety-relevant coordinate math in the same trusted process.

## 5. Control mapping

For each of the three arms, the delta-pose-composition scheme is the same
one `eef_avp_control_singlearm.py` uses for its single head-driven arm,
applied independently to three (source, arm) pairs:

| Source | Arm | Gripper |
| --- | --- | --- |
| Head pose | `mid` | fixed open (no trigger available for the head) |
| Left controller pose | `left` | left trigger, analog |
| Right controller pose | `right` | right trigger, analog |

**Clutch (engage/disengage).** Pressing both controllers' grip buttons
together is a **toggle**, not a hold-to-track gesture: one press engages
all three arms simultaneously (latching the current head/left/right poses
as the zero-delta reference for each), a second press disengages all
three, holding their last commanded joint configuration. This differs from
the AVP flow's pinch-based state machine (which distinguishes a quick
pinch from a held pinch with separate timing thresholds) mainly in using
discrete controller buttons instead of a continuous hand-tracking signal,
which removes the need for hysteresis/debounce logic AVP requires to
reject false pinch triggers.

**Gripper.** Trigger released (0) maps to fully open (0.1 m); fully
squeezed (1) maps to fully closed (0.0 m), linear in between -- "squeeze to
grab."

**Mobile base.** Independent of the arm clutch by design (grip is a
middle/ring-finger action, face buttons are thumb-actuated, so both can be
held simultaneously): left controller X/Y button turns the base left/right
(`angular.z`, default 0.3 rad/s), right controller A/B drives it
backward/forward (`linear.x`, default 0.15 m/s), published as
`geometry_msgs/Twist` on `/cmd_vel`.

**Panic / emergency stop.** Thumbstick click on either controller
immediately disengages all three arms (ramping them to a fixed home
configuration at a capped joint speed) and zeros the base command,
overriding the toggle state.

## 6. Safety architecture

All safety-relevant decisions are made in `quest_server.py`, the only
process with a trust relationship to the robot's actuators. Three
independent layers, from application logic down to firmware:

1. **Staleness watchdog (application layer, `quest_server.py`).** Two
   thresholds measured against wall-clock time since the last received
   network packet: past 0.2 s, all arms are frozen (last commanded pose
   held, clutch state cannot be trusted) and the base is zeroed; past
   1.0 s, all arms additionally ramp to a fixed home joint configuration
   at a capped speed (0.3 rad/s, ~17 deg/s). Re-engaging after either
   requires a fresh clutch toggle -- staleness cannot silently resume
   motion.
2. **Per-tick joint-step clamp (application layer).** Each IK solution is
   clamped to at most 0.05 rad of change per 50 Hz control tick (~2.5
   rad/s ceiling) relative to the previously commanded joint state, before
   being published, independent of how large a jump the operator's motion
   or an IK solver artifact might otherwise produce.
3. **Gripper torque limit (firmware layer, not implemented by this
   module).** Every gripper command published to `/master/joint_<arm>`
   passes through the existing `piper_start_ms_node.py`, which calls
   `piper_sdk`'s `GripperCtrl(angle, effort, ...)` with a hardcoded
   `effort` of 1000 (the SDK's units are 0.001 N*m, so 1.0 N*m) -- a
   torque limit enforced by the gripper's own motor controller over CAN,
   not a software polling loop. The motor does not continue tightening
   past this torque even while still commanded further closed, which is
   the same mechanism (and the same limit value) already relied on by the
   AVP teleop path, since both publish through the identical downstream
   node.
4. **Base driver watchdog (firmware/driver layer, not implemented by this
   module).** `interbotix_slate_driver`'s `slate_base_node` has its own
   independent 300 ms `/cmd_vel` timeout (zeros velocity if no message
   arrives), beneath and in addition to this module's own staleness
   handling.

## 7. Visual feedback

Three camera feeds (one per arm position) are streamed into the headset as
textured panels rendered above the operator HUD, via a path independent of
the control stream (Section 3). Frames are downsampled (320x240 default),
JPEG-compressed (quality 55 default) and sent at a low rate (5 Hz default)
-- explicitly scoped for situational awareness during teleoperation, not
as a vision pipeline. As of this writing this path is implemented and
verified through the network/bridge layer with a synthetic image source,
but not yet confirmed against real camera frames on-headset (panel
placement and JPEG orientation are first-pass values pending that check).

## 8. Parameters (defaults, `quest_server.py`)

| Parameter | Default | Meaning |
| --- | --- | --- |
| Control loop rate | 50 Hz | |
| Boot ramp duration | 3.0 s | Speed-limited ramp from current pose to the fixed home configuration at process start |
| Return-to-home speed | 0.3 rad/s | Used for both the panic ramp-home and (with a longer implicit duration) the boot ramp |
| Max per-tick joint step | 0.05 rad | ~2.5 rad/s ceiling at 50 Hz |
| Staleness: freeze threshold | 0.2 s | |
| Staleness: e-stop/ramp-home threshold | 1.0 s | |
| Gripper range | 0.0 - 0.1 m | |
| Gripper torque limit | 1.0 N*m | Firmware-level, CAN ID `0x159`, not adjustable from this module |
| Base linear speed | 0.15 m/s | Driver's own software ceiling is 1.0 m/s |
| Base angular speed | 0.3 rad/s | Driver's own software ceiling is 1.0 rad/s |
| Base driver watchdog | 300 ms | Fixed in `interbotix_slate_driver`, not configurable from this module |
| Camera stream rate | 5 Hz | Configurable; deliberately low, see Section 7 |
| Camera resolution (sent) | 320x240 | Downsampled from native camera resolution |

## 9. Known limitations (as of this writing)

- Tri-arm/base control has not yet been run against the powered physical
  robot (see the disclosure at the top of this document).
- Camera panel placement and JPEG texture orientation in the headset are
  unconfirmed with real camera frames.
- Single operator only: `quest_server.py` serves one TCP client at a time,
  `webxr_server.py` accepts multiple browser connections but only one
  upstream robot connection.
- No head pose is consumed for anything but driving the `mid` arm -- e.g.
  no camera-gimbal use, unlike some possible extensions.
