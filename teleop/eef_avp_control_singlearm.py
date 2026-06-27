#!/usr/bin/env python3
"""
Step B: AVP head pose -> Piper single-arm EE teleop.

Single-process: Vuer subprocess + main thread (rospy + IK + publish + HUD).
The selected arm (default: mid) follows the operator's head; gripper fixed.

Run:
  conda activate aloha
  cd .../Piper-AVP-Teleop/teleop
  python eef_avp_control.py             # interactive arm prompt
  python eef_avp_control.py --arm m     # or m / mid / l / left / r / right
"""

# Initial arm joint configuration (joint0..joint5, radians).
# Captured from /puppet/joint_<arm> with the arm manually posed in teach mode.
# Boot ramp drives joint angles directly to these values; the corresponding
# EE pose is derived from FK at runtime and used as the teleop reset anchor.
# Same Piper hardware on all three arms -> same "ready" joint config works.
INITIAL_ARM_JOINTS = (0.0463, 0.5300, -0.5562, 0.000, 0.2419, 0.0000)
INITIAL_GRIPPER = 0.1

import argparse
import os
import sys
import time
from multiprocessing import shared_memory

import numpy as np

# IMPORTANT: import casadi (via eef_keyboard_control / pinocchio) BEFORE rospy.
# rospy loads the system libstdc++.so.6 (Ubuntu 20.04 = GCC 9, no GLIBCXX_3.4.29);
# conda's casadi binary needs GLIBCXX_3.4.29. If rospy loads first the wrong
# libstdc++ is already in-process and casadi import fails.
HERE = os.path.dirname(os.path.abspath(__file__))
AVP_DIR = os.path.normpath(os.path.join(HERE, "..", "avp"))
sys.path.insert(0, AVP_DIR)
sys.path.insert(0, HERE)

from eef_keyboard_control_singlearm import PinocchioIKSolver  # noqa: E402  (loads casadi)
import pinocchio as pin  # noqa: E402  (already in-process via the line above)
from avp_gesture_test import (  # noqa: E402
    GestureStateMachine, HandFreshness,
    State, THUMB, MIDDLE, STATE_COLOR, STATE_HINT,
)
from tele_vision import OpenTeleVision  # noqa: E402

# ROS imports come after casadi is in.
import rospy  # noqa: E402
from sensor_msgs.msg import JointState, Image as ImageMsg  # noqa: E402
from std_msgs.msg import Header, String  # noqa: E402
from tf.transformations import euler_matrix, euler_from_matrix  # noqa: E402
from PIL import Image, ImageDraw, ImageFont  # noqa: E402

# Topic the gesture state is broadcast on (subscribed by collect_data_3arm.py).
TELEOP_STATE_TOPIC = "/teleop/state"


# Arm name normalization, matches eef_keyboard_control_singlearm.py.
ARM_CHOICES = {
    "l": "left",  "left":  "left",
    "m": "mid",   "mid":   "mid",
    "r": "right", "right": "right",
}
# Per-arm camera short name. mid uses the existing front camera /camera_f
# (the head-mounted view, kept from the dual-arm setup).
ARM_CAM_SHORT = {"left": "l", "mid": "f", "right": "r"}


def resolve_arm(arm_arg):
    if arm_arg is None:
        while True:
            x = input("Select arm (l=left / m=mid / r=right): ").strip().lower()
            if x in ARM_CHOICES:
                return ARM_CHOICES[x]
            print("Please input l / m / r (or left / mid / right)")
    key = arm_arg.strip().lower()
    if key not in ARM_CHOICES:
        raise SystemExit(
            "Invalid --arm value: %s. Expected one of l/m/r/left/mid/right" % arm_arg
        )
    return ARM_CHOICES[key]


# AVP world (right / up / back) -> Piper world (forward / left / up).
R_AVP_TO_PIPER = np.array([
    [ 0,  0, -1],
    [-1,  0,  0],
    [ 0,  1,  0],
], dtype=float)

CERT = os.path.join(AVP_DIR, "cert.pem")
KEY = os.path.join(AVP_DIR, "key.pem")
SHM_NAME = "avp_eef_teleop_shm"
IMG_SHAPE = (480, 640, 3)


def _make_shm():
    size = int(np.prod(IMG_SHAPE))
    try:
        shm = shared_memory.SharedMemory(create=True, size=size, name=SHM_NAME)
    except FileExistsError:
        old = shared_memory.SharedMemory(name=SHM_NAME)
        old.close()
        old.unlink()
        shm = shared_memory.SharedMemory(create=True, size=size, name=SHM_NAME)
    img = np.ndarray(IMG_SHAPE, dtype=np.uint8, buffer=shm.buf)
    img[:] = 0
    return shm


class AvpEefController:
    def __init__(self, args):
        self.args = args
        self.arm = args.arm
        self.scale = args.scale

        # Vuer
        self.shm = _make_shm()
        self.img_view = np.ndarray(IMG_SHAPE, dtype=np.uint8, buffer=self.shm.buf)
        self.vr = OpenTeleVision(
            IMG_SHAPE[:2], SHM_NAME, stereo=False, cert_file=CERT, key_file=KEY
        )

        # ROS
        rospy.init_node("eef_avp_teleop", anonymous=True)
        self.joint = None
        self.latest_camera_frame = None  # numpy uint8 (H, W, 3) RGB
        rospy.Subscriber(args.joint_topic, JointState, self._joint_cb, queue_size=50)
        rospy.Subscriber(
            args.camera_topic, ImageMsg, self._camera_cb, queue_size=1, buff_size=2 ** 24
        )
        self.pub = rospy.Publisher(args.cmd_topic, JointState, queue_size=10)

        # Broadcast gesture state so collect_data_3arm.py can start/stop
        # recording on ENGAGED / IDLE transitions.
        self.state_pub = rospy.Publisher(TELEOP_STATE_TOPIC, String, queue_size=1)

        # IK
        self.ik = PinocchioIKSolver(args.urdf)

        # State machine
        self.fsm = GestureStateMachine()
        self._l_fresh = HandFreshness()
        self._r_fresh = HandFreshness()
        self.head_pose_at_lock = None  # (4, 4) head pose in AVP world at lock time
        self.frame_idx = 0
        self.last_print = 0.0

        # Last commanded joint config. Once boot ramp finishes we keep publishing
        # this every frame; we DO NOT echo /puppet feedback, otherwise feedback
        # lag drives the motor backwards (fight-back loop).
        self.target_q = None

        # Initial EE pose anchor for teleop delta tracking, derived from FK
        # on INITIAL_ARM_JOINTS during boot ramp.
        self.initial_arm_xyz = None
        self.initial_arm_R = None

        # HUD fonts
        try:
            self.font_big = ImageFont.truetype(
                "/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf", 60
            )
            self.font = ImageFont.truetype(
                "/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf", 26
            )
        except OSError:
            self.font_big = ImageFont.load_default()
            self.font = ImageFont.load_default()

    # ------------- ROS callbacks -------------
    def _joint_cb(self, msg):
        self.joint = msg

    def _camera_cb(self, msg):
        if msg.encoding == "rgb8":
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        elif msg.encoding == "bgr8":
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)[:, :, ::-1]
        else:
            return  # unsupported encoding
        if (msg.height, msg.width) != IMG_SHAPE[:2]:
            arr = np.array(
                Image.fromarray(arr).resize((IMG_SHAPE[1], IMG_SHAPE[0]), Image.BILINEAR)
            )
        # Atomic reference swap; main thread reads via Image.fromarray which copies.
        self.latest_camera_frame = arr

    def wait_feedback(self, timeout_sec):
        deadline = rospy.Time.now() + rospy.Duration(timeout_sec)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.joint is not None and len(self.joint.position) >= 6:
                return True
            if rospy.Time.now() > deadline:
                return False
            rate.sleep()
        return False

    def publish_joints(self, six, gripper):
        names = ["joint0", "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        m = JointState()
        m.header = Header(stamp=rospy.Time.now())
        m.name = names
        m.position = list(six) + [gripper]
        self.pub.publish(m)

    # ------------- FK helper (for verification / debug) -------------
    def _fk(self, q6):
        """Forward-kinematics on the IK reduced model. Returns (xyz, rpy) of EE."""
        q = np.asarray(q6, dtype=float).flatten()
        pin.framesForwardKinematics(self.ik.model, self.ik.data, q)
        se3 = self.ik.data.oMf[self.ik.ee_frame_id]
        xyz = np.asarray(se3.translation, dtype=float).flatten()
        rpy = np.array(euler_from_matrix(se3.rotation), dtype=float)
        return xyz, rpy

    # ------------- Boot ramp -------------
    def boot_ramp_to_initial(self, duration=3.0, hz=30.0):
        # Joint-space anchor: drive directly to INITIAL_ARM_JOINTS (no IK).
        # Avoids IK multi-solution drift when current pose is far from anchor.
        target_q = np.asarray(INITIAL_ARM_JOINTS, dtype=float)
        current_q = np.asarray(self.joint.position[:6], dtype=float)
        steps = max(1, int(duration * hz))
        rate = rospy.Rate(hz)
        print(f"[teleop] Boot ramp ({duration}s): {current_q.round(3)} -> {target_q.round(3)}")
        for i in range(steps):
            if rospy.is_shutdown():
                return
            alpha = (i + 1) / steps
            interp = (1.0 - alpha) * current_q + alpha * target_q
            self.publish_joints(list(interp), INITIAL_GRIPPER)
            self._draw_hud_text(["BOOTING", f"alpha = {alpha:.2f}"], colors=[(255, 200, 0), (200, 200, 200)])
            rate.sleep()
        self.target_q = target_q.tolist()
        print("[teleop] Boot ramp done.")

        # Compute the EE pose at INITIAL_ARM_JOINTS via FK; store as the
        # teleop-tracking anchor (used by compute_target_pose).
        fk_xyz, fk_rpy = self._fk(target_q)
        self.initial_arm_xyz = fk_xyz
        self.initial_arm_R = euler_matrix(*fk_rpy)[:3, :3]
        print(
            f"[teleop] FK at INITIAL_ARM_JOINTS: xyz={fk_xyz.round(4).tolist()}  "
            f"rpy(deg)={np.rad2deg(fk_rpy).round(2).tolist()}"
        )

    # ------------- Pose math -------------
    def compute_target_pose(self, head_now):
        """World-frame composition: target = R_remap(delta_avp) * initial_arm_pose."""
        head_lock = self.head_pose_at_lock
        delta_pos_avp = head_now[:3, 3] - head_lock[:3, 3]
        delta_R_avp = head_now[:3, :3] @ head_lock[:3, :3].T

        delta_pos_piper = R_AVP_TO_PIPER @ delta_pos_avp * self.scale
        delta_R_piper = R_AVP_TO_PIPER @ delta_R_avp @ R_AVP_TO_PIPER.T

        target_pos = self.initial_arm_xyz + delta_pos_piper
        target_R = delta_R_piper @ self.initial_arm_R
        target_rpy = np.array(euler_from_matrix(target_R), dtype=float)
        return target_pos, target_rpy

    # ------------- HUD -------------
    def _make_canvas(self):
        """Use the latest camera frame as background if available, else dark."""
        cam = self.latest_camera_frame
        if cam is not None:
            return Image.fromarray(cam.copy())
        return Image.new("RGB", (IMG_SHAPE[1], IMG_SHAPE[0]), (20, 20, 30))

    def _draw_hud_text(self, lines, colors=None):
        canvas = self._make_canvas()
        draw = ImageDraw.Draw(canvas)
        CW = IMG_SHAPE[1]
        if colors is None:
            colors = [(255, 255, 255)] * len(lines)
        y = 60
        for text, fill in zip(lines, colors):
            fnt = self.font_big if y == 60 else self.font
            w = draw.textlength(text, font=fnt)
            draw.text(
                ((CW - w) / 2, y), text, fill=fill, font=fnt,
                stroke_width=2, stroke_fill=(0, 0, 0),
            )
            y += 80 if fnt is self.font_big else 40
        self.img_view[:] = np.array(canvas)

    def update_hud(self, state, target_pos=None, target_rpy=None, ik_msg=""):
        canvas = self._make_canvas()
        draw = ImageDraw.Draw(canvas)
        CW = IMG_SHAPE[1]

        def centered(text, y, fnt, fill):
            w = draw.textlength(text, font=fnt)
            draw.text(
                ((CW - w) / 2, y), text, fill=fill, font=fnt,
                stroke_width=2, stroke_fill=(0, 0, 0),
            )

        centered(state.value, 10, self.font_big, STATE_COLOR[state])
        for k, line in enumerate(STATE_HINT[state]):
            centered(line, 80 + k * 32, self.font, (220, 220, 220))

        if target_pos is not None and target_rpy is not None:
            centered(
                f"x={target_pos[0]:+.3f} y={target_pos[1]:+.3f} z={target_pos[2]:+.3f}",
                380, self.font, (255, 255, 255),
            )
            rpy_d = np.rad2deg(target_rpy)
            centered(
                f"r={rpy_d[0]:+5.1f} p={rpy_d[1]:+5.1f} y={rpy_d[2]:+5.1f}",
                415, self.font, (160, 220, 255),
            )

        if ik_msg:
            centered(ik_msg, 200, self.font, (255, 90, 90))
        text = f"#{self.frame_idx}"
        w = draw.textlength(text, font=self.font)
        draw.text(
            (IMG_SHAPE[1] - w - 8, IMG_SHAPE[0] - 32), text,
            fill=(180, 180, 180), font=self.font,
            stroke_width=2, stroke_fill=(0, 0, 0),
        )
        self.img_view[:] = np.array(canvas)

    # ------------- Main loop -------------
    def main_loop(self, hz=30.0):
        rate = rospy.Rate(hz)
        last_state = None
        ik_msg = ""

        while not rospy.is_shutdown():
            self.frame_idx += 1

            ll = self.vr.left_landmarks
            rl = self.vr.right_landmarks
            l_ok = self._l_fresh.is_fresh(ll)
            r_ok = self._r_fresh.is_fresh(rl)
            STALE_OPEN = 1.0  # fake fully-open pinch when hand is out of FOV
            l_pinch = float(np.linalg.norm(ll[THUMB] - ll[MIDDLE])) if l_ok else STALE_OPEN
            r_pinch = float(np.linalg.norm(rl[THUMB] - rl[MIDDLE])) if r_ok else STALE_OPEN
            prev_state = self.fsm.state
            state = self.fsm.update(l_pinch, r_pinch)

            # Lock the head origin only on the FIRST engage (IDLE->ENGAGED).
            # Coming back from DISARMED (cancelled pause) must NOT re-anchor,
            # or the arm would jump mid-teleop.
            if prev_state is State.IDLE and state is State.ENGAGED:
                self.head_pose_at_lock = self.vr.head_matrix.copy()
                hp = self.head_pose_at_lock[:3, 3]
                print(f"[teleop] ENGAGED. head origin xyz={hp.round(3)}")

            target_pos = None
            target_rpy = None

            if state is State.ENGAGED and self.head_pose_at_lock is not None:
                head_now = self.vr.head_matrix
                target_pos, target_rpy = self.compute_target_pose(head_now)
                # Seed IK with the last command, NOT the puppet feedback.
                # Feedback lags command, so seeding with it can flip the IK
                # solver to a different (uglier) solution branch each frame,
                # which shows up as the arm jittering / hopping at certain poses.
                seed = self.target_q if self.target_q is not None else (
                    list(self.joint.position[:6]) if self.joint else None
                )
                sol, ok, ik_msg_now = self.ik.solve(
                    target_pos, target_rpy, gripper=INITIAL_GRIPPER, motorstate=seed
                )
                if ok:
                    # Per-joint step limit: clip IK output to +/- max_joint_step
                    # of the previous command. Catches residual big jumps the
                    # seed-fix alone can't kill (e.g., wrist near gimbal lock).
                    sol_arr = np.asarray(sol, dtype=float)
                    if self.target_q is not None:
                        prev = np.asarray(self.target_q, dtype=float)
                        step = self.args.max_joint_step
                        delta = sol_arr - prev
                        if np.max(np.abs(delta)) > step:
                            sol_arr = np.clip(sol_arr, prev - step, prev + step)
                            ik_msg = "clipped"
                        else:
                            ik_msg = ""
                    else:
                        ik_msg = ""
                    self.target_q = sol_arr.tolist()
                else:
                    ik_msg = f"IK fail: {ik_msg_now}"
            else:
                ik_msg = ""

            self.publish_joints(self.target_q, INITIAL_GRIPPER)
            self.state_pub.publish(String(data=state.value))
            self.update_hud(state, target_pos, target_rpy, ik_msg)

            now = time.monotonic()
            transitioned = state is not last_state
            if transitioned or (now - self.last_print) > 1.0:
                tag = " <- TRANSITION" if transitioned else ""
                target_str = ""
                if target_pos is not None:
                    target_str = (
                        f" target=({target_pos[0]:+.3f},{target_pos[1]:+.3f},{target_pos[2]:+.3f})"
                    )
                print(
                    f"[{self.frame_idx:5d}] {state.value:<8} "
                    f"L={l_pinch:.3f} R={r_pinch:.3f}{target_str}{tag}",
                    flush=True,
                )
                self.last_print = now
            last_state = state
            rate.sleep()

    def run(self):
        print("=" * 60)
        print(f"AVP head -> Piper {self.arm.upper()} EE teleop")
        print("On AVP Safari open:")
        print("    https://10.7.132.66:8012?ws=wss://10.7.132.66:8012")
        print(f"  arm                = {self.arm}")
        print(f"  joint_topic        = {self.args.joint_topic}")
        print(f"  cmd_topic          = {self.args.cmd_topic}")
        print(f"  camera_topic       = {self.args.camera_topic}")
        print(f"  INITIAL_ARM_JOINTS = {INITIAL_ARM_JOINTS}")
        print(f"  INITIAL_GRIPPER    = {INITIAL_GRIPPER}")
        print(f"  scale              = {self.scale}")
        print("=" * 60)
        print("[teleop] Waiting for joint feedback (10s timeout)...")
        if not self.wait_feedback(10.0):
            print(f"[teleop] Timed out. Is {self.args.joint_topic} being published?")
            return
        print(f"[teleop] {self.arm} joint = {np.array(self.joint.position).round(3).tolist()}")
        self.boot_ramp_to_initial(duration=self.args.boot_duration)
        print("[teleop] Pinch BOTH hands (thumb+middle) to ENGAGE; pinch both again then "
              "HOLD 4s to PAUSE.")
        self.main_loop()


def get_args():
    p = argparse.ArgumentParser(description="AVP head-pose -> Piper single-arm EE teleop")
    p.add_argument("--arm", type=str, default=None,
                   help="Which arm to control: l/m/r or left/mid/right. Default prompt at startup; pass 'm' for mid.")

    # Derived from --arm; can be overridden.
    p.add_argument("--joint_topic",  type=str, default=None)
    p.add_argument("--cmd_topic",    type=str, default=None)
    p.add_argument("--camera_topic", type=str, default=None,
                   help="Color camera topic for the chosen arm; piped into the AVP HUD background.")

    default_urdf = (
        "/home/agilex/cobot_magic/Piper_ros_private-ros-noetic/src/piper_description/urdf/"
        "piper_description_new.urdf"
    )
    p.add_argument("--urdf", type=str, default=default_urdf)

    p.add_argument("--scale", type=float, default=1.0,
                   help="Position-only scale factor: head delta * scale = EE delta.")
    p.add_argument("--boot_duration", type=float, default=3.0,
                   help="Seconds to ramp from current arm pose to INITIAL_ARM_JOINTS.")
    p.add_argument("--max_joint_step", type=float, default=0.05,
                   help="Max per-joint change per main-loop frame in rad. "
                        "0.05 rad/frame @ 30 Hz = 1.5 rad/s ~= 86 deg/s. Caps IK jumps.")

    args = p.parse_args()

    args.arm = resolve_arm(args.arm)
    if args.joint_topic  is None: args.joint_topic  = "/puppet/joint_%s"  % args.arm
    if args.cmd_topic    is None: args.cmd_topic    = "/master/joint_%s"  % args.arm
    if args.camera_topic is None:
        args.camera_topic = "/camera_%s/color/image_raw" % ARM_CAM_SHORT[args.arm]

    return args


def main():
    args = get_args()
    ctrl = AvpEefController(args)
    try:
        ctrl.run()
    except KeyboardInterrupt:
        print("\n[teleop] stopping.")
    finally:
        try:
            ctrl.shm.close()
            ctrl.shm.unlink()
        except Exception:
            pass


if __name__ == "__main__":
    main()
