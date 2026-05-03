#!/usr/bin/env python3
"""
Step B: AVP head pose -> Piper right-arm EE teleop.

Single-process: Vuer subprocess + main thread (rospy + IK + publish + HUD).
Right arm follows the operator's head; left arm holds, gripper fixed at 0.1.

Run:
  conda activate aloha
  cd .../Piper-AVP-Teleop/teleop
  python eef_avp_control.py
"""

# Initial right-arm EE pose (Piper base frame) used as the AVP-teleop reset anchor.
# Format: (x, y, z, roll, pitch, yaw, gripper) — meters / radians / meters.
INITIAL_ARM_POSE = (0.065, -0.00, 0.38, 0.0, 0.0, 0.0, 0.1)

# Right-arm-only test. Left arm is held by echoing its current joint state.

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

from eef_keyboard_control import PinocchioIKSolver  # noqa: E402  (loads casadi)
import pinocchio as pin  # noqa: E402  (already in-process via the line above)
from avp_gesture_test import (  # noqa: E402
    GestureStateMachine, State, THUMB, MIDDLE, STATE_COLOR, STATE_HINT,
)
from tele_vision import OpenTeleVision  # noqa: E402

# ROS imports come after casadi is in.
import rospy  # noqa: E402
from sensor_msgs.msg import JointState, Image as ImageMsg  # noqa: E402
from std_msgs.msg import Header  # noqa: E402
from tf.transformations import euler_matrix, euler_from_matrix  # noqa: E402
from PIL import Image, ImageDraw, ImageFont  # noqa: E402

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
        self.scale = args.scale

        # Vuer
        self.shm = _make_shm()
        self.img_view = np.ndarray(IMG_SHAPE, dtype=np.uint8, buffer=self.shm.buf)
        self.vr = OpenTeleVision(
            IMG_SHAPE[:2], SHM_NAME, stereo=False, cert_file=CERT, key_file=KEY
        )

        # ROS
        rospy.init_node("eef_avp_teleop", anonymous=True)
        self.left_joint = None
        self.right_joint = None
        self.latest_camera_frame = None  # numpy uint8 (H, W, 3) RGB
        rospy.Subscriber(args.left_joint_topic, JointState, self._left_joint_cb, queue_size=50)
        rospy.Subscriber(args.right_joint_topic, JointState, self._right_joint_cb, queue_size=50)
        rospy.Subscriber(
            args.right_camera_topic, ImageMsg, self._right_camera_cb, queue_size=1, buff_size=2 ** 24
        )
        self.left_pub = rospy.Publisher(args.left_cmd_topic, JointState, queue_size=10)
        self.right_pub = rospy.Publisher(args.right_cmd_topic, JointState, queue_size=10)

        # IK (right arm only; left held)
        self.right_ik = PinocchioIKSolver(args.right_urdf)

        # State machine
        self.fsm = GestureStateMachine()
        self.head_pose_at_lock = None  # (4, 4) head pose in AVP world at lock time
        self.frame_idx = 0
        self.last_print = 0.0

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
    def _left_joint_cb(self, msg):  self.left_joint = msg
    def _right_joint_cb(self, msg): self.right_joint = msg

    def _right_camera_cb(self, msg):
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
            if (
                self.left_joint is not None and len(self.left_joint.position) >= 6
                and self.right_joint is not None and len(self.right_joint.position) >= 6
            ):
                return True
            if rospy.Time.now() > deadline:
                return False
            rate.sleep()
        return False

    def publish_joints(self, left6, right6, left_g, right_g):
        names = ["joint0", "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        m_left = JointState(); m_left.header = Header(stamp=rospy.Time.now())
        m_left.name = names; m_left.position = list(left6) + [left_g]
        m_right = JointState(); m_right.header = Header(stamp=rospy.Time.now())
        m_right.name = names; m_right.position = list(right6) + [right_g]
        self.left_pub.publish(m_left)
        self.right_pub.publish(m_right)

    # ------------- FK helper (for verification / debug) -------------
    def _fk_right(self, q6):
        """Forward-kinematics on the IK reduced model. Returns (xyz, rpy) of EE."""
        q = np.asarray(q6, dtype=float).flatten()
        pin.framesForwardKinematics(self.right_ik.model, self.right_ik.data, q)
        se3 = self.right_ik.data.oMf[self.right_ik.ee_frame_id]
        xyz = np.asarray(se3.translation, dtype=float).flatten()
        rpy = np.array(euler_from_matrix(se3.rotation), dtype=float)
        return xyz, rpy

    # ------------- Boot ramp -------------
    def boot_ramp_to_initial(self, duration=3.0, hz=30.0):
        seed = list(self.right_joint.position[:6]) if self.right_joint else None
        sol, ok, msg = self.right_ik.solve(
            INITIAL_ARM_POSE[:3], INITIAL_ARM_POSE[3:6], gripper=0.1, motorstate=seed
        )
        if not ok:
            raise RuntimeError(f"IK failed for INITIAL_ARM_POSE: {msg}")
        target_q = np.asarray(sol, dtype=float)
        current_q = np.asarray(self.right_joint.position[:6], dtype=float)
        steps = max(1, int(duration * hz))
        rate = rospy.Rate(hz)
        print(f"[teleop] Boot ramp ({duration}s): {current_q.round(3)} -> {target_q.round(3)}")
        for i in range(steps):
            if rospy.is_shutdown():
                return
            alpha = (i + 1) / steps
            interp = (1.0 - alpha) * current_q + alpha * target_q
            left6 = list(self.left_joint.position[:6])
            self.publish_joints(left6, list(interp), 0.1, 0.1)
            self._draw_hud_text(["BOOTING", f"alpha = {alpha:.2f}"], colors=[(255, 200, 0), (200, 200, 200)])
            rate.sleep()
        print("[teleop] Boot ramp done.")

        # Verify the IK solution actually corresponds to INITIAL_ARM_POSE.
        fk_xyz, fk_rpy = self._fk_right(target_q)
        tgt_xyz = np.array(INITIAL_ARM_POSE[:3])
        tgt_rpy = np.array(INITIAL_ARM_POSE[3:6])
        print(
            f"[teleop] FK on target_q : xyz={fk_xyz.round(4).tolist()}  "
            f"rpy(deg)={np.rad2deg(fk_rpy).round(2).tolist()}"
        )
        print(
            f"[teleop] INITIAL_ARM_POSE: xyz={tgt_xyz.round(4).tolist()}  "
            f"rpy(deg)={np.rad2deg(tgt_rpy).round(2).tolist()}"
        )
        print(
            f"[teleop] residual        : dpos={(fk_xyz - tgt_xyz).round(4).tolist()}  "
            f"drpy(deg)={np.rad2deg(fk_rpy - tgt_rpy).round(2).tolist()}"
        )

    # ------------- Pose math -------------
    def compute_target_pose(self, head_now):
        """World-frame composition: target = R_remap(delta_avp) * INITIAL_ARM_POSE."""
        head_lock = self.head_pose_at_lock
        delta_pos_avp = head_now[:3, 3] - head_lock[:3, 3]
        delta_R_avp = head_now[:3, :3] @ head_lock[:3, :3].T

        delta_pos_piper = R_AVP_TO_PIPER @ delta_pos_avp * self.scale
        delta_R_piper = R_AVP_TO_PIPER @ delta_R_avp @ R_AVP_TO_PIPER.T

        initial_xyz = np.array(INITIAL_ARM_POSE[:3], dtype=float)
        initial_R = euler_matrix(*INITIAL_ARM_POSE[3:6])[:3, :3]

        target_pos = initial_xyz + delta_pos_piper
        target_R = delta_R_piper @ initial_R
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

        # State name + hint at top
        centered(state.value, 10, self.font_big, STATE_COLOR[state])
        for k, line in enumerate(STATE_HINT[state]):
            centered(line, 80 + k * 32, self.font, (220, 220, 220))

        # Target pose at bottom
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
        # frame# bottom-right
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

            # Gesture
            ll = self.vr.left_landmarks
            rl = self.vr.right_landmarks
            l_pinch = float(np.linalg.norm(ll[THUMB] - ll[MIDDLE]))
            r_pinch = float(np.linalg.norm(rl[THUMB] - rl[MIDDLE]))
            prev_state = self.fsm.state
            state = self.fsm.update(l_pinch, r_pinch)

            # On IDLE -> LOCKED transition: capture origin head pose.
            if prev_state is State.IDLE and state is State.LOCKED:
                self.head_pose_at_lock = self.vr.head_matrix.copy()
                hp = self.head_pose_at_lock[:3, 3]
                print(f"[teleop] LOCKED. head xyz={hp.round(3)}")

            # Build joint commands
            left6_hold = list(self.left_joint.position[:6]) if self.left_joint else [0.0] * 6
            right6_hold = list(self.right_joint.position[:6]) if self.right_joint else [0.0] * 6
            target_pos = None
            target_rpy = None

            if state is State.ENGAGED and self.head_pose_at_lock is not None:
                head_now = self.vr.head_matrix
                target_pos, target_rpy = self.compute_target_pose(head_now)
                seed = list(self.right_joint.position[:6]) if self.right_joint else None
                sol, ok, ik_msg_now = self.right_ik.solve(
                    target_pos, target_rpy, gripper=0.1, motorstate=seed
                )
                if ok:
                    right6_to_send = sol
                    ik_msg = ""
                else:
                    right6_to_send = right6_hold
                    ik_msg = f"IK fail: {ik_msg_now}"
            else:
                right6_to_send = right6_hold
                ik_msg = ""

            self.publish_joints(left6_hold, right6_to_send, 0.1, 0.1)
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
        print("AVP head -> Piper right EE teleop")
        print("On AVP Safari open:")
        print("    https://10.7.132.66:8012?ws=wss://10.7.132.66:8012")
        print(f"INITIAL_ARM_POSE = {INITIAL_ARM_POSE}")
        print(f"scale = {self.scale}")
        print("=" * 60)
        print("[teleop] Waiting for joint feedback (10s timeout)...")
        if not self.wait_feedback(10.0):
            print("[teleop] Timed out. Is /puppet/joint_left,right being published?")
            return
        print(f"[teleop] left  joint = {np.array(self.left_joint.position).round(3).tolist()}")
        print(f"[teleop] right joint = {np.array(self.right_joint.position).round(3).tolist()}")
        self.boot_ramp_to_initial(duration=self.args.boot_duration)
        print("[teleop] Pinch L thumb+middle to LOCK, then HOLD R thumb+middle to ENGAGE.")
        self.main_loop()


def get_args():
    p = argparse.ArgumentParser(description="AVP head-pose -> Piper right EE teleop")
    p.add_argument("--left_joint_topic",  type=str, default="/puppet/joint_left")
    p.add_argument("--right_joint_topic", type=str, default="/puppet/joint_right")
    p.add_argument("--left_cmd_topic",    type=str, default="/master/joint_left")
    p.add_argument("--right_cmd_topic",   type=str, default="/master/joint_right")
    default_urdf = (
        "/home/agilex/cobot_magic/Piper_ros_private-ros-noetic/src/piper_description/urdf/"
        "piper_description_new.urdf"
    )
    p.add_argument("--right_urdf", type=str, default=default_urdf)
    p.add_argument("--right_camera_topic", type=str, default="/camera_r/color/image_raw",
                   help="Right-arm color camera topic; piped into the AVP HUD background.")
    p.add_argument("--scale", type=float, default=1.0,
                   help="Position-only scale factor: head delta * scale = EE delta.")
    p.add_argument("--boot_duration", type=float, default=3.0,
                   help="Seconds to ramp from current right-arm pose to INITIAL_ARM_POSE.")
    return p.parse_args()


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
