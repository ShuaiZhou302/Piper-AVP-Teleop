#!/usr/bin/env python3
"""
Quest 2 dual-arm EEF teleop (robot side, runs on cobot_magic in `aloha`).

Receives raw HMD + left/right controller poses from quest_client.py (Windows)
over a TCP socket (reached through an SSH -L tunnel -- this process only
ever binds 127.0.0.1, it is not meant to be reachable except through that
tunnel). Left controller drives the LEFT arm's end-effector, right
controller drives the RIGHT arm's, exactly the way eef_avp_control_singlearm.py
drives one arm from AVP head motion -- same delta-pose-composition math,
same Pinocchio IK solver, same per-joint step clamp. Head pose is received
and could later drive a camera gimbal, but nothing consumes it yet.

Safety model (all enforced here, not on the Windows side -- the network
sensor relay is not trusted):
  - Clutch: an arm only moves while that controller's GRIP is held. Grip
    press latches the current hand pose as the delta-tracking origin (like
    head_pose_at_lock in the AVP script); grip release freezes the arm in
    place (holds last commanded joints) -- no drift, no auto-return.
  - Staleness watchdog: if no fresh packet arrives within --stale_freeze_sec,
    both arms are forced to the frozen (disengaged) state regardless of the
    last button reading, because stale button state cannot be trusted.
  - E-stop ramp-home: if no fresh packet arrives within --stale_estop_sec
    (network dead, client crashed, ...), both arms drive back to
    INITIAL_ARM_JOINTS at a capped joint speed (--return_speed_rad_s), the
    same speed-limited return used at episode end in the AVP script. Also
    triggered manually by the A/X button on either controller (accessible
    panic stop from inside the headset).
  - Trigger (analog) maps to gripper opening: released = open, fully
    squeezed = closed (intuitive "squeeze to grab").

Run (on cobot_magic, after CAN init + `roslaunch piper start_ms_piper.launch ...`):
    conda activate aloha
    cd .../Piper-AVP-Teleop/quest
    python quest_server.py
"""

# See eef_avp_control_singlearm.py: casadi (via PinocchioIKSolver) must be
# imported BEFORE rospy, or rospy's libstdc++ shadowing breaks casadi's import.
INITIAL_ARM_JOINTS = (0.0463, 0.5300, -0.5562, 0.000, 0.6500, 0.0000)
INITIAL_GRIPPER = 0.1  # fully open

import argparse
import os
import socket
import sys
import threading
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TELEOP_DIR = os.path.normpath(os.path.join(HERE, "..", "teleop"))
sys.path.insert(0, TELEOP_DIR)
sys.path.insert(0, HERE)

from eef_keyboard_control_singlearm import PinocchioIKSolver  # noqa: E402  (loads casadi)
import pinocchio as pin  # noqa: E402

import rospy  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import Header  # noqa: E402
from tf.transformations import euler_matrix, euler_from_matrix  # noqa: E402

from protocol import FrameReader, DEFAULT_PORT, wire_to_pose  # noqa: E402


# Quest / OpenVR standing world (right / up / back) -> Piper world
# (forward / left / up). Numerically identical to R_AVP_TO_PIPER in
# eef_avp_control_singlearm.py: OpenVR's standing universe uses the same
# right-handed, +Y-up, +Z-back convention as the WebXR frame AVP reports
# (see avp/Readme.md section 9; re-verified for SteamVR in quest/Readme.md).
R_QUEST_TO_PIPER = np.array([
    [0, 0, -1],
    [-1, 0, 0],
    [0, 1, 0],
], dtype=float)

DEFAULT_URDF = (
    "/home/agilex/cobot_magic/Piper_ros_private-ros-noetic/src/piper_description/urdf/"
    "piper_description_new.urdf"
)


def remap_to_piper(T_quest):
    """4x4 pose in OpenVR world -> 4x4 pose in Piper world (rotation-only remap)."""
    T = np.eye(4)
    T[:3, 3] = R_QUEST_TO_PIPER @ T_quest[:3, 3]
    T[:3, :3] = R_QUEST_TO_PIPER @ T_quest[:3, :3] @ R_QUEST_TO_PIPER.T
    return T


class ArmChannel(object):
    """One arm's IK/state/clutch logic. Mirrors AvpEefController but keyed
    on a hand controller instead of the head, and non-blocking (single step()
    call per main-loop tick instead of a dedicated blocking ramp loop)."""

    def __init__(self, name, urdf_path, joint_topic, cmd_topic, pub_queue_size=10):
        self.name = name
        self.ik = PinocchioIKSolver(urdf_path)
        self.joint = None  # latest /puppet feedback (JointState)
        self.pub = rospy.Publisher(cmd_topic, JointState, queue_size=pub_queue_size)
        rospy.Subscriber(joint_topic, JointState, self._joint_cb, queue_size=50)

        self.engaged = False
        self.prev_grip = False
        self.lock_hand_T = None       # (4,4) hand pose in Piper world at grip-press
        self.target_q = None          # last commanded 6 joints
        self.gripper = INITIAL_GRIPPER
        self.initial_xyz = None
        self.initial_R = None
        self.returning_home = False

    def _joint_cb(self, msg):
        self.joint = msg

    def has_feedback(self):
        return self.joint is not None and len(self.joint.position) >= 6

    def fk(self, q6):
        q = np.asarray(q6, dtype=float).flatten()
        pin.framesForwardKinematics(self.ik.model, self.ik.data, q)
        se3 = self.ik.data.oMf[self.ik.ee_frame_id]
        xyz = np.asarray(se3.translation, dtype=float).flatten()
        rpy = np.array(euler_from_matrix(se3.rotation), dtype=float)
        return xyz, rpy

    def publish(self):
        names = ["joint0", "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        m = JointState()
        m.header = Header(stamp=rospy.Time.now())
        m.name = names
        m.position = list(self.target_q) + [self.gripper]
        self.pub.publish(m)

    def boot_ramp_to_initial(self, duration, hz=30.0):
        target_q = np.asarray(INITIAL_ARM_JOINTS, dtype=float)
        current_q = np.asarray(self.joint.position[:6], dtype=float)
        steps = max(1, int(duration * hz))
        rate = rospy.Rate(hz)
        print("[%s] boot ramp (%.1fs): %s -> %s" %
              (self.name, duration, current_q.round(3), target_q.round(3)))
        for i in range(steps):
            if rospy.is_shutdown():
                return
            alpha = (i + 1) / steps
            interp = (1.0 - alpha) * current_q + alpha * target_q
            self.target_q = interp.tolist()
            self.gripper = INITIAL_GRIPPER
            self.publish()
            rate.sleep()
        self.target_q = target_q.tolist()
        fk_xyz, fk_rpy = self.fk(target_q)
        self.initial_xyz = fk_xyz
        self.initial_R = euler_matrix(*fk_rpy)[:3, :3]
        print("[%s] boot ramp done. FK anchor xyz=%s" % (self.name, fk_xyz.round(4)))

    def on_grip_edge(self, grip_pressed, hand_T_piper):
        """Call every tick with the current grip state; handles rising/falling edges."""
        if grip_pressed and not self.prev_grip:
            self.lock_hand_T = hand_T_piper.copy()
            self.engaged = True
            print("[%s] ENGAGED" % self.name)
        elif not grip_pressed and self.prev_grip:
            self.engaged = False
            print("[%s] disengaged (frozen)" % self.name)
        self.prev_grip = grip_pressed

    def force_disengage(self):
        self.engaged = False
        self.prev_grip = False

    def track_step(self, hand_T_piper, scale, max_joint_step, gripper_trigger,
                    gripper_min, gripper_max, respect_collision):
        """One IK-tracking tick while engaged. Returns an ik status string ('' = ok)."""
        delta_pos = hand_T_piper[:3, 3] - self.lock_hand_T[:3, 3]
        delta_R = hand_T_piper[:3, :3] @ self.lock_hand_T[:3, :3].T
        target_pos = self.initial_xyz + delta_pos * scale
        target_R = delta_R @ self.initial_R
        target_rpy = np.array(euler_from_matrix(target_R), dtype=float)

        seed = self.target_q if self.target_q is not None else (
            list(self.joint.position[:6]) if self.joint else None
        )
        sol, ok, msg = self.ik.solve(
            target_pos, target_rpy, gripper=INITIAL_GRIPPER, motorstate=seed,
            allow_collision=not respect_collision,
        )
        if not ok:
            return "IK fail: %s" % msg

        sol_arr = np.asarray(sol, dtype=float)
        if self.target_q is not None:
            prev = np.asarray(self.target_q, dtype=float)
            delta = sol_arr - prev
            if np.max(np.abs(delta)) > max_joint_step:
                sol_arr = np.clip(sol_arr, prev - max_joint_step, prev + max_joint_step)
        self.target_q = sol_arr.tolist()

        # Trigger: released (0) = open (gripper_max), squeezed (1) = closed (gripper_min).
        self.gripper = gripper_max - gripper_trigger * (gripper_max - gripper_min)
        return ""

    def home_step(self, max_step_rad):
        """One capped-speed step toward INITIAL_ARM_JOINTS. Returns True once arrived."""
        self.returning_home = True
        self.engaged = False
        target = np.asarray(INITIAL_ARM_JOINTS, dtype=float)
        cur = np.asarray(self.target_q, dtype=float)
        delta = target - cur
        step = np.clip(delta, -max_step_rad, max_step_rad)
        new_q = cur + step
        self.target_q = new_q.tolist()
        self.gripper = INITIAL_GRIPPER
        arrived = bool(np.max(np.abs(target - new_q)) < 1e-3)
        if arrived:
            self.returning_home = False
        return arrived


class QuestTeleopServer(object):
    def __init__(self, args):
        self.args = args
        rospy.init_node("quest_teleop", anonymous=True)

        self.left = ArmChannel("left", args.left_urdf, args.left_joint_topic,
                                args.left_cmd_topic)
        self.right = ArmChannel("right", args.right_urdf, args.right_joint_topic,
                                 args.right_cmd_topic)

        self._lock = threading.Lock()
        self._latest = None
        self._last_recv_mono = None
        self._server_sock = None
        self._stop = False

    # ---------- networking (background thread) ----------
    def _accept_loop(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((self.args.host, self.args.port))
        s.listen(1)
        self._server_sock = s
        print("[net] listening on %s:%d" % (self.args.host, self.args.port))
        while not self._stop:
            try:
                conn, addr = s.accept()
            except OSError:
                break
            print("[net] client connected: %s" % (addr,))
            conn.settimeout(2.0)
            reader = FrameReader(conn)
            try:
                while not self._stop:
                    try:
                        frame = reader.read()
                    except socket.timeout:
                        continue
                    if frame is None:
                        break
                    with self._lock:
                        self._latest = frame
                        self._last_recv_mono = time.monotonic()
            except (OSError, ValueError) as e:
                print("[net] recv error: %s" % e)
            finally:
                conn.close()
                print("[net] client disconnected; waiting for reconnect")

    def _snapshot(self):
        with self._lock:
            return self._latest, self._last_recv_mono

    # ---------- boot ----------
    def wait_feedback(self, timeout_sec):
        deadline = rospy.Time.now() + rospy.Duration(timeout_sec)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.left.has_feedback() and self.right.has_feedback():
                return True
            if rospy.Time.now() > deadline:
                return False
            rate.sleep()
        return False

    # ---------- main loop ----------
    def _hand_from_frame(self, frame, side):
        if frame is None:
            return None, None
        d = frame.get(side)
        if not d or not d.get("connected") or not d.get("valid"):
            return None, None
        T_piper = remap_to_piper(wire_to_pose(d))
        return T_piper, d.get("buttons")

    def run(self):
        print("=" * 60)
        print("Quest dual-arm EEF teleop")
        print("  listen              = %s:%d" % (self.args.host, self.args.port))
        print("  left  joint/cmd     = %s / %s" % (self.args.left_joint_topic, self.args.left_cmd_topic))
        print("  right joint/cmd     = %s / %s" % (self.args.right_joint_topic, self.args.right_cmd_topic))
        print("  stale_freeze_sec    = %.2f" % self.args.stale_freeze_sec)
        print("  stale_estop_sec     = %.2f" % self.args.stale_estop_sec)
        print("  return_speed_rad_s  = %.2f" % self.args.return_speed_rad_s)
        print("=" * 60)

        net_thread = threading.Thread(target=self._accept_loop, daemon=True)
        net_thread.start()

        print("[boot] waiting for /puppet joint feedback (10s timeout)...")
        if not self.wait_feedback(10.0):
            print("[boot] timed out waiting for joint feedback. Is the piper driver running?")
            return
        self.left.boot_ramp_to_initial(self.args.boot_duration)
        self.right.boot_ramp_to_initial(self.args.boot_duration)
        print("[teleop] ready. Hold GRIP on a controller to engage that arm; "
              "TRIGGER controls gripper; A/X on either controller = panic ramp-home.")

        hz = self.args.rate
        rate = rospy.Rate(hz)
        home_step_rad = self.args.return_speed_rad_s / hz
        max_joint_step = self.args.max_joint_step
        last_print = 0.0

        while not rospy.is_shutdown():
            frame, last_recv = self._snapshot()
            recv_age = (time.monotonic() - last_recv) if last_recv is not None else float("inf")
            stale_freeze = recv_age > self.args.stale_freeze_sec
            stale_estop = recv_age > self.args.stale_estop_sec

            l_T, l_btn = (None, None) if stale_freeze else self._hand_from_frame(frame, "left")
            r_T, r_btn = (None, None) if stale_freeze else self._hand_from_frame(frame, "right")

            panic = False
            for btn in (l_btn, r_btn):
                if btn and btn.get("button_ax"):
                    panic = True

            ik_msgs = []
            for chan, T_piper, btn in ((self.left, l_T, l_btn), (self.right, r_T, r_btn)):
                if stale_estop or panic:
                    chan.force_disengage()
                    if chan.target_q is not None:
                        chan.home_step(home_step_rad)
                    chan.publish()
                    continue

                if stale_freeze or T_piper is None:
                    chan.force_disengage()
                    if chan.target_q is not None:
                        chan.publish()  # hold last commanded pose
                    continue

                grip_pressed = bool(btn and btn.get("grip_pressed"))
                chan.on_grip_edge(grip_pressed, T_piper)

                if chan.engaged:
                    trig = float(btn.get("trigger", 0.0)) if btn else 0.0
                    msg = chan.track_step(
                        T_piper, self.args.scale, max_joint_step, trig,
                        self.args.gripper_min, self.args.gripper_max,
                        self.args.respect_collision,
                    )
                    if msg:
                        ik_msgs.append("%s: %s" % (chan.name, msg))

                if chan.target_q is not None:
                    chan.publish()

            now = time.monotonic()
            if now - last_print > 1.0:
                status = "STALE(%.1fs)" % recv_age if stale_freeze else "live"
                print("[teleop] %-16s L=%s%s R=%s%s%s" % (
                    status,
                    "ENGAGED" if self.left.engaged else ("HOME" if self.left.returning_home else "idle"),
                    "*" if panic else "",
                    "ENGAGED" if self.right.engaged else ("HOME" if self.right.returning_home else "idle"),
                    "*" if panic else "",
                    ("  " + "; ".join(ik_msgs)) if ik_msgs else "",
                ))
                last_print = now

            rate.sleep()

        self._stop = True
        if self._server_sock is not None:
            try:
                self._server_sock.close()
            except OSError:
                pass


def get_args():
    p = argparse.ArgumentParser(description="Quest 2 -> Piper dual-arm EEF teleop (robot side)")
    p.add_argument("--host", default="127.0.0.1",
                    help="Bind address. Keep this 127.0.0.1 -- reachability is meant to "
                         "come ONLY from the SSH -L tunnel, not the open network.")
    p.add_argument("--port", type=int, default=DEFAULT_PORT)
    p.add_argument("--rate", type=float, default=50.0, help="Control loop rate (Hz).")

    p.add_argument("--left_urdf", default=DEFAULT_URDF)
    p.add_argument("--right_urdf", default=DEFAULT_URDF)
    p.add_argument("--left_joint_topic", default="/puppet/joint_left")
    p.add_argument("--left_cmd_topic", default="/master/joint_left")
    p.add_argument("--right_joint_topic", default="/puppet/joint_right")
    p.add_argument("--right_cmd_topic", default="/master/joint_right")

    p.add_argument("--scale", type=float, default=1.0,
                    help="Position-only scale factor: hand delta * scale = EE delta.")
    p.add_argument("--boot_duration", type=float, default=3.0)
    p.add_argument("--return_speed_rad_s", type=float, default=0.3,
                    help="Max joint speed while auto-returning home (e-stop / panic).")
    p.add_argument("--max_joint_step", type=float, default=0.05,
                    help="Max per-joint change per control tick (rad) during normal tracking.")
    p.add_argument("--stale_freeze_sec", type=float, default=0.2,
                    help="No fresh packet for longer than this -> freeze both arms in place.")
    p.add_argument("--stale_estop_sec", type=float, default=1.0,
                    help="No fresh packet for longer than this -> ramp both arms home.")
    p.add_argument("--gripper_min", type=float, default=0.0)
    p.add_argument("--gripper_max", type=float, default=0.1)
    p.add_argument("--respect_collision", action="store_true")

    return p.parse_args()


def main():
    args = get_args()
    server = QuestTeleopServer(args)
    try:
        server.run()
    except KeyboardInterrupt:
        print("\n[teleop] stopping.")


if __name__ == "__main__":
    main()
