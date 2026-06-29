#!/usr/bin/env python3
"""
Move each Piper arm TCP to the same point expressed in a shared world frame.

Shared-frame convention used here:
  - unified frame has the same orientation as mid arm base_link
  - unified origin is 0.25 m lower than mid arm base_link
  - Piper axes: +x forward, +y left, +z up

Run with all 3 arms in ROS command mode:
  roslaunch .../start_ms_piper_3arm.launch mode:=1 auto_enable:=true
"""
import argparse
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

# IMPORTANT: import casadi (via eef_keyboard_control_singlearm) before modules
# that import rospy/tf, otherwise Ubuntu's system libstdc++ may be loaded first.
from eef_keyboard_control_singlearm import PinocchioIKSolver  # noqa: E402
from arm_unified_coords import (  # noqa: E402
    ArmUnifiedConverter,
    DEFAULT_EXTRINSICS_JSON,
)

import pinocchio as pin  # noqa: E402
import rospy  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import Header  # noqa: E402
from tf.transformations import euler_from_matrix  # noqa: E402


ARM_ORDER = ("mid", "left", "right")
JOINT_NAMES = ["joint0", "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
INITIAL_ARM_JOINTS = (0.0463, 0.5300, -0.5562, 0.0000, 0.6500, 0.0000)
INITIAL_GRIPPER = 0.1


class TouchPointTester:
    def __init__(self, args):
        self.args = args
        rospy.init_node("touch_same_point_3arm", anonymous=True)
        self.ik = PinocchioIKSolver(args.urdf)
        self.converter = ArmUnifiedConverter(args.extrinsics_json)
        self.puppet = {arm: None for arm in ARM_ORDER}
        self.pubs = {}
        for arm in ARM_ORDER:
            rospy.Subscriber(
                f"/puppet/joint_{arm}",
                JointState,
                self._make_joint_cb(arm),
                queue_size=50,
                tcp_nodelay=True,
            )
            self.pubs[arm] = rospy.Publisher(
                f"/master/joint_{arm}",
                JointState,
                queue_size=10,
            )

    def _make_joint_cb(self, arm):
        def cb(msg):
            self.puppet[arm] = msg
        return cb

    def wait_feedback(self, arms, timeout_s):
        deadline = rospy.Time.now() + rospy.Duration(timeout_s)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            ready = all(
                self.puppet[arm] is not None and len(self.puppet[arm].position) >= 7
                for arm in arms
            )
            if ready:
                return True
            if rospy.Time.now() > deadline:
                return False
            rate.sleep()
        return False

    def publish(self, arm, seven):
        msg = JointState()
        msg.header = Header(stamp=rospy.Time.now())
        msg.name = JOINT_NAMES
        msg.position = list(seven)
        self.pubs[arm].publish(msg)

    def ramp_one(self, arm, target7):
        current = np.asarray(self.puppet[arm].position[:7], dtype=float)
        target = np.asarray(target7, dtype=float)
        max_delta = float(np.max(np.abs(target[:6] - current[:6])))
        duration = max(max_delta / self.args.ramp_speed_rad_s, self.args.min_ramp_sec)
        steps = max(1, int(duration * self.args.hz))
        rate = rospy.Rate(self.args.hz)
        print(
            f"[touch] {arm}: ramp max_delta={max_delta:.3f} rad, "
            f"duration={duration:.2f}s"
        )
        for i in range(steps):
            if rospy.is_shutdown():
                return False
            alpha = float(i + 1) / float(steps)
            cmd = (1.0 - alpha) * current + alpha * target
            self.publish(arm, cmd)
            rate.sleep()
        for _ in range(max(1, int(self.args.hold_sec * self.args.hz))):
            if rospy.is_shutdown():
                return False
            self.publish(arm, target)
            rate.sleep()
        return True

    def ready_rpy(self):
        q = np.asarray(INITIAL_ARM_JOINTS, dtype=float)
        pin.framesForwardKinematics(self.ik.model, self.ik.data, q)
        T = self.ik.data.oMf[self.ik.ee_frame_id]
        return np.array(euler_from_matrix(T.rotation), dtype=float)

    def solve_arm(self, arm, target_world, rpy_world, ready_rpy_arm):
        xyz_arm, rpy_arm = self.converter.arm_xyz_rpy_from_unified(
            arm,
            target_world,
            rpy_unified=rpy_world,
        )
        if rpy_arm is None:
            rpy_arm = np.asarray(ready_rpy_arm, dtype=float)
        seed = np.asarray(self.puppet[arm].position[:6], dtype=float)
        sol6, ok, msg = self.ik.solve(
            xyz_arm,
            rpy_arm,
            gripper=self.args.gripper,
            motorstate=seed,
        )
        if not ok:
            return xyz_arm, rpy_arm, None, msg
        return xyz_arm, rpy_arm, np.array(list(sol6) + [self.args.gripper], dtype=float), msg

    def run(self):
        arms = tuple(self.args.arms)
        target_world = np.asarray(self.args.target_world, dtype=float)
        rpy_world = None if self.args.rpy_world is None else np.asarray(self.args.rpy_world, dtype=float)
        ready_rpy_arm = self.ready_rpy()

        print("=" * 72)
        print("[touch] Shared frame: final unified frame")
        for line in self.converter.summary_lines():
            print(f"[touch] {line}")
        print(f"[touch] target_unified xyz = {target_world.round(4).tolist()}")
        if rpy_world is None:
            print(
                "[touch] target orientation: each arm uses ready-pose local EE rpy = "
                f"{ready_rpy_arm.round(4).tolist()}"
            )
        else:
            print(f"[touch] target_unified rpy = {rpy_world.round(4).tolist()}")
        print(f"[touch] arms = {', '.join(arms)}")
        print("=" * 72)

        print("[touch] waiting for /puppet/joint_<arm> feedback...")
        if not self.wait_feedback(arms, self.args.feedback_wait):
            print("[touch] timed out. Check roslaunch mode:=1 and arm topics.")
            sys.exit(1)

        plans = {}
        for arm in arms:
            xyz_arm, rpy_arm, target7, msg = self.solve_arm(
                arm, target_world, rpy_world, ready_rpy_arm
            )
            print()
            print(f"[touch] {arm.upper()}")
            print(f"  target in {arm}_base xyz = {xyz_arm.round(4).tolist()}")
            print(f"  target in {arm}_base rpy = {rpy_arm.round(4).tolist()}")
            if target7 is None:
                print(f"  IK FAILED: {msg}")
                continue
            print(f"  target joints = {target7.round(4).tolist()}")
            plans[arm] = target7

        missing = [arm for arm in arms if arm not in plans]
        if missing:
            print(f"\n[touch] abort: IK failed for {missing}")
            sys.exit(1)
        if self.args.dry_run:
            print("\n[touch] dry run only; no commands published.")
            return

        print()
        print("[touch] The script will move arms ONE BY ONE to the same world point.")
        print("[touch] Keep one hand near emergency stop / disable. Clear the workspace.")
        input("[touch] Press ENTER to start, or Ctrl-C to abort.")

        for arm in arms:
            print()
            cur = np.asarray(self.puppet[arm].position[:7], dtype=float)
            reset7 = np.array(list(INITIAL_ARM_JOINTS) + [self.args.gripper], dtype=float)
            print(f"[touch] {arm}: current joints = {cur.round(4).tolist()}")
            print(f"[touch] {arm}: target  joints = {plans[arm].round(4).tolist()}")
            input(f"[touch] Press ENTER to move {arm.upper()} to the shared point.")
            if not self.ramp_one(arm, plans[arm]):
                print("[touch] interrupted.")
                return
            input(
                f"[touch] {arm.upper()} is holding. Inspect contact point, "
                "then ENTER to RESET this arm before continuing."
            )
            if not self.ramp_one(arm, reset7):
                print("[touch] reset interrupted.")
                return
            input(f"[touch] {arm.upper()} reset done. Press ENTER to continue.")

        print("\n[touch] done. Tested arms have been reset to INITIAL_ARM_JOINTS.")


def get_args():
    default_urdf = (
        "/home/agilex/cobot_magic/Piper_ros_private-ros-noetic/src/piper_description/urdf/"
        "piper_description_new.urdf"
    )
    p = argparse.ArgumentParser(
        description="Command mid/left/right Piper TCPs to the same shared-frame point."
    )
    p.add_argument(
        "--target_world",
        nargs=3,
        type=float,
        required=True,
        metavar=("X", "Y", "Z"),
        help="Target point in final unified frame, meters. "
             "Unified origin is 0.25 m lower than mid_base.",
    )
    p.add_argument(
        "--rpy_world",
        nargs=3,
        type=float,
        default=None,
        metavar=("ROLL", "PITCH", "YAW"),
        help="Optional target orientation in shared world frame, radians. "
             "Default: use the local EE rpy of INITIAL_ARM_JOINTS for each arm.",
    )
    p.add_argument(
        "--arms",
        nargs="+",
        choices=ARM_ORDER,
        default=list(ARM_ORDER),
        help="Arms to test, in order. Default: mid left right.",
    )
    p.add_argument("--urdf", type=str, default=default_urdf)
    p.add_argument(
        "--extrinsics_json",
        type=str,
        default=DEFAULT_EXTRINSICS_JSON,
        help="Calibration JSON from calibrate_3arm_touch_points.py. "
             f"Default: {DEFAULT_EXTRINSICS_JSON}",
    )
    p.add_argument("--gripper", type=float, default=INITIAL_GRIPPER)
    p.add_argument("--ramp_speed_rad_s", type=float, default=0.15)
    p.add_argument("--min_ramp_sec", type=float, default=2.0)
    p.add_argument("--hold_sec", type=float, default=1.0)
    p.add_argument("--hz", type=float, default=30.0)
    p.add_argument("--feedback_wait", type=float, default=10.0)
    p.add_argument(
        "--dry_run",
        action="store_true",
        help="Only print transformed targets and IK solutions; do not move hardware.",
    )
    return p.parse_args()


def main():
    args = get_args()
    try:
        TouchPointTester(args).run()
    except KeyboardInterrupt:
        print("\n[touch] stopped.")


if __name__ == "__main__":
    main()
