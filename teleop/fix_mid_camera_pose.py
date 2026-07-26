#!/usr/bin/env python3
"""Hold the middle arm at a fixed camera pose.

This is meant for using the middle-arm wrist camera as a fixed viewpoint during
data collection or inference. Start the arm launch first, then run this script.
It publishes joint commands to /master/joint_mid and keeps publishing the target
so other teleop/data-collection processes do not move the middle arm.
"""

import argparse
import math
import time

import numpy as np

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


JOINT_NAMES = ["joint0", "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

# Raw joint[0:6] requested for the fixed middle-arm camera pose.
FIXED_MID_CAMERA_Q6 = np.array([
    0.0,
    0.583641,
    -1.011822,
    0.0,
    1.568702,
    0.0,
], dtype=float)


class FixedMidCameraPose:
    def __init__(self, args):
        self.args = args
        self.feedback = None
        rospy.init_node("fix_mid_camera_pose", anonymous=True)
        rospy.Subscriber(
            args.joint_topic,
            JointState,
            self._joint_cb,
            queue_size=50,
            tcp_nodelay=True,
        )
        self.pub = rospy.Publisher(args.cmd_topic, JointState, queue_size=10)

    def _joint_cb(self, msg):
        self.feedback = msg

    def wait_feedback(self):
        deadline = rospy.Time.now() + rospy.Duration(self.args.wait_timeout)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.feedback is not None and len(self.feedback.position) >= 6:
                return True
            if rospy.Time.now() > deadline:
                return False
            rate.sleep()
        return False

    def publish(self, q7):
        msg = JointState()
        msg.header = Header(stamp=rospy.Time.now())
        msg.name = JOINT_NAMES
        msg.position = [float(x) for x in q7]
        self.pub.publish(msg)

    def current_q7(self):
        q = list(self.feedback.position[:7])
        if len(q) < 7:
            q.append(self.args.gripper)
        return np.asarray(q, dtype=float)

    def target_q7(self, current):
        gripper = float(current[6]) if self.args.keep_current_gripper else self.args.gripper
        return np.asarray(list(FIXED_MID_CAMERA_Q6) + [gripper], dtype=float)

    def ramp_to_target(self, current, target):
        max_delta = float(np.max(np.abs(target[:6] - current[:6])))
        duration = max(self.args.min_duration, max_delta / self.args.speed_rad_s)
        steps = max(1, int(math.ceil(duration * self.args.rate)))
        rate = rospy.Rate(self.args.rate)
        print(
            "[fix-mid-camera] ramp: max_delta=%.4f rad, duration=%.2fs, steps=%d"
            % (max_delta, duration, steps)
        )
        for i in range(steps):
            if rospy.is_shutdown():
                return
            alpha = float(i + 1) / float(steps)
            q = (1.0 - alpha) * current + alpha * target
            self.publish(q)
            rate.sleep()

    def hold_target(self, target):
        rate = rospy.Rate(self.args.rate)
        print("[fix-mid-camera] holding target. Ctrl-C to stop publishing.")
        while not rospy.is_shutdown():
            self.publish(target)
            rate.sleep()

    def run(self):
        print("[fix-mid-camera] joint_topic: %s" % self.args.joint_topic)
        print("[fix-mid-camera] cmd_topic  : %s" % self.args.cmd_topic)
        print("[fix-mid-camera] target q6  : %s" % FIXED_MID_CAMERA_Q6.tolist())
        print("[fix-mid-camera] waiting for middle-arm feedback...")
        if not self.wait_feedback():
            raise SystemExit(
                "Timed out waiting for %s. Is the middle arm launch running?"
                % self.args.joint_topic
            )

        current = self.current_q7()
        target = self.target_q7(current)
        raw_q6 = ["%.6f" % x for x in current[:6]]
        print("[fix-mid-camera] Raw joint[0:6]: %s" % raw_q6)
        print("[fix-mid-camera] current q7: %s" % current.round(6).tolist())
        print("[fix-mid-camera] target  q7: %s" % target.round(6).tolist())

        if not self.args.no_confirm:
            input("[fix-mid-camera] Clear the workspace, then press ENTER to ramp.")

        self.ramp_to_target(current, target)
        print("[fix-mid-camera] reached fixed middle-camera pose.")
        if self.args.hold:
            self.hold_target(target)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Ramp the middle arm to a fixed camera joint pose and hold it."
    )
    parser.add_argument("--joint_topic", default="/puppet/joint_mid")
    parser.add_argument("--cmd_topic", default="/master/joint_mid")
    parser.add_argument("--rate", type=float, default=30.0)
    parser.add_argument("--speed_rad_s", type=float, default=0.25)
    parser.add_argument("--min_duration", type=float, default=2.0)
    parser.add_argument("--wait_timeout", type=float, default=10.0)
    parser.add_argument("--gripper", type=float, default=0.1)
    parser.add_argument(
        "--keep_current_gripper",
        action="store_true",
        default=True,
        help="Keep current joint6/gripper value when feedback has it. Default: true.",
    )
    parser.add_argument(
        "--set_gripper",
        dest="keep_current_gripper",
        action="store_false",
        help="Use --gripper instead of keeping the feedback joint6 value.",
    )
    parser.add_argument("--no_confirm", action="store_true")
    parser.add_argument(
        "--no_hold",
        dest="hold",
        action="store_false",
        help="Ramp once and exit instead of continuously holding the target.",
    )
    parser.set_defaults(hold=True)
    return parser.parse_args()


def main():
    args = parse_args()
    node = FixedMidCameraPose(args)
    try:
        node.run()
    except KeyboardInterrupt:
        print("\n[fix-mid-camera] stopped.")


if __name__ == "__main__":
    main()
