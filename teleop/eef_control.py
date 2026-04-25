#!/usr/bin/env python3
import argparse
import select
import sys
import termios
import tty
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from piper_msgs.msg import PosCmd


class EEFOperator:
    def __init__(self, args):
        self.args = args
        self.left_pose = None
        self.right_pose = None
        self.left_joint = None
        self.pos_pub = None
        self.init_ros()

    def init_ros(self):
        rospy.init_node("eef_keyboard_control", anonymous=True)
        rospy.Subscriber(self.args.left_pose_topic, PoseStamped, self._left_pose_cb, queue_size=100)
        rospy.Subscriber(self.args.right_pose_topic, PoseStamped, self._right_pose_cb, queue_size=100)
        rospy.Subscriber(self.args.left_joint_topic, JointState, self._left_joint_cb, queue_size=100)
        self.pos_pub = rospy.Publisher(self.args.pos_cmd_topic, PosCmd, queue_size=10)

    def _left_pose_cb(self, msg):
        self.left_pose = msg

    def _right_pose_cb(self, msg):
        self.right_pose = msg

    def _left_joint_cb(self, msg):
        self.left_joint = msg

    def wait_feedback(self, timeout_sec):
        deadline = rospy.Time.now() + rospy.Duration(timeout_sec)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.left_pose is not None and self.right_pose is not None:
                return True
            if rospy.Time.now() > deadline:
                return False
            rate.sleep()
        return False

    def initial_cmd(self):
        p = self.left_pose.pose
        cmd = [p.position.x, p.position.y, p.position.z, 0.0, 0.0, 0.0, self.args.initial_gripper]
        if self.left_joint is not None and len(self.left_joint.position) >= 7:
            cmd[6] = self.left_joint.position[6]
        return cmd

    def publish_cmd(self, cmd):
        msg = PosCmd()
        msg.x, msg.y, msg.z = cmd[0], cmd[1], cmd[2]
        msg.roll, msg.pitch, msg.yaw = cmd[3], cmd[4], cmd[5]
        msg.gripper = min(max(cmd[6], self.args.gripper_min), self.args.gripper_max)
        msg.mode1 = self.args.mode1
        msg.mode2 = self.args.mode2
        self.pos_pub.publish(msg)
        cmd[6] = msg.gripper


def parse_args():
    p = argparse.ArgumentParser(description="Keyboard EEF PosCmd publisher.")
    p.add_argument("--left_pose_topic", type=str, default="/puppet/end_pose_left")
    p.add_argument("--right_pose_topic", type=str, default="/puppet/end_pose_right")
    p.add_argument("--left_joint_topic", type=str, default="/puppet/joint_left")
    p.add_argument("--pos_cmd_topic", type=str, default="/pos_cmd")
    p.add_argument("--mode1", type=int, default=1)
    p.add_argument("--mode2", type=int, default=0)
    p.add_argument("--xyz_step", type=float, default=0.002)
    p.add_argument("--rpy_step", type=float, default=0.02)
    p.add_argument("--gripper_step", type=float, default=0.002)
    p.add_argument("--gripper_min", type=float, default=0.0)
    p.add_argument("--gripper_max", type=float, default=0.1)
    p.add_argument("--initial_gripper", type=float, default=0.02)
    p.add_argument("--feedback_wait", type=float, default=30.0)
    p.add_argument("--publish_rate", type=float, default=30.0)
    return p.parse_args()


def step_for(args, dim):
    if dim <= 2:
        return args.xyz_step
    if dim <= 5:
        return args.rpy_step
    return args.gripper_step


def main():
    args = parse_args()
    op = EEFOperator(args)
    rospy.loginfo("eef_control started. pos_cmd=%s mode=(%d,%d)", args.pos_cmd_topic, args.mode1, args.mode2)
    if not op.wait_feedback(args.feedback_wait):
        rospy.logerr("No end pose feedback received within %.1f s.", args.feedback_wait)
        return

    print("Initial left xyz:", [op.left_pose.pose.position.x, op.left_pose.pose.position.y, op.left_pose.pose.position.z])
    print("Initial right xyz:", [op.right_pose.pose.position.x, op.right_pose.pose.position.y, op.right_pose.pose.position.z])

    while not rospy.is_shutdown():
        s = input("Select dim 0-6 (q quit): ").strip().lower()
        if s == "q":
            return
        try:
            dim = int(s)
            if 0 <= dim <= 6:
                break
        except ValueError:
            pass
        print("Please input 0-6.")

    cmd = op.initial_cmd()
    st = step_for(args, dim)
    period = 1.0 / max(1.0, args.publish_rate)
    poll = min(period, 0.05)
    labels = ["x", "y", "z", "roll", "pitch", "yaw", "gripper"]

    def read_key():
        if select.select([sys.stdin], [], [], poll)[0]:
            c = sys.stdin.read(1)
            if c in ("\x03", "\x04"):
                return "q"
            return c.lower()
        return None

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    print("Single-key: w/s adjust, q quit.")
    try:
        while not rospy.is_shutdown():
            sys.stdout.write("\rmode=(%d,%d) %s=%.6f cmd=%s   " % (args.mode1, args.mode2, labels[dim], cmd[dim], ["%.4f" % v for v in cmd]))
            sys.stdout.flush()
            ch = read_key()
            if ch == "q":
                break
            if ch == "w":
                cmd[dim] += st
                op.publish_cmd(cmd)
            elif ch == "s":
                cmd[dim] -= st
                op.publish_cmd(cmd)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    print()


if __name__ == "__main__":
    main()
