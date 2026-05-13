#!/usr/bin/env python3
# -- coding: UTF-8
"""单臂 joint 键盘步进测试(从 joint_keyboard_control.py 派生的单臂版)。

跟 eef_keyboard_control_singlearm.py 同样的模式:
- 启动时用 --arm 选 l/m/r,或交互式提示
- 控制选中的那一个臂(其他臂不发命令,自动停在原地)
- dim 0-6 = joint0..joint5 + gripper
- w/s 加减,q 退出
"""

import argparse
import select
import sys
import termios
import tty
from collections import deque

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import JointState


ARM_CHOICES = {
    'l': 'left', 'left': 'left',
    'm': 'mid',  'mid':  'mid',
    'r': 'right','right':'right',
}


def resolve_arm(arm_arg):
    """规范化 --arm 输入,缺省则提示用户输入。"""
    if arm_arg is None:
        while True:
            x = input('Select arm (l=left / m=mid / r=right): ').strip().lower()
            if x in ARM_CHOICES:
                return ARM_CHOICES[x]
            print('Please input l / m / r (or left / mid / right)')
    key = arm_arg.strip().lower()
    if key not in ARM_CHOICES:
        raise SystemExit('Invalid --arm value: %s. Expected one of l/m/r/left/mid/right' % arm_arg)
    return ARM_CHOICES[key]


class RosOperator:
    def __init__(self, args):
        self.args = args
        self.puppet_arm_deque = deque()
        rospy.init_node('joint_state_publisher_singlearm', anonymous=True)
        rospy.Subscriber(self.args.puppet_arm_topic, JointState,
                         self.puppet_arm_callback, queue_size=1000, tcp_nodelay=True)
        self.puppet_arm_publisher = rospy.Publisher(self.args.puppet_arm_cmd_topic,
                                                    JointState, queue_size=10)

    def puppet_arm_callback(self, msg):
        if len(self.puppet_arm_deque) >= 2000:
            self.puppet_arm_deque.popleft()
        self.puppet_arm_deque.append(msg)

    def puppet_arm_publish(self, seven):
        joint_state_msg = JointState()
        joint_state_msg.header = Header()
        joint_state_msg.header.stamp = rospy.Time.now()
        joint_state_msg.name = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        joint_state_msg.position = seven
        self.puppet_arm_publisher.publish(joint_state_msg)


def get_arguments():
    parser = argparse.ArgumentParser(description='Single-arm joint keyboard step test.')
    parser.add_argument('--arm', type=str, default=None,
                        help='Which arm to control: l/m/r or left/mid/right. If omitted, prompt at startup.')

    # 由 --arm 推导;也可手动覆盖
    parser.add_argument('--puppet_arm_topic',     type=str, default=None)
    parser.add_argument('--puppet_arm_cmd_topic', type=str, default=None)

    parser.add_argument('--publish_rate', action='store', type=int, default=40)
    parser.add_argument('--arm_steps_length', action='store', type=float, nargs='+',
                        default=[0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.005])

    args = parser.parse_args()

    args.arm = resolve_arm(args.arm)
    if args.puppet_arm_topic     is None: args.puppet_arm_topic     = '/puppet/joint_%s'  % args.arm
    if args.puppet_arm_cmd_topic is None: args.puppet_arm_cmd_topic = '/master/joint_%s'  % args.arm

    return args


def run_interactive_joint_test(ros_operator, args):
    """7 dims: 0-5 = joint0..joint5, 6 = gripper."""
    print('Controlling arm: %s' % args.arm.upper())
    print('  puppet_arm_topic:     %s' % args.puppet_arm_topic)
    print('  puppet_arm_cmd_topic: %s' % args.puppet_arm_cmd_topic)

    deadline = rospy.Time.now() + rospy.Duration(60.0)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if ros_operator.puppet_arm_deque:
            break
        if rospy.Time.now() > deadline:
            rospy.logerr('No puppet joint state received within 60s')
            return
        rate.sleep()

    def snap():
        q = list(ros_operator.puppet_arm_deque[-1].position)[:7]
        q += [0.0] * (7 - len(q))
        return q

    q0 = snap()
    print('Initial:', q0)

    while not rospy.is_shutdown():
        s = input('Dimension 0-6 (q to quit): ').strip().lower()
        if s == 'q':
            return
        try:
            dim = int(s)
        except ValueError:
            print('Please enter an integer in 0-6')
            continue
        if dim < 0 or dim > 6:
            print('Please enter an integer in 0-6')
            continue
        break

    cmd = list(q0)
    period = 1.0 / max(1, args.publish_rate)
    poll = min(period, 0.05)

    def read_key():
        if select.select([sys.stdin], [], [], poll)[0]:
            c = sys.stdin.read(1)
            if c in ('\x03', '\x04'):
                return 'q'
            return c.lower()
        return None

    if not sys.stdin.isatty():
        rospy.logwarn('stdin is not a TTY; w/s requires Enter')
        while not rospy.is_shutdown():
            line = input('w+/s-/q: ').strip().lower()
            if not line:
                continue
            c = line[0]
            if c == 'q':
                break
            st = args.arm_steps_length[dim]
            if c == 'w':
                cmd[dim] += st
            elif c == 's':
                cmd[dim] -= st
            else:
                continue
            ros_operator.puppet_arm_publish(cmd)
            print('Dim', dim, '=', cmd[dim], '|', cmd)
        return

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    print('Single-key control: w/s. Quit with q or Ctrl+C / Ctrl+D')
    try:
        while not rospy.is_shutdown():
            sys.stdout.write(
                '\rDim%d=%.4f | %s   ' % (
                    dim, cmd[dim],
                    '[' + ', '.join('%.3f' % x for x in cmd) + ']'))
            sys.stdout.flush()
            ch = read_key()
            if ch == 'q':
                break
            if ch in ('w', 's'):
                st = args.arm_steps_length[dim]
                if ch == 'w':
                    cmd[dim] += st
                else:
                    cmd[dim] -= st
                ros_operator.puppet_arm_publish(cmd)
    finally:
        try:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        except (termios.error, OSError):
            pass
    print()


def main():
    args = get_arguments()
    ros_operator = RosOperator(args)
    rospy.loginfo('teleop: single-arm joint step test')
    try:
        run_interactive_joint_test(ros_operator, args)
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
