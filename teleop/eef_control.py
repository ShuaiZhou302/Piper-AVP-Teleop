#!/usr/bin/env python3
# -- coding: UTF-8

import argparse
import select
import sys
import termios
import tty

import casadi
import numpy as np
import rospy
import pinocchio as pin
from pinocchio import casadi as cpin
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


class PinocchioIKSolver:
    def __init__(self, urdf_path):
        self.robot = pin.RobotWrapper.BuildFromURDF(urdf_path)
        self.reduced_robot = self.robot.buildReducedRobot(
            list_of_joints_to_lock=["joint7", "joint8"],
            reference_configuration=np.array([0.0] * self.robot.model.nq),
        )

        # Keep the same end-effector convention as official piper_pinocchio.py
        q_identity = np.array([0.0, 0.0, 0.0, 1.0])  # x y z w
        self.reduced_robot.model.addFrame(
            pin.Frame(
                "ee",
                self.reduced_robot.model.getJointId("joint6"),
                pin.SE3(
                    pin.Quaternion(q_identity[3], q_identity[0], q_identity[1], q_identity[2]),
                    np.array([0.0, 0.0, 0.0]),
                ),
                pin.FrameType.OP_FRAME,
            )
        )

        self.model = self.reduced_robot.model
        self.data = self.reduced_robot.data
        self.default_q = np.zeros(self.model.nq)
        self.init_data = np.zeros(self.model.nq)
        self.history_data = np.zeros(self.model.nq)

        self.cmodel = cpin.Model(self.model)
        self.cdata = self.cmodel.createData()
        self.cq = casadi.SX.sym("q", self.model.nq, 1)
        self.cTf = casadi.SX.sym("tf", 4, 4)
        cpin.framesForwardKinematics(self.cmodel, self.cdata, self.cq)

        self.ee_frame_id = self.model.getFrameId("ee")
        self.error_func = casadi.Function(
            "error",
            [self.cq, self.cTf],
            [
                casadi.vertcat(
                    cpin.log6(self.cdata.oMf[self.ee_frame_id].inverse() * cpin.SE3(self.cTf)).vector,
                )
            ],
        )

        self.opti = casadi.Opti()
        self.var_q = self.opti.variable(self.model.nq)
        self.param_tf = self.opti.parameter(4, 4)

        err = self.error_func(self.var_q, self.param_tf)
        pos_err = err[:3]
        ori_err = err[3:]
        total_cost = casadi.sumsqr(1.0 * pos_err) + casadi.sumsqr(0.1 * ori_err)
        reg_cost = casadi.sumsqr(self.var_q)

        self.opti.subject_to(
            self.opti.bounded(self.model.lowerPositionLimit, self.var_q, self.model.upperPositionLimit)
        )
        self.opti.minimize(20.0 * total_cost + 0.01 * reg_cost)
        self.opti.solver(
            "ipopt",
            {
                "ipopt": {"print_level": 0, "max_iter": 50, "tol": 1e-4},
                "print_time": False,
            },
        )

    def solve(self, xyz, rpy):
        q = quaternion_from_euler(rpy[0], rpy[1], rpy[2])
        target = pin.SE3(pin.Quaternion(q[3], q[0], q[1], q[2]), np.array(xyz, dtype=float))

        self.opti.set_initial(self.var_q, self.init_data)
        self.opti.set_value(self.param_tf, target.homogeneous)

        try:
            self.opti.solve_limited()
            sol_q = np.array(self.opti.value(self.var_q)).reshape(-1)
            max_diff = np.max(np.abs(self.history_data - sol_q))
            self.init_data = sol_q.copy()
            if max_diff > (30.0 / 180.0 * np.pi):
                self.init_data = np.zeros(self.model.nq)
            self.history_data = sol_q.copy()
            return sol_q[:6], True, "ok"
        except Exception:
            return None, False, "ipopt_failed_or_max_iter"


class Controller:
    def __init__(self, args):
        self.args = args
        self.left_pose = None
        self.right_pose = None
        self.left_joint = None
        self.right_joint = None

        rospy.init_node("eef_pinocchio_teleop", anonymous=True)

        rospy.Subscriber(args.left_pose_topic, PoseStamped, self._left_pose_cb, queue_size=50)
        rospy.Subscriber(args.right_pose_topic, PoseStamped, self._right_pose_cb, queue_size=50)
        rospy.Subscriber(args.left_joint_topic, JointState, self._left_joint_cb, queue_size=50)
        rospy.Subscriber(args.right_joint_topic, JointState, self._right_joint_cb, queue_size=50)

        self.left_pub = rospy.Publisher(args.left_cmd_topic, JointState, queue_size=10)
        self.right_pub = rospy.Publisher(args.right_cmd_topic, JointState, queue_size=10)

        self.left_ik = PinocchioIKSolver(args.left_urdf)
        self.right_ik = PinocchioIKSolver(args.right_urdf)

    def _left_pose_cb(self, msg):
        self.left_pose = msg

    def _right_pose_cb(self, msg):
        self.right_pose = msg

    def _left_joint_cb(self, msg):
        self.left_joint = msg

    def _right_joint_cb(self, msg):
        self.right_joint = msg

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

    def initial_state14(self):
        lp = self.left_pose.pose
        rp = self.right_pose.pose
        l_rpy = euler_from_quaternion([lp.orientation.x, lp.orientation.y, lp.orientation.z, lp.orientation.w])
        r_rpy = euler_from_quaternion([rp.orientation.x, rp.orientation.y, rp.orientation.z, rp.orientation.w])
        lg = self.args.gripper_init
        rg = self.args.gripper_init
        if self.left_joint is not None and len(self.left_joint.position) >= 7:
            lg = self.left_joint.position[6]
        if self.right_joint is not None and len(self.right_joint.position) >= 7:
            rg = self.right_joint.position[6]
        return [
            lp.position.x, lp.position.y, lp.position.z,
            l_rpy[0], l_rpy[1], l_rpy[2],
            lg,
            rp.position.x, rp.position.y, rp.position.z,
            r_rpy[0], r_rpy[1], r_rpy[2],
            rg,
        ]

    def publish_joints(self, left6, right6, left_g, right_g):
        names = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

        m1 = JointState()
        m1.header = Header()
        m1.header.stamp = rospy.Time.now()
        m1.name = names
        m1.position = list(left6) + [left_g]

        m2 = JointState()
        m2.header = Header()
        m2.header.stamp = rospy.Time.now()
        m2.name = names
        m2.position = list(right6) + [right_g]

        self.left_pub.publish(m1)
        self.right_pub.publish(m2)


def get_args():
    p = argparse.ArgumentParser(description='Dual-arm EEF(xyz+rpy)+gripper keyboard teleop via Pinocchio IK')
    p.add_argument('--left_pose_topic', type=str, default='/puppet/end_pose_left')
    p.add_argument('--right_pose_topic', type=str, default='/puppet/end_pose_right')
    p.add_argument('--left_joint_topic', type=str, default='/puppet/joint_left')
    p.add_argument('--right_joint_topic', type=str, default='/puppet/joint_right')
    p.add_argument('--left_cmd_topic', type=str, default='/master/joint_left')
    p.add_argument('--right_cmd_topic', type=str, default='/master/joint_right')

    default_urdf = '/home/agilex/cobot_magic/Piper_ros_private-ros-noetic/src/piper_description/urdf/piper_description.urdf'
    p.add_argument('--left_urdf', type=str, default=default_urdf)
    p.add_argument('--right_urdf', type=str, default=default_urdf)

    p.add_argument('--xyz_step', type=float, default=0.002)
    p.add_argument('--rpy_step', type=float, default=0.01)
    p.add_argument('--gripper_step', type=float, default=0.002)
    p.add_argument('--gripper_min', type=float, default=0.0)
    p.add_argument('--gripper_max', type=float, default=0.1)
    p.add_argument('--gripper_init', type=float, default=0.02)
    p.add_argument('--feedback_wait', type=float, default=30.0)
    p.add_argument('--publish_rate', type=float, default=30.0)
    return p.parse_args()


def dim_step(args, dim):
    if dim in [0, 1, 2, 7, 8, 9]:
        return args.xyz_step
    if dim in [3, 4, 5, 10, 11, 12]:
        return args.rpy_step
    return args.gripper_step


def run(controller, args):
    if not controller.wait_feedback(args.feedback_wait):
        rospy.logerr('No /puppet/end_pose feedback in %.1f s', args.feedback_wait)
        return

    if controller.left_joint is not None and len(controller.left_joint.position) >= 6:
        controller.left_ik.init_data = np.array(controller.left_joint.position[:6], dtype=float)
        controller.left_ik.history_data = controller.left_ik.init_data.copy()
    if controller.right_joint is not None and len(controller.right_joint.position) >= 6:
        controller.right_ik.init_data = np.array(controller.right_joint.position[:6], dtype=float)
        controller.right_ik.history_data = controller.right_ik.init_data.copy()

    s = controller.initial_state14()
    print('Initial LEFT pose+gripper:', ['%.6f' % v for v in s[:7]])
    print('Initial RIGHT pose+gripper:', ['%.6f' % v for v in s[7:14]])
    print('Dim map: left[0..6]=xyz rpy g, right[7..13]=xyz rpy g')

    while not rospy.is_shutdown():
        x = input('Select dim 0-13 (q quit): ').strip().lower()
        if x == 'q':
            return
        try:
            dim = int(x)
        except ValueError:
            print('Please input integer 0-13')
            continue
        if 0 <= dim <= 13:
            break
        print('Please input integer 0-13')

    step = dim_step(args, dim)
    period = 1.0 / max(1.0, args.publish_rate)
    poll = min(period, 0.05)

    labels = [
        'L.x', 'L.y', 'L.z', 'L.r', 'L.p', 'L.y', 'L.g',
        'R.x', 'R.y', 'R.z', 'R.r', 'R.p', 'R.y', 'R.g',
    ]

    def read_key():
        if select.select([sys.stdin], [], [], poll)[0]:
            c = sys.stdin.read(1)
            if c in ('\x03', '\x04'):
                return 'q'
            return c.lower()
        return None

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    print('Single-key: w/s adjust current dim, q quit.')
    print('Pose input uses xyz + rpy (same as official piper_pinocchio).')

    try:
        while not rospy.is_shutdown():
            sys.stdout.write('\\r%s=%.6f step=%.6f   ' % (labels[dim], s[dim], step))
            sys.stdout.flush()

            ch = read_key()
            if ch == 'q':
                break
            if ch not in ('w', 's'):
                continue

            if ch == 'w':
                s[dim] += step
            else:
                s[dim] -= step

            s[6] = min(max(s[6], args.gripper_min), args.gripper_max)
            s[13] = min(max(s[13], args.gripper_min), args.gripper_max)

            if dim in (6, 13):
                if controller.left_joint is None or controller.right_joint is None:
                    print('\\nSkip publish: no joint feedback yet.')
                    continue
                left_now = list(controller.left_joint.position)[:6]
                right_now = list(controller.right_joint.position)[:6]
                if len(left_now) < 6 or len(right_now) < 6:
                    print('\\nSkip publish: joint feedback length < 6.')
                    continue
                controller.publish_joints(left_now, right_now, s[6], s[13])
                print('\\nPublish gripper only. left_g=%.4f right_g=%.4f' % (s[6], s[13]))
                continue

            l6, lok, lmsg = controller.left_ik.solve(s[0:3], s[3:6])
            r6, rok, rmsg = controller.right_ik.solve(s[7:10], s[10:13])

            print('\\nLEFT IK:', 'SUCCESS' if lok else 'FAILED', lmsg)
            print('RIGHT IK:', 'SUCCESS' if rok else 'FAILED', rmsg)

            if lok and rok:
                controller.publish_joints(l6, r6, s[6], s[13])
                print('Publish joints ok. left_g=%.4f right_g=%.4f' % (s[6], s[13]))
            else:
                print('Skip publish due to IK failure.')

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    print('')


def main():
    args = get_args()
    c = Controller(args)
    run(c, args)


if __name__ == '__main__':
    main()
