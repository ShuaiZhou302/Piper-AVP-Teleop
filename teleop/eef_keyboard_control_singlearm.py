#!/usr/bin/env python3
# -- coding: UTF-8

import argparse
import os
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
        self.urdf_path = urdf_path
        ap = os.path.abspath(urdf_path)
        i = ap.find(os.sep + 'piper_description' + os.sep)
        _pkg = [ap[:i]] if i >= 0 else []
        # new urdf 可能引用 private 包里没有的 mesh（如 gripper_base.STL），补一条本仓库 piper_ros/src
        _extra = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'piper_ros', 'src'))
        if _extra not in _pkg and os.path.isdir(os.path.join(_extra, 'piper_description', 'meshes')):
            _pkg = _pkg + [_extra]
        self.robot = pin.RobotWrapper.BuildFromURDF(urdf_path, package_dirs=_pkg)
        self.reduced_robot = self.robot.buildReducedRobot(
            list_of_joints_to_lock=["joint7", "joint8"],
            reference_configuration=np.array([0.0] * self.robot.model.nq),
        )

        # 驱动报的 EE frame 与 URDF 的 joint6 frame 之间的静态偏移(已用 frame_visualize.py 验证):
        #   位置:在 joint6 局部系下沿 -X 平移 0.05m
        #   姿态:绕 joint6 局部 Y 轴旋转 -90°
        ee_off_quat = quaternion_from_euler(0.0, -np.pi / 2.0, 0.0)  # x y z w (sxyz)
        self.reduced_robot.model.addFrame(
            pin.Frame(
                "ee",
                self.reduced_robot.model.getJointId("joint6"),
                pin.SE3(
                    pin.Quaternion(ee_off_quat[3], ee_off_quat[0], ee_off_quat[1], ee_off_quat[2]),
                    np.array([0.0, 0.0, 0.0]), # the xyz gap of the gripper
                    # np.array([-0.05, 0.0, 0.0]), # the xyz gap of the camera
                ),
                pin.FrameType.OP_FRAME,
            )
        )

        self.model = self.reduced_robot.model
        # addFrame 之后必须重建 data,否则 data.oMf 长度还是旧值,访问 ee_frame_id 越界
        self.data = self.model.createData()
        self.reduced_robot.data = self.data
        self.default_q = np.zeros(self.model.nq)
        self.init_data = np.zeros(self.model.nq)
        self.history_data = np.zeros(self.model.nq)
        self.geom_model = pin.buildGeomFromUrdf(
            self.robot.model, urdf_path, pin.GeometryType.COLLISION, package_dirs=_pkg
        )
        ngeom = len(self.geom_model.geometryObjects)
        for i in range(4, 10):
            for j in range(0, 3):
                if i < ngeom and j < ngeom and i != j:
                    self.geom_model.addCollisionPair(pin.CollisionPair(i, j))
        self.geometry_data = pin.GeometryData(self.geom_model)

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

    def check_self_collision(self, q, gripper=np.array([0.0, 0.0])):
        q_full = np.concatenate([np.array(q, dtype=float), np.array(gripper, dtype=float)], axis=0)
        pin.forwardKinematics(self.robot.model, self.robot.data, q_full)
        pin.updateGeometryPlacements(self.robot.model, self.robot.data, self.geom_model, self.geometry_data)
        return pin.computeCollisions(self.geom_model, self.geometry_data, False)

    def ik_fun(self, target_pose, gripper=0.0, motorstate=None, motorV=None):
        gripper_vec = np.array([gripper / 2.0, -gripper / 2.0])
        if motorstate is not None:
            self.init_data = np.array(motorstate, dtype=float).copy()
        self.opti.set_initial(self.var_q, self.init_data)
        self.opti.set_value(self.param_tf, target_pose)
        try:
            self.opti.solve_limited()
            sol_q = np.array(self.opti.value(self.var_q)).reshape(-1)
            max_diff = np.max(np.abs(self.history_data - sol_q))
            self.init_data = sol_q.copy()
            if max_diff > (30.0 / 180.0 * np.pi):
                self.init_data = np.zeros(self.model.nq)
            self.history_data = sol_q.copy()
            if motorV is not None:
                v = np.array(motorV, dtype=float) * 0.0
            else:
                v = (sol_q - self.init_data) * 0.0
            tau_ff = pin.rnea(
                self.reduced_robot.model,
                self.reduced_robot.data,
                sol_q,
                v,
                np.zeros(self.reduced_robot.model.nv),
            )
            is_collision = self.check_self_collision(sol_q, gripper_vec)
            return sol_q, tau_ff, (not is_collision)
        except Exception:
            return None, None, False

    def solve(self, xyz, rpy, gripper=0.0, motorstate=None, allow_collision=False):
        q = quaternion_from_euler(rpy[0], rpy[1], rpy[2])
        target = pin.SE3(pin.Quaternion(q[3], q[0], q[1], q[2]), np.array(xyz, dtype=float))
        sol_q, _, get_result = self.ik_fun(target.homogeneous, gripper=gripper, motorstate=motorstate)
        if sol_q is not None and (get_result or allow_collision):
            msg = "ok" if get_result else "ok_allow_collision"
            return sol_q[:6], True, msg
        return None, False, "ik_failed_or_collision"


class Controller:
    def __init__(self, args):
        self.args = args
        self.pose = None
        self.joint = None

        rospy.init_node("eef_pinocchio_teleop_singlearm", anonymous=True)

        rospy.Subscriber(args.pose_topic, PoseStamped, self._pose_cb, queue_size=50)
        rospy.Subscriber(args.joint_topic, JointState, self._joint_cb, queue_size=50)

        self.pub = rospy.Publisher(args.cmd_topic, JointState, queue_size=10)

        self.ik = PinocchioIKSolver(args.urdf)

    def _pose_cb(self, msg):
        self.pose = msg

    def _joint_cb(self, msg):
        self.joint = msg

    def wait_feedback(self, timeout_sec):
        deadline = rospy.Time.now() + rospy.Duration(timeout_sec)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            pose_ready = self.pose is not None
            joint_ready = self.joint is not None and len(self.joint.position) >= 6
            if pose_ready and joint_ready:
                return True
            if rospy.Time.now() > deadline:
                return False
            rate.sleep()
        return False

    def publish_joints(self, six, gripper):
        names = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        m = JointState()
        m.header = Header()
        m.header.stamp = rospy.Time.now()
        m.name = names
        m.position = list(six) + [gripper]
        self.pub.publish(m)


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


def get_args():
    p = argparse.ArgumentParser(description='Single-arm EEF(xyz+rpy)+gripper keyboard teleop via Pinocchio IK')
    p.add_argument('--arm', type=str, default=None,
                   help='Which arm to control: l/m/r or left/mid/right. If omitted, prompt at startup.')

    # 由 --arm 推导;也可手动覆盖
    p.add_argument('--pose_topic',  type=str, default=None)
    p.add_argument('--joint_topic', type=str, default=None)
    p.add_argument('--cmd_topic',   type=str, default=None)

    default_urdf = '/home/agilex/cobot_magic/Piper_ros_private-ros-noetic/src/piper_description/urdf/piper_description_new.urdf'
    # default_urdf = '/home/agilex/cobot_magic/Piper_ros_private-ros-noetic/src/piper_description/urdf/piper_description.urdf'
    p.add_argument('--urdf', type=str, default=default_urdf)

    p.add_argument('--xyz_step', type=float, default=0.002)
    p.add_argument('--rpy_step', type=float, default=0.01)
    p.add_argument('--gripper_step', type=float, default=0.002)
    p.add_argument('--gripper_min', type=float, default=0.0)
    p.add_argument('--gripper_max', type=float, default=0.1)
    p.add_argument('--gripper_init', type=float, default=0.02)
    p.add_argument('--feedback_wait', type=float, default=30.0)
    p.add_argument('--publish_rate', type=float, default=30.0)

    args = p.parse_args()

    # 解析 --arm,根据它填充未显式覆盖的 topic 默认值
    args.arm = resolve_arm(args.arm)
    if args.pose_topic  is None: args.pose_topic  = '/puppet/end_pose_%s' % args.arm
    if args.joint_topic is None: args.joint_topic = '/puppet/joint_%s'    % args.arm
    if args.cmd_topic   is None: args.cmd_topic   = '/master/joint_%s'    % args.arm

    return args


def dim_step(args, dim):
    if dim in [0, 1, 2]:
        return args.xyz_step
    if dim in [3, 4, 5]:
        return args.rpy_step
    return args.gripper_step


def run(controller, args):
    print('Controlling arm: %s' % args.arm.upper())
    print('  pose_topic:  %s' % args.pose_topic)
    print('  joint_topic: %s' % args.joint_topic)
    print('  cmd_topic:   %s' % args.cmd_topic)

    if not controller.wait_feedback(args.feedback_wait):
        rospy.logerr('No %s feedback in %.1f s', args.pose_topic, args.feedback_wait)
        return

    p = controller.pose.pose
    q = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
    rpy_raw = euler_from_quaternion(q)
    print('Raw quat(xyzw):', ['%.6f' % v for v in q], ' -> rpy:', ['%.6f' % v for v in rpy_raw])
    if controller.joint is not None and len(controller.joint.position) >= 6:
        print('Raw joint[0:6]:', ['%.6f' % v for v in controller.joint.position[:6]])

    if controller.joint is not None and len(controller.joint.position) >= 6:
        controller.ik.init_data = np.array(controller.joint.position[:6], dtype=float)
        controller.ik.history_data = controller.ik.init_data.copy()
        _q = np.array(controller.joint.position[:6], dtype=float)
        pin.framesForwardKinematics(controller.ik.model, controller.ik.data, _q)
        _T = controller.ik.data.oMf[controller.ik.ee_frame_id]
        _qq = pin.Quaternion(_T.rotation)
        _rpy = euler_from_quaternion([_qq.x, _qq.y, _qq.z, _qq.w])
        print('FK   xyz:', ['%.6f' % v for v in _T.translation], ' -> rpy:', ['%.6f' % v for v in _rpy])

    # 用 FK 的输出作为初始 target —— FK 已经把 ee 偏移算进去了,
    # 所以 s 里的 (xyz, rpy) 和 IK 内部的 ee frame 是同一个约定(自然 EE,而非驱动的 joint6)。
    g = controller.args.gripper_init
    if controller.joint is not None and len(controller.joint.position) >= 7:
        g = controller.joint.position[6]
    s = [
        _T.translation[0], _T.translation[1], _T.translation[2],
        _rpy[0], _rpy[1], _rpy[2],
        g,
    ]
    print('Initial pose+gripper:', ['%.6f' % v for v in s])
    print('Dim map: [0..6] = xyz rpy g')

    while not rospy.is_shutdown():
        x = input('Select dim 0-6 (q quit): ').strip().lower()
        if x == 'q':
            return
        try:
            dim = int(x)
        except ValueError:
            print('Please input integer 0-6')
            continue
        if 0 <= dim <= 6:
            break
        print('Please input integer 0-6')

    step = dim_step(args, dim)
    period = 1.0 / max(1.0, args.publish_rate)
    poll = min(period, 0.05)

    labels = ['x', 'y', 'z', 'r', 'p', 'y', 'g']

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

            if dim == 6:
                if controller.joint is None:
                    print('\\nSkip publish: no joint feedback yet.')
                    continue
                now6 = list(controller.joint.position)[:6]
                if len(now6) < 6:
                    print('\\nSkip publish: joint feedback length < 6.')
                    continue
                controller.publish_joints(now6, s[6])
                print('\\nPublish gripper only. g=%.4f' % s[6])
                continue

            motor = None
            if controller.joint is not None and len(controller.joint.position) >= 6:
                motor = controller.joint.position[:6]
            six, ok, msg = controller.ik.solve(s[0:3], s[3:6], gripper=s[6], motorstate=motor)

            print('\\nIK:', 'SUCCESS' if ok else 'FAILED', msg)

            if ok:
                controller.publish_joints(six, s[6])
                print('Publish joints ok. g=%.4f' % s[6])
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
