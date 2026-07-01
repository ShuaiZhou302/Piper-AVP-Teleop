#!/usr/bin/env python3
"""Pinocchio FK helper for the Piper teleop IK end-effector frame."""
import os

import numpy as np
import pinocchio as pin
from tf.transformations import (
    euler_from_matrix,
    quaternion_from_euler,
    quaternion_from_matrix,
)


HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_URDF = os.path.join(HERE, "urdf", "piper_description.urdf")


def package_dirs_for_urdf(urdf_path):
    ap = os.path.abspath(urdf_path)
    package_dirs = []
    marker = os.sep + "piper_description" + os.sep
    i = ap.find(marker)
    if i >= 0:
        package_dirs.append(ap[:i])
    repo_piper_src = os.path.normpath(os.path.join(HERE, "..", "piper_ros", "src"))
    if os.path.isdir(os.path.join(repo_piper_src, "piper_description", "meshes")):
        package_dirs.append(repo_piper_src)
    return package_dirs


class PiperFkModel:
    """Forward kinematics for the same `ee` frame used by teleop IK.

    The Piper driver pose topic reports joint6. Teleop IK uses an operational
    frame named `ee`, which is joint6 rotated -90 deg around local Y. This class
    intentionally returns that IK/control frame.
    """

    def __init__(self, urdf_path=DEFAULT_URDF):
        self.urdf_path = urdf_path
        robot = pin.RobotWrapper.BuildFromURDF(
            urdf_path, package_dirs=package_dirs_for_urdf(urdf_path)
        )
        self.model = robot.buildReducedRobot(
            list_of_joints_to_lock=["joint7", "joint8"],
            reference_configuration=np.array([0.0] * robot.model.nq),
        ).model

        ee_off_quat = quaternion_from_euler(0.0, -np.pi / 2.0, 0.0)
        self.model.addFrame(
            pin.Frame(
                "ee",
                self.model.getJointId("joint6"),
                pin.SE3(
                    pin.Quaternion(
                        ee_off_quat[3],
                        ee_off_quat[0],
                        ee_off_quat[1],
                        ee_off_quat[2],
                    ),
                    np.array([0.0, 0.0, 0.0]),
                ),
                pin.FrameType.OP_FRAME,
            )
        )
        self.data = self.model.createData()
        self.ee_frame_id = self.model.getFrameId("ee")

    def base_from_ee(self, joint_q6):
        q = np.asarray(joint_q6, dtype=float).flatten()
        if q.size < 6:
            raise ValueError(f"need 6 joints, got {q.size}")
        pin.framesForwardKinematics(self.model, self.data, q[:6])
        se3 = self.data.oMf[self.ee_frame_id]
        T = np.eye(4)
        T[:3, :3] = np.asarray(se3.rotation, dtype=float)
        T[:3, 3] = np.asarray(se3.translation, dtype=float).flatten()
        return T


def matrix_to_quat_xyz(T):
    """4x4 -> (x, y, z, qx, qy, qz, qw)."""
    q = quaternion_from_matrix(T)
    t = T[:3, 3]
    return np.array([t[0], t[1], t[2], q[0], q[1], q[2], q[3]], dtype=float)


def matrix_to_rpy_xyz(T):
    """4x4 -> (x, y, z, roll, pitch, yaw)."""
    r, p, y = euler_from_matrix(T)
    t = T[:3, 3]
    return np.array([t[0], t[1], t[2], r, p, y], dtype=float)
