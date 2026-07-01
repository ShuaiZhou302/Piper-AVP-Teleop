import os
import sys
import unittest

import numpy as np


sys.path.insert(0, os.path.abspath("teleop"))
import arm_camera_in_unified as acu  # noqa: E402


def transform(xyz, rpy=(0.0, 0.0, 0.0)):
    roll, pitch, yaw = rpy
    Rx = np.array([
        [1.0, 0.0, 0.0],
        [0.0, np.cos(roll), -np.sin(roll)],
        [0.0, np.sin(roll), np.cos(roll)],
    ])
    Ry = np.array([
        [np.cos(pitch), 0.0, np.sin(pitch)],
        [0.0, 1.0, 0.0],
        [-np.sin(pitch), 0.0, np.cos(pitch)],
    ])
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0.0],
        [np.sin(yaw), np.cos(yaw), 0.0],
        [0.0, 0.0, 1.0],
    ])
    T = np.eye(4)
    T[:3, :3] = Rz @ Ry @ Rx
    T[:3, 3] = np.asarray(xyz, dtype=float)
    return T


class FakeUnified:
    def __init__(self, T_unified_from_arm):
        self.T_unified_from_arm = {"mid": T_unified_from_arm}


class WristCameraUnifiedConverterTest(unittest.TestCase):
    def test_camera_pose_chain_applies_fk_before_handeye(self):
        conv = acu.WristCameraUnifiedConverter.__new__(acu.WristCameraUnifiedConverter)
        T_unified_from_arm = transform([1.0, 0.0, 0.0])
        T_wrist_from_camera = transform([0.0, 0.0, 3.0], rpy=(0.0, 0.0, 0.7))
        conv.unified = FakeUnified(T_unified_from_arm)
        conv.T_wrist_from_camera = {"mid": T_wrist_from_camera}
        conv._unified_X = {"mid": T_unified_from_arm @ T_wrist_from_camera}
        conv.fk_base_from_wrist = lambda _q: transform([0.0, 2.0, 0.0], rpy=(0.2, 0.0, 0.0))

        actual = conv.camera_pose_in_unified("mid", np.zeros(6))

        expected = T_unified_from_arm @ conv.fk_base_from_wrist(None) @ T_wrist_from_camera
        np.testing.assert_allclose(actual, expected)


if __name__ == "__main__":
    unittest.main()
