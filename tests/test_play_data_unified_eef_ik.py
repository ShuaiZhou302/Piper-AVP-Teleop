import unittest

import numpy as np

from data_collect import play_data_unified_eef_ik as playback


def transform(xyz):
    T = np.eye(4)
    T[:3, 3] = np.asarray(xyz, dtype=float)
    return T


class FakeConverter:
    def arm_pose_from_unified(self, arm, T_unified):
        self.last_arm = arm
        return transform([-1.0, 0.0, 0.0]) @ T_unified


class UnifiedEefPlaybackTest(unittest.TestCase):
    def test_unified_matrix_is_converted_to_arm_ik_pose(self):
        matrices = {"left": np.asarray([transform([1.0, 2.0, 3.0])])}

        poses = playback.unified_matrices_to_arm_rpy_poses(matrices, FakeConverter())

        self.assertEqual(poses["left"].shape, (1, 6))
        np.testing.assert_allclose(poses["left"][0, :3], np.array([0.0, 2.0, 3.0]))


if __name__ == "__main__":
    unittest.main()
