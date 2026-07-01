import unittest

import numpy as np

from data_collect import play_data_eef_ik as playback


class PlayDataEefIkHelpersTest(unittest.TestCase):
    def test_gripper_targets_come_from_each_arm_seventh_action(self):
        action = np.arange(21, dtype=float)

        grippers = playback.extract_grippers(action)

        self.assertEqual(grippers, {"left": 6.0, "right": 13.0, "mid": 20.0})

    def test_clip_joint_step_limits_each_joint_independently(self):
        previous = np.array([0.0, 1.0, -1.0, 0.2, -0.2, 0.0])
        target = np.array([0.2, 0.8, -0.95, -0.1, 0.3, 0.01])

        clipped = playback.clip_joint_step(target, previous, max_step=0.05)

        np.testing.assert_allclose(
            clipped,
            np.array([0.05, 0.95, -0.95, 0.15, -0.15, 0.01]),
        )

    def test_failed_ik_holds_previous_command_with_new_gripper(self):
        previous = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7])

        held = playback.command_from_ik_result(
            sol6=None,
            ok=False,
            gripper=0.9,
            previous_command=previous,
            max_joint_step=0.05,
        )

        np.testing.assert_allclose(held, np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.9]))

    def test_joint6_pose_is_converted_to_teleop_ik_ee_frame(self):
        pose = np.array([0.1, 0.2, 0.3, 0.0, 0.0, 0.0])

        converted = playback.joint6_pose_to_ik_ee_pose(pose)

        np.testing.assert_allclose(converted[:3], pose[:3])
        np.testing.assert_allclose(converted[3:], np.array([0.0, -np.pi / 2.0, 0.0]), atol=1e-7)


if __name__ == "__main__":
    unittest.main()
