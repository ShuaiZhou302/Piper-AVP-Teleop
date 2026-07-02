import os
import tempfile
import unittest

import h5py
import numpy as np

from data_collect import postprocess_camera_poses as post


class PostprocessCameraPosesTest(unittest.TestCase):
    def test_find_hdf5_files_accepts_files_and_dirs_sorted_by_episode_number(self):
        with tempfile.TemporaryDirectory() as d:
            paths = [
                os.path.join(d, "episode_10.hdf5"),
                os.path.join(d, "episode_2.hdf5"),
                os.path.join(d, "ignore.txt"),
            ]
            for path in paths:
                with open(path, "w", encoding="utf-8") as f:
                    f.write("")

            found = post.find_hdf5_files([paths[0], d])

        self.assertEqual(
            [os.path.basename(p) for p in found],
            ["episode_2.hdf5", "episode_10.hdf5"],
        )

    def test_postprocess_writes_camera_eef_and_fov_fields(self):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "episode_0.hdf5")
            with h5py.File(path, "w") as f:
                obs = f.create_group("observations")
                obs.create_dataset("qpos", data=np.zeros((2, 21), dtype=float))
                ee_q = obs.create_group("ee_pose_quat")
                ee_r = obs.create_group("ee_pose_rpy")
                for arm in ("left", "right", "mid"):
                    ee_q.create_dataset(arm, data=np.zeros((2, 7), dtype=float))
                    ee_r.create_dataset(arm, data=np.zeros((2, 6), dtype=float))
                f.create_dataset("action", data=np.zeros((2, 21), dtype=float))
                ci = f.create_group("camera_info")
                for cam in ("cam_front", "cam_left", "cam_right"):
                    g = ci.create_group(cam)
                    g.create_dataset(
                        "K",
                        data=np.array([500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0]),
                    )
                    g.attrs["width"] = 640
                    g.attrs["height"] = 480

            frames, _summary = post.postprocess_file(path)

            self.assertEqual(frames, 2)
            with h5py.File(path, "r") as f:
                self.assertEqual(
                    f["observations/camera_pose_in_unified/cam_front/matrix"].shape,
                    (2, 4, 4),
                )
                self.assertEqual(
                    f["observations/ee_pose_in_unified/left/matrix"].shape,
                    (2, 4, 4),
                )
                self.assertEqual(
                    f["observations/ee_pose_in_unified"].attrs["eef_frame"],
                    "teleop_ik_ee",
                )
                self.assertEqual(
                    f["observations/ee_pose_in_unified/left/quat"].attrs[
                        "quaternion_order"
                    ],
                    "xyzw",
                )
                self.assertEqual(
                    f["observations/ee_pose_in_unified/left/quat"].attrs["columns"],
                    "x,y,z,qx,qy,qz,qw",
                )
                self.assertEqual(
                    f["observations/camera_pose_in_unified/cam_front/quat"].attrs[
                        "quaternion_order"
                    ],
                    "xyzw",
                )
                self.assertIn("horizontal_fov_rad", f["camera_info/cam_front"].attrs)
                self.assertEqual(
                    f["observations/ee_pose_quat"].attrs["frame"],
                    "raw_driver_joint6",
                )
                self.assertIn(
                    "Do not compare directly",
                    f["observations/ee_pose_quat"].attrs["warning"],
                )
                self.assertEqual(
                    f["observations/ee_pose_quat/left"].attrs["preferred_aligned_dataset"],
                    "observations/ee_pose_in_unified/left/quat",
                )
                self.assertEqual(
                    f["observations/ee_pose_quat/left"].attrs["quaternion_order"],
                    "xyzw",
                )


if __name__ == "__main__":
    unittest.main()
