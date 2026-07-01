import unittest

import numpy as np

from data_collect import collect_data_3arm as collect


class CameraFovTest(unittest.TestCase):
    def test_camera_fov_from_intrinsics_uses_width_height_and_focal_lengths(self):
        K = np.array([500.0, 0.0, 320.0, 0.0, 250.0, 240.0, 0.0, 0.0, 1.0])

        hfov, vfov = collect.camera_fov_from_intrinsics(K, width=640, height=480)

        self.assertAlmostEqual(hfov, 2.0 * np.arctan(640.0 / (2.0 * 500.0)))
        self.assertAlmostEqual(vfov, 2.0 * np.arctan(480.0 / (2.0 * 250.0)))


if __name__ == "__main__":
    unittest.main()
