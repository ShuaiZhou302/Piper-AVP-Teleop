import os
import tempfile
import unittest

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


if __name__ == "__main__":
    unittest.main()
