import os
import sys
import unittest

import numpy as np


sys.path.insert(0, os.path.abspath("teleop"))
import unified_eef_pose as uep  # noqa: E402


def transform(xyz):
    T = np.eye(4)
    T[:3, 3] = np.asarray(xyz, dtype=float)
    return T


class FakeArmUnified:
    def __init__(self, T_unified_from_arm):
        self.T_unified_from_arm = {"mid": T_unified_from_arm}
        self.T_arm_from_unified = {"mid": np.linalg.inv(T_unified_from_arm)}


class FakeFk:
    def __init__(self, T_base_from_ee):
        self.T_base_from_ee = T_base_from_ee

    def base_from_ee(self, _q):
        return self.T_base_from_ee


class UnifiedEefConverterTest(unittest.TestCase):
    def test_eef_pose_chain_is_unified_from_arm_times_fk(self):
        conv = uep.UnifiedEefConverter.__new__(uep.UnifiedEefConverter)
        conv.unified = FakeArmUnified(transform([1.0, 0.0, 0.0]))
        conv.fk = FakeFk(transform([0.0, 2.0, 0.0]))

        actual = conv.eef_pose_in_unified("mid", np.zeros(6))

        expected = transform([1.0, 0.0, 0.0]) @ transform([0.0, 2.0, 0.0])
        np.testing.assert_allclose(actual, expected)

    def test_unified_pose_converts_back_to_arm_base(self):
        conv = uep.UnifiedEefConverter.__new__(uep.UnifiedEefConverter)
        conv.unified = FakeArmUnified(transform([1.0, 0.0, 0.0]))
        T_unified_from_ee = transform([1.0, 2.0, 0.0])

        actual = conv.arm_pose_from_unified("mid", T_unified_from_ee)

        np.testing.assert_allclose(actual, transform([0.0, 2.0, 0.0]))


if __name__ == "__main__":
    unittest.main()
