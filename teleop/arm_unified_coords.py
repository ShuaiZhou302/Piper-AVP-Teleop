#!/usr/bin/env python3
"""
Unified coordinate utilities for the 3-arm Piper setup.

Frame convention:
  - Calibration JSON stores T_mid_from_arm for each arm base.
  - The final unified frame has the same orientation as mid_base.
  - The unified origin is 0.25 m lower than mid_base along mid Z.

Therefore:
  p_unified = p_mid + [0, 0, 0.25]
"""
import json
import os

import numpy as np
from tf.transformations import euler_from_matrix, euler_matrix


ARM_ORDER = ("mid", "left", "right")
DEFAULT_EXTRINSICS_JSON = (
    "/home/agilex/cobot_magic/aloha-devel/Piper-AVP-Teleop/teleop/"
    "calibration_outputs/arm_extrinsics_touch_20260629_131357.json"
)

# Unified origin expressed in mid_base coordinates. Negative z means the new
# origin sits below mid_base; point coordinates gain +0.25 in unified Z.
DEFAULT_UNIFIED_ORIGIN_IN_MID = np.array([0.0, 0.0, -0.25], dtype=float)


def make_transform(xyz, rpy=(0.0, 0.0, 0.0)):
    T = euler_matrix(rpy[0], rpy[1], rpy[2])
    T[:3, 3] = np.asarray(xyz, dtype=float)
    return T


def load_mid_from_arm_transforms(extrinsics_json=DEFAULT_EXTRINSICS_JSON):
    path = os.path.abspath(os.path.expanduser(extrinsics_json))
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    transforms = {}
    for arm in ARM_ORDER:
        transforms[arm] = np.asarray(data["transforms"][arm]["matrix"], dtype=float)
    return transforms, path


class ArmUnifiedConverter:
    """Convert points/poses between arm base frames and final unified frame."""

    def __init__(
        self,
        extrinsics_json=DEFAULT_EXTRINSICS_JSON,
        unified_origin_in_mid=DEFAULT_UNIFIED_ORIGIN_IN_MID,
    ):
        self.T_mid_from_arm, self.extrinsics_json = load_mid_from_arm_transforms(
            extrinsics_json
        )
        self.unified_origin_in_mid = np.asarray(unified_origin_in_mid, dtype=float)

        self.T_unified_from_mid = np.eye(4)
        self.T_unified_from_mid[:3, 3] = -self.unified_origin_in_mid
        self.T_mid_from_unified = np.linalg.inv(self.T_unified_from_mid)

        self.T_unified_from_arm = {
            arm: self.T_unified_from_mid @ self.T_mid_from_arm[arm]
            for arm in ARM_ORDER
        }
        self.T_arm_from_unified = {
            arm: np.linalg.inv(self.T_unified_from_arm[arm])
            for arm in ARM_ORDER
        }

    def arm_xyz_rpy_from_unified(self, arm, xyz_unified, rpy_unified=None):
        """Return target xyz/rpy expressed in the requested arm base frame."""
        if arm not in ARM_ORDER:
            raise ValueError(f"unknown arm: {arm}")
        if rpy_unified is None:
            xyz_h = np.ones(4)
            xyz_h[:3] = np.asarray(xyz_unified, dtype=float)
            xyz_arm = (self.T_arm_from_unified[arm] @ xyz_h)[:3]
            return xyz_arm, None

        T_unified_target = make_transform(xyz_unified, rpy_unified)
        T_arm_target = self.T_arm_from_unified[arm] @ T_unified_target
        xyz_arm = T_arm_target[:3, 3]
        rpy_arm = np.asarray(euler_from_matrix(T_arm_target), dtype=float)
        return xyz_arm, rpy_arm

    def unified_xyz_from_arm(self, arm, xyz_arm):
        """Return a point from an arm base frame expressed in unified frame."""
        if arm not in ARM_ORDER:
            raise ValueError(f"unknown arm: {arm}")
        xyz_h = np.ones(4)
        xyz_h[:3] = np.asarray(xyz_arm, dtype=float)
        return (self.T_unified_from_arm[arm] @ xyz_h)[:3]

    def summary_lines(self):
        lines = [
            f"extrinsics_json: {self.extrinsics_json}",
            "unified_frame: same orientation as mid_base",
            (
                "unified_origin_in_mid: "
                f"{self.unified_origin_in_mid.round(6).tolist()}"
            ),
            "T_unified_from_arm translations:",
        ]
        for arm in ARM_ORDER:
            xyz = self.T_unified_from_arm[arm][:3, 3]
            lines.append(f"  {arm:5s}: {xyz.round(6).tolist()}")
        return lines
