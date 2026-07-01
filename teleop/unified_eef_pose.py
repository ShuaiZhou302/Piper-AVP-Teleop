#!/usr/bin/env python3
"""Compute Piper IK end-effector poses in the shared unified frame."""
import os
import sys

import numpy as np


HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

from piper_fk import (  # noqa: E402
    DEFAULT_URDF, PiperFkModel, matrix_to_quat_xyz, matrix_to_rpy_xyz,
)
from arm_unified_coords import ARM_ORDER, ArmUnifiedConverter  # noqa: E402


class UnifiedEefConverter:
    """Convert teleop IK `ee` frame poses between arm bases and unified frame."""

    def __init__(self, urdf_path=DEFAULT_URDF, extrinsics_json=None):
        if extrinsics_json is None:
            self.unified = ArmUnifiedConverter()
        else:
            self.unified = ArmUnifiedConverter(extrinsics_json=extrinsics_json)
        self.fk = PiperFkModel(urdf_path)
        self._urdf_path = urdf_path

    def eef_pose_in_unified(self, arm, joint_q6):
        """Return 4x4 T_unified_from_ee for one arm joint sample."""
        if arm not in ARM_ORDER:
            raise ValueError(f"unknown arm: {arm}")
        return self.unified.T_unified_from_arm[arm] @ self.fk.base_from_ee(joint_q6)

    def eef_poses_in_unified_batch(self, arm, joint_q6_array):
        """Vectorize over T samples: input (T, >=6) joints -> (T, 4, 4)."""
        arr = np.asarray(joint_q6_array, dtype=float)
        if arr.ndim != 2 or arr.shape[1] < 6:
            raise ValueError(f"expected (T, >=6), got {arr.shape}")
        out = np.empty((arr.shape[0], 4, 4), dtype=float)
        T_ua = self.unified.T_unified_from_arm[arm]
        for t in range(arr.shape[0]):
            out[t] = T_ua @ self.fk.base_from_ee(arr[t, :6])
        return out

    def arm_pose_from_unified(self, arm, T_unified_from_ee):
        """Return 4x4 T_arm_from_ee for a unified-frame EE pose."""
        if arm not in ARM_ORDER:
            raise ValueError(f"unknown arm: {arm}")
        return self.unified.T_arm_from_unified[arm] @ np.asarray(T_unified_from_ee)

    def summary(self):
        return {
            "urdf": self._urdf_path,
            "extrinsics_json": self.unified.extrinsics_json,
            "unified_origin_in_mid_m": self.unified.unified_origin_in_mid.tolist(),
            "eef_frame": "teleop_ik_ee",
        }


if __name__ == "__main__":
    conv = UnifiedEefConverter()
    for line in conv.unified.summary_lines():
        print(line)
    print("\nEEF pose @ joints=(0,0,0,0,0,0) in unified:")
    for arm in ARM_ORDER:
        T = conv.eef_pose_in_unified(arm, np.zeros(6))
        print(f"  {arm:5s}: t = {T[:3, 3].round(4).tolist()} m")
