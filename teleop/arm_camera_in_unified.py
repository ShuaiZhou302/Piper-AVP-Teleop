#!/usr/bin/env python3
"""
Compute each wrist camera's pose in the unified frame.

Chain:
    T_unified_from_camera(arm, q) =
        T_unified_from_arm_base[arm]      # touch-point extrinsics
      @ FK_arm(q)                         # pinocchio, "ee" frame
      @ T_wrist_from_camera[arm]          # hand-eye calibration

The unified frame is defined in arm_unified_coords.py (mid_base orientation,
origin 25 cm below mid_base along its Z).

Intended use: OFFLINE post-processing during HDF5 save in collect_data_3arm.py.
Pinocchio + the URDF are loaded once; per-frame FK is a few microseconds.
"""
import glob
import json
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

DEFAULT_CAL_DIR = os.path.join(HERE, "calibration_outputs")


def find_latest_handeye(arm, cal_dir=DEFAULT_CAL_DIR):
    """Return the most recent handeye_<arm>_*.json by timestamp suffix."""
    pat = os.path.join(cal_dir, f"handeye_{arm}_*.json")
    files = sorted(glob.glob(pat))  # YYYYMMDD_HHMMSS sorts lexicographically
    if not files:
        raise FileNotFoundError(f"no hand-eye calibration for arm={arm} in {cal_dir}")
    return files[-1]


def load_wrist_from_camera(arm, cal_dir=DEFAULT_CAL_DIR, json_path=None):
    """Load the 4x4 T_wrist_from_camera for ONE arm. Returns (matrix, source_path)."""
    path = json_path or find_latest_handeye(arm, cal_dir)
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    X = np.asarray(data["T_wrist_from_camera"]["matrix"], dtype=float)
    if X.shape != (4, 4):
        raise ValueError(f"bad matrix in {path}: shape={X.shape}")
    return X, path


class WristCameraUnifiedConverter:
    """Combine arm-arm extrinsics + hand-eye + FK -> camera pose in unified.

    Lazily imports pinocchio so callers that only need arm-arm transforms
    can keep using ArmUnifiedConverter without pulling pinocchio in.
    """

    def __init__(
        self,
        urdf_path=DEFAULT_URDF,
        cal_dir=DEFAULT_CAL_DIR,
        extrinsics_json=None,
        handeye_paths=None,
    ):
        # Arm-arm (touch-point) extrinsics -> unified frame.
        if extrinsics_json is None:
            self.unified = ArmUnifiedConverter()
        else:
            self.unified = ArmUnifiedConverter(extrinsics_json=extrinsics_json)

        # Hand-eye per arm.
        handeye_paths = handeye_paths or {}
        self.T_wrist_from_camera = {}
        self.handeye_source = {}
        for arm in ARM_ORDER:
            X, src = load_wrist_from_camera(
                arm, cal_dir=cal_dir, json_path=handeye_paths.get(arm),
            )
            self.T_wrist_from_camera[arm] = X
            self.handeye_source[arm] = src

        self.fk = PiperFkModel(urdf_path)
        self._urdf_path = urdf_path

    # ---------- FK helper ----------
    def fk_base_from_wrist(self, joint_q6):
        """Forward-kinematics: 6-vector joint -> 4x4 T_base_from_wrist (ee frame)."""
        return self.fk.base_from_ee(joint_q6)

    # ---------- Main API ----------
    def camera_pose_in_unified(self, arm, joint_q6):
        """Return 4x4 T_unified_from_camera for one (arm, joint) sample."""
        if arm not in ARM_ORDER:
            raise ValueError(f"unknown arm: {arm}")
        T_bw = self.fk_base_from_wrist(joint_q6)
        return self.unified.T_unified_from_arm[arm] @ T_bw @ self.T_wrist_from_camera[arm]

    def camera_poses_in_unified_batch(self, arm, joint_q6_array):
        """Vectorize over T samples: input (T, >=6) joints -> (T, 4, 4)."""
        arr = np.asarray(joint_q6_array, dtype=float)
        if arr.ndim != 2 or arr.shape[1] < 6:
            raise ValueError(f"expected (T, >=6), got {arr.shape}")
        T = arr.shape[0]
        out = np.empty((T, 4, 4), dtype=float)
        T_ua = self.unified.T_unified_from_arm[arm]
        T_wc = self.T_wrist_from_camera[arm]
        for t in range(T):
            out[t] = T_ua @ self.fk_base_from_wrist(arr[t, :6]) @ T_wc
        return out

    # ---------- Metadata for HDF5 attrs ----------
    def summary(self):
        s = {
            "urdf": self._urdf_path,
            "extrinsics_json": self.unified.extrinsics_json,
            "unified_origin_in_mid_m": self.unified.unified_origin_in_mid.tolist(),
            "handeye_per_arm": {arm: self.handeye_source[arm] for arm in ARM_ORDER},
        }
        return s

if __name__ == "__main__":
    # Smoke test: load + print summary + one FK at zero joints.
    conv = WristCameraUnifiedConverter()
    for line in conv.unified.summary_lines():
        print(line)
    print("\nhand-eye sources:")
    for arm, src in conv.handeye_source.items():
        print(f"  {arm:5s}: {os.path.basename(src)}")
        X = conv.T_wrist_from_camera[arm]
        print(f"         |t| = {np.linalg.norm(X[:3,3])*100:.2f} cm")
    print("\nCamera pose @ joints=(0,0,0,0,0,0) in unified:")
    for arm in ARM_ORDER:
        T = conv.camera_pose_in_unified(arm, np.zeros(6))
        print(f"  {arm:5s}: t = {T[:3, 3].round(4).tolist()} m")
