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
import pinocchio as pin
from tf.transformations import quaternion_from_euler

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

from arm_unified_coords import ARM_ORDER, ArmUnifiedConverter  # noqa: E402

DEFAULT_CAL_DIR = os.path.join(HERE, "calibration_outputs")
DEFAULT_URDF = os.path.join(HERE, "urdf", "piper_description.urdf")


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

        # Pinocchio FK only. Keep this independent of PinocchioIKSolver/CasADi
        # so collect_data_3arm.py can post-process after rospy is already loaded.
        self._pin = pin
        self._robot, self._model, self._data = self._build_fk_model(urdf_path)
        self._ee_frame_id = self._model.getFrameId("ee")
        self._urdf_path = urdf_path

    def _package_dirs_for_urdf(self, urdf_path):
        ap = os.path.abspath(urdf_path)
        package_dirs = []
        marker = os.sep + "piper_description" + os.sep
        i = ap.find(marker)
        if i >= 0:
            package_dirs.append(ap[:i])
        repo_piper_src = os.path.normpath(os.path.join(HERE, "..", "piper_ros", "src"))
        if os.path.isdir(os.path.join(repo_piper_src, "piper_description", "meshes")):
            package_dirs.append(repo_piper_src)
        return package_dirs

    def _build_fk_model(self, urdf_path):
        pin = self._pin
        robot = pin.RobotWrapper.BuildFromURDF(
            urdf_path, package_dirs=self._package_dirs_for_urdf(urdf_path)
        )
        model = robot.buildReducedRobot(
            list_of_joints_to_lock=["joint7", "joint8"],
            reference_configuration=np.array([0.0] * robot.model.nq),
        ).model

        # Match teleop/eef_keyboard_control_singlearm.py exactly: the IK/control
        # frame named "ee" is joint6 rotated by -90 deg around local Y.
        ee_off_quat = quaternion_from_euler(0.0, -np.pi / 2.0, 0.0)
        model.addFrame(
            pin.Frame(
                "ee",
                model.getJointId("joint6"),
                pin.SE3(
                    pin.Quaternion(
                        ee_off_quat[3], ee_off_quat[0],
                        ee_off_quat[1], ee_off_quat[2],
                    ),
                    np.array([0.0, 0.0, 0.0]),
                ),
                pin.FrameType.OP_FRAME,
            )
        )
        data = model.createData()
        return robot, model, data

    # ---------- FK helper ----------
    def fk_base_from_wrist(self, joint_q6):
        """Forward-kinematics: 6-vector joint -> 4x4 T_base_from_wrist (ee frame)."""
        q = np.asarray(joint_q6, dtype=float).flatten()
        if q.size < 6:
            raise ValueError(f"need 6 joints, got {q.size}")
        self._pin.framesForwardKinematics(self._model, self._data, q[:6])
        se3 = self._data.oMf[self._ee_frame_id]
        T = np.eye(4)
        T[:3, :3] = np.asarray(se3.rotation, dtype=float)
        T[:3, 3] = np.asarray(se3.translation, dtype=float).flatten()
        return T

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


# ---------------- HDF5 conversion helpers ----------------
def matrix_to_quat_xyz(T):
    """4x4 -> (x, y, z, qx, qy, qz, qw). Quaternion via tf.transformations."""
    from tf.transformations import quaternion_from_matrix
    q = quaternion_from_matrix(T)  # returns (x, y, z, w)
    t = T[:3, 3]
    return np.array([t[0], t[1], t[2], q[0], q[1], q[2], q[3]], dtype=float)


def matrix_to_rpy_xyz(T):
    """4x4 -> (x, y, z, roll, pitch, yaw)."""
    from tf.transformations import euler_from_matrix
    r, p, y = euler_from_matrix(T)
    t = T[:3, 3]
    return np.array([t[0], t[1], t[2], r, p, y], dtype=float)


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
