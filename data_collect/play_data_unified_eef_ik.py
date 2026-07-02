#!/usr/bin/env python3
"""Replay unified-frame EE poses by converting them back to arm-base IK targets."""
import argparse
import os
import sys

import h5py
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

from play_data_eef_ik import (  # noqa: E402
    ARM_ORDER,
    DEFAULT_URDF,
    EefIkPlanner,
    Player,
    print_summary,
    slice_for,
)


def _teleop_dir():
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.normpath(os.path.join(here, "..", "teleop"))


def import_unified_converter():
    teleop_dir = _teleop_dir()
    if teleop_dir not in sys.path:
        sys.path.insert(0, teleop_dir)
    from unified_eef_pose import UnifiedEefConverter  # noqa: WPS433
    return UnifiedEefConverter


def matrix_to_rpy_xyz(T):
    from tf.transformations import euler_from_matrix  # noqa: WPS433

    r, p, y = euler_from_matrix(T)
    t = T[:3, 3]
    return np.array([t[0], t[1], t[2], r, p, y], dtype=float)


def quaternion_matrix_xyzw(qxyzw):
    """Return 4x4 rotation matrix from quaternion (qx,qy,qz,qw)."""
    x, y, z, w = np.asarray(qxyzw, dtype=float)
    n = x * x + y * y + z * z + w * w
    if n <= 1e-12:
        raise ValueError("zero-norm quaternion")
    s = 2.0 / n
    xs, ys, zs = x * s, y * s, z * s
    wx, wy, wz = w * xs, w * ys, w * zs
    xx, xy, xz = x * xs, x * ys, x * zs
    yy, yz, zz = y * ys, y * zs, z * zs
    T = np.eye(4)
    T[:3, :3] = np.array([
        [1.0 - (yy + zz), xy - wz, xz + wy],
        [xy + wz, 1.0 - (xx + zz), yz - wx],
        [xz - wy, yz + wx, 1.0 - (xx + yy)],
    ])
    return T


def normalize_quat_signs(xyz_qxyzw):
    """Normalize quaternions and make signs continuous over time."""
    arr = np.asarray(xyz_qxyzw, dtype=float).copy()
    q = arr[:, 3:7]
    norms = np.linalg.norm(q, axis=1)
    if np.any(norms <= 1e-12):
        raise ValueError("zero-norm quaternion in unified EE pose")
    q /= norms[:, None]
    for t in range(1, q.shape[0]):
        if float(np.dot(q[t - 1], q[t])) < 0.0:
            q[t] *= -1.0
    arr[:, 3:7] = q
    return arr


def quat_xyz_to_matrices(xyz_qxyzw):
    """Convert (T,7) xyz+qx,qy,qz,qw to (T,4,4) matrices."""
    arr = normalize_quat_signs(xyz_qxyzw)
    out = np.empty((arr.shape[0], 4, 4), dtype=float)
    for t in range(arr.shape[0]):
        T = quaternion_matrix_xyzw(arr[t, 3:7])
        T[:3, 3] = arr[t, :3]
        out[t] = T
    return out


def unified_matrices_to_arm_rpy_poses(matrices, converter):
    """Convert T_unified_from_ee matrices to per-arm xyz+rpy IK targets."""
    poses = {}
    for arm, Ts in matrices.items():
        arr = np.asarray(Ts, dtype=float)
        out = np.empty((arr.shape[0], 6), dtype=float)
        for t in range(arr.shape[0]):
            T_arm_from_ee = converter.arm_pose_from_unified(arm, arr[t])
            out[t] = matrix_to_rpy_xyz(T_arm_from_ee)
        poses[arm] = out
    return poses


def load_episode(path, urdf=DEFAULT_URDF, pose_source="matrix"):
    UnifiedEefConverter = import_unified_converter()
    with h5py.File(path, "r") as f:
        action = f["action"][:]
        frame_rate = int(f.attrs.get("frame_rate", 30))
        task_name = str(f.attrs.get("task_name", "?"))
        task_desc = str(f.attrs.get("task_description", "?"))
        arm_order = str(f.attrs.get("arm_order", ",".join(ARM_ORDER)))
        if arm_order != ",".join(ARM_ORDER):
            raise ValueError(
                "HDF5 arm_order is %r, script expects %r"
                % (arm_order, ",".join(ARM_ORDER))
            )

        matrices = {}
        for arm in ARM_ORDER:
            key = f"observations/ee_pose_in_unified/{arm}/{pose_source}"
            if key not in f:
                raise KeyError(
                    f"missing HDF5 dataset: {key}; run "
                    "data_collect/postprocess_camera_poses.py first"
                )
            data = f[key][:]
            matrices[arm] = quat_xyz_to_matrices(data) if pose_source == "quat" else data

    T = action.shape[0]
    if action.shape[1:] != (len(ARM_ORDER) * 7,):
        raise ValueError(f"action shape should be (T, 21), got {action.shape}")
    for arm in ARM_ORDER:
        if matrices[arm].shape != (T, 4, 4):
            raise ValueError(f"{arm} unified EE matrix shape should be ({T}, 4, 4), "
                             f"got {matrices[arm].shape}")

    converter = UnifiedEefConverter(urdf_path=urdf)
    poses = unified_matrices_to_arm_rpy_poses(matrices, converter)
    return {
        "action": action,
        "poses": poses,
        "frame_rate": frame_rate,
        "task_name": task_name,
        "task_description": task_desc,
        "arm_order": arm_order,
    }


def get_args():
    parser = argparse.ArgumentParser(
        description="Replay unified-frame EE pose through IK for all 3 arms."
    )
    parser.add_argument("episode", type=str, help="Path to episode_*.hdf5")
    parser.add_argument("--urdf", type=str, default=DEFAULT_URDF)
    parser.add_argument("--frame_rate", type=int, default=None)
    parser.add_argument("--max_joint_step", type=float, default=0.05,
                        help="Per-frame joint step cap after IK. 0 disables clipping.")
    parser.add_argument("--pose_source", choices=("matrix", "quat"), default="matrix",
                        help="Read unified EE pose from matrix or quaternion dataset. "
                             "Quaternion mode normalizes and sign-continuizes qxyzw.")
    parser.add_argument("--respect_collision", action="store_true",
                        help="Reject IK solutions flagged as collision. Default ignores "
                             "the collision flag to match teleop/play_data_eef_ik.py.")
    parser.add_argument("--ramp_speed_rad_s", type=float, default=0.25)
    parser.add_argument("--execute", action="store_true",
                        help="Actually publish /master/joint_<arm>. Default is dry-run.")
    parser.add_argument("--no_confirm", action="store_true",
                        help="Skip ENTER prompts when --execute is used.")
    parser.add_argument("--save_commands", type=str, default=None,
                        help="Optional .npy path to save planned joint commands.")
    return parser.parse_args()


def main():
    args = get_args()
    if not os.path.isfile(args.episode):
        raise SystemExit(f"[unified-eef-play] no such file: {args.episode}")

    episode = load_episode(args.episode, urdf=args.urdf, pose_source=args.pose_source)
    planner = EefIkPlanner(
        args.urdf,
        max_joint_step=args.max_joint_step,
        input_frame="ik_ee",
        allow_collision=not args.respect_collision,
    )
    commands, stats = planner.plan(episode)
    print_summary(args, episode, stats)

    if args.save_commands:
        np.save(args.save_commands, commands)
        print(f"[unified-eef-play] saved commands: {args.save_commands}")

    if not args.execute:
        print("[unified-eef-play] dry-run only. Add --execute to publish robot commands.")
        return

    frame_rate = args.frame_rate or episode["frame_rate"]
    player = Player()
    print("[unified-eef-play] waiting for /puppet/joint_<arm> feedback...")
    if not player.wait_feedback(10.0):
        raise SystemExit("[unified-eef-play] timed out waiting for puppet feedback.")

    start_targets = {
        arm: commands[0, slice_for(arm_idx)]
        for arm_idx, arm in enumerate(ARM_ORDER)
    }
    for arm in ARM_ORDER:
        current = np.asarray(player.puppet[arm].position[:7], dtype=float)
        print(f"  {arm:5s} current: {current.round(3).tolist()}")
        print(f"  {arm:5s} target : {start_targets[arm].round(3).tolist()}")

    if not args.no_confirm:
        input("\n[unified-eef-play] Clear the workspace. Press ENTER to ramp to start.")
    if not player.ramp(start_targets, args.ramp_speed_rad_s, hz=frame_rate):
        return
    if not args.no_confirm:
        input("\n[unified-eef-play] At start pose. Press ENTER to play trajectory.")
    player.play(commands, frame_rate=frame_rate)


if __name__ == "__main__":
    main()
