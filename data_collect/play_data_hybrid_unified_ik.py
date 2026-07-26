#!/usr/bin/env python3
"""Replay left/right unified EEF poses and mid unified camera pose through IK.

Hybrid target convention:
  * left/right arms track observations/ee_pose_in_unified/<arm>.
  * mid arm tracks observations/camera_pose_in_unified/cam_front by converting
    the desired camera pose back to the mid teleop IK EE frame with hand-eye.

Default mode is dry-run. Use --execute to publish /master/joint_<arm>.
"""
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
    slice_for,
)
from play_data_unified_eef_ik import (  # noqa: E402
    import_unified_converter,
    matrix_to_rpy_xyz,
    quat_xyz_to_matrices,
    unified_matrices_to_arm_rpy_poses,
)


def _teleop_dir():
    return os.path.normpath(os.path.join(HERE, "..", "teleop"))


def import_wrist_camera_converter():
    teleop_dir = _teleop_dir()
    if teleop_dir not in sys.path:
        sys.path.insert(0, teleop_dir)
    from arm_camera_in_unified import WristCameraUnifiedConverter  # noqa: WPS433
    return WristCameraUnifiedConverter


def load_pose_matrices(f, key, pose_source):
    if key not in f:
        raise KeyError(f"missing HDF5 dataset: {key}")
    data = f[key][:]
    if pose_source == "quat":
        return quat_xyz_to_matrices(data)
    return np.asarray(data, dtype=float)


def camera_matrices_to_mid_ee(camera_matrices, urdf):
    """Convert T_unified_from_camera to T_unified_from_mid_ee."""
    WristCameraUnifiedConverter = import_wrist_camera_converter()
    cam_conv = WristCameraUnifiedConverter(urdf_path=urdf)
    T_ee_from_camera = cam_conv.T_wrist_from_camera["mid"]
    T_camera_from_ee = np.linalg.inv(T_ee_from_camera)
    arr = np.asarray(camera_matrices, dtype=float)
    out = np.empty_like(arr)
    for t in range(arr.shape[0]):
        out[t] = arr[t] @ T_camera_from_ee
    return out, cam_conv.summary()


def load_episode(path, urdf=DEFAULT_URDF, pose_source="matrix", mid_source="camera"):
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
        for arm in ("left", "right"):
            key = f"observations/ee_pose_in_unified/{arm}/{pose_source}"
            matrices[arm] = load_pose_matrices(f, key, pose_source)

        camera_summary = None
        if mid_source == "camera":
            key = f"observations/camera_pose_in_unified/cam_front/{pose_source}"
            cam_mats = load_pose_matrices(f, key, pose_source)
            matrices["mid"], camera_summary = camera_matrices_to_mid_ee(cam_mats, urdf)
        else:
            key = f"observations/ee_pose_in_unified/mid/{pose_source}"
            matrices["mid"] = load_pose_matrices(f, key, pose_source)

    T = action.shape[0]
    if action.shape[1:] != (len(ARM_ORDER) * 7,):
        raise ValueError(f"action shape should be (T, 21), got {action.shape}")
    for arm in ARM_ORDER:
        if matrices[arm].shape != (T, 4, 4):
            raise ValueError(
                f"{arm} target matrix shape should be ({T}, 4, 4), "
                f"got {matrices[arm].shape}"
            )

    converter = UnifiedEefConverter(urdf_path=urdf)
    poses = unified_matrices_to_arm_rpy_poses(matrices, converter)
    return {
        "action": action,
        "poses": poses,
        "frame_rate": frame_rate,
        "task_name": task_name,
        "task_description": task_desc,
        "arm_order": arm_order,
        "mid_source": mid_source,
        "pose_source": pose_source,
        "camera_summary": camera_summary,
    }


def print_summary(args, episode, stats):
    print("=" * 68)
    print(f"[hybrid-play] episode    : {args.episode}")
    print(f"[hybrid-play] task       : {episode['task_name']}")
    print(f"[hybrid-play] desc       : {episode['task_description']}")
    print(f"[hybrid-play] frames     : {stats['frames']}")
    print(f"[hybrid-play] frame_rate : {args.frame_rate or episode['frame_rate']} Hz")
    print(f"[hybrid-play] pose_source: {episode['pose_source']}")
    print("[hybrid-play] targets    : left/right=unified EE, "
          f"mid={episode['mid_source']}")
    print(f"[hybrid-play] max_step   : {args.max_joint_step} rad/frame")
    print(f"[hybrid-play] mode       : {'EXECUTE' if args.execute else 'dry-run'}")
    if episode.get("camera_summary"):
        src = episode["camera_summary"]["handeye_per_arm"].get("mid")
        print(f"[hybrid-play] mid handeye: {src}")
    for arm in ARM_ORDER:
        print(
            f"[hybrid-play] {arm:5s}: ik_fail={stats['failures'][arm]} "
            f"first_fail={stats['first_failure'][arm]} "
            f"ramp_delta={stats['ramp_delta'][arm]:.4f} "
            f"max_step={stats['max_step_seen'][arm]:.4f}"
        )
    print("=" * 68)


def get_args():
    parser = argparse.ArgumentParser(
        description=(
            "Replay hybrid unified targets: left/right EEF pose plus mid camera "
            "pose converted back to mid EEF IK target."
        )
    )
    parser.add_argument("episode", type=str, help="Path to episode_*.hdf5")
    parser.add_argument("--urdf", type=str, default=DEFAULT_URDF)
    parser.add_argument("--frame_rate", type=int, default=None)
    parser.add_argument("--max_joint_step", type=float, default=0.05,
                        help="Per-frame joint step cap after IK. 0 disables clipping.")
    parser.add_argument("--pose_source", choices=("matrix", "quat"), default="quat",
                        help="Read unified pose from matrix or xyz+qxyzw datasets.")
    parser.add_argument("--mid_source", choices=("camera", "ee"), default="camera",
                        help="camera tracks cam_front pose; ee tracks mid EEF pose.")
    parser.add_argument("--respect_collision", action="store_true",
                        help="Reject IK solutions flagged as collision. Default ignores "
                             "collision to match current teleop/playback behavior.")
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
        raise SystemExit(f"[hybrid-play] no such file: {args.episode}")

    episode = load_episode(
        args.episode,
        urdf=args.urdf,
        pose_source=args.pose_source,
        mid_source=args.mid_source,
    )
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
        print(f"[hybrid-play] saved commands: {args.save_commands}")

    if not args.execute:
        print("[hybrid-play] dry-run only. Add --execute to publish robot commands.")
        return

    frame_rate = args.frame_rate or episode["frame_rate"]
    player = Player()
    print("[hybrid-play] waiting for /puppet/joint_<arm> feedback...")
    if not player.wait_feedback(10.0):
        raise SystemExit("[hybrid-play] timed out waiting for puppet feedback.")

    start_targets = {
        arm: commands[0, slice_for(arm_idx)]
        for arm_idx, arm in enumerate(ARM_ORDER)
    }
    for arm in ARM_ORDER:
        current = np.asarray(player.puppet[arm].position[:7], dtype=float)
        print(f"  {arm:5s} current: {current.round(3).tolist()}")
        print(f"  {arm:5s} target : {start_targets[arm].round(3).tolist()}")

    if not args.no_confirm:
        input("\n[hybrid-play] Clear the workspace. Press ENTER to ramp to start.")
    if not player.ramp(start_targets, args.ramp_speed_rad_s, hz=frame_rate):
        return
    if not args.no_confirm:
        input("\n[hybrid-play] At start pose. Press ENTER to play hybrid trajectory.")
    player.play(commands, frame_rate=frame_rate)


if __name__ == "__main__":
    main()
