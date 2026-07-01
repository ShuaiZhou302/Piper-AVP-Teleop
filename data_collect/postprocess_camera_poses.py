#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
"""Add/rewrite per-frame wrist-camera poses in the unified arm frame.

This is for previously collected HDF5 episodes. New collection writes the same
group during save_data().
"""
import argparse
import os
import re
import sys

import h5py
import numpy as np


REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)


_EP_RE = re.compile(r"episode_(\d+)\.hdf5$")


def _episode_sort_key(path):
    base = os.path.basename(path)
    m = _EP_RE.search(base)
    if m:
        return (0, int(m.group(1)), base)
    return (1, base)


def find_hdf5_files(inputs):
    """Return unique HDF5 episode files from files/dirs, sorted by episode id."""
    found = []
    seen = set()
    for item in inputs:
        path = os.path.abspath(item)
        candidates = []
        if os.path.isdir(path):
            candidates = [
                os.path.join(path, name)
                for name in os.listdir(path)
                if name.endswith(".hdf5")
            ]
        elif os.path.isfile(path) and path.endswith(".hdf5"):
            candidates = [path]

        for cand in candidates:
            real = os.path.abspath(cand)
            if real not in seen:
                seen.add(real)
                found.append(real)
    return sorted(found, key=_episode_sort_key)


def _load_pose_helpers():
    from data_collect.collect_data_3arm import (  # noqa: WPS433
        _compute_camera_poses_in_unified,
        _compute_eef_poses_in_unified,
        annotate_raw_driver_ee_pose_groups,
        write_camera_fov_attrs,
        write_camera_poses_in_unified,
        write_eef_poses_in_unified,
    )

    return (
        _compute_camera_poses_in_unified,
        _compute_eef_poses_in_unified,
        annotate_raw_driver_ee_pose_groups,
        write_camera_fov_attrs,
        write_camera_poses_in_unified,
        write_eef_poses_in_unified,
    )


def postprocess_file(path, overwrite=True, dry_run=False):
    (
        compute_camera_poses,
        compute_eef_poses,
        annotate_raw_ee,
        write_fov_attrs,
        write_camera_poses,
        write_eef_poses,
    ) = _load_pose_helpers()
    with h5py.File(path, "r" if dry_run else "r+") as root:
        if "observations/qpos" not in root:
            raise KeyError(f"{path}: missing observations/qpos")
        qpos = np.asarray(root["observations/qpos"], dtype=float)
        if qpos.ndim != 2 or qpos.shape[1] < 21:
            raise ValueError(f"{path}: expected qpos shape (T, >=21), got {qpos.shape}")

        cam_poses, camera_summary = compute_camera_poses(qpos)
        if cam_poses is None:
            raise RuntimeError(f"{path}: camera pose conversion unavailable")
        eef_poses, eef_summary = compute_eef_poses(qpos)
        if eef_poses is None:
            raise RuntimeError(f"{path}: unified EEF pose conversion unavailable")

        if dry_run:
            return qpos.shape[0], camera_summary

        write_camera_poses(root, cam_poses, camera_summary, overwrite=overwrite)
        write_eef_poses(root, eef_poses, eef_summary, overwrite=overwrite)
        write_fov_attrs(root)
        annotate_raw_ee(root)
        return qpos.shape[0], camera_summary


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Post-process HDF5 episodes with unified camera/EEF poses and FOV."
    )
    parser.add_argument("inputs", nargs="+", help="episode_*.hdf5 file(s) or directory")
    parser.add_argument(
        "--no_overwrite",
        action="store_true",
        help="fail instead of replacing an existing camera_pose_in_unified group",
    )
    parser.add_argument(
        "--dry_run",
        action="store_true",
        help="compute and print what would be written, without modifying HDF5 files",
    )
    args = parser.parse_args(argv)

    files = find_hdf5_files(args.inputs)
    if not files:
        print("[postprocess] no .hdf5 files found")
        return 1

    failures = 0
    for path in files:
        try:
            nframes, summary = postprocess_file(
                path, overwrite=not args.no_overwrite, dry_run=args.dry_run
            )
            mode = "DRY" if args.dry_run else "OK"
            print(f"[postprocess] {mode}: {path} ({nframes} frames)")
            print(f"[postprocess]      extrinsics: {summary['extrinsics_json']}")
        except Exception as e:
            failures += 1
            print(f"[postprocess] FAIL: {path}: {e}", file=sys.stderr)

    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
