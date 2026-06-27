#!/usr/bin/env python3
"""
Extract side-by-side MP4 videos from collected HDF5 episodes.

Default: 3 cameras horizontally stacked (LEFT | FRONT | RIGHT), so you can
glance-review what each arm saw + the mid arm's overhead-ish view in one file.
"""
import argparse
import glob
import os

import cv2
import h5py
import numpy as np


def extract(hdf5_path, out_dir, fps=30):
    name = os.path.splitext(os.path.basename(hdf5_path))[0]
    with h5py.File(hdf5_path, "r") as f:
        imgs_f = f["observations/images/cam_front"][:]
        imgs_l = f["observations/images/cam_left"][:]
        imgs_r = f["observations/images/cam_right"][:]
    T, H, W, _ = imgs_f.shape
    out_path = os.path.join(out_dir, f"{name}_3cam.mp4")

    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    vw = cv2.VideoWriter(out_path, fourcc, fps, (W * 3, H))
    if not vw.isOpened():
        raise RuntimeError(f"VideoWriter failed for {out_path}")

    for i in range(T):
        # HDF5 stores RGB; cv2 wants BGR.
        frame_rgb = np.hstack([imgs_l[i], imgs_f[i], imgs_r[i]])
        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        vw.write(frame_bgr)
    vw.release()
    print(f"[extract] wrote {out_path}  ({T} frames, {T / fps:.1f}s)")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--dataset_dir", default="~/data_shuai_3arm")
    p.add_argument("--task_name",   default="three_arm_pickup")
    p.add_argument("--fps",         type=int, default=30)
    p.add_argument("--out_dir",     default=None,
                   help="Default: same as the task folder.")
    args = p.parse_args()

    task_dir = os.path.join(os.path.expanduser(args.dataset_dir), args.task_name)
    out_dir = args.out_dir or task_dir
    os.makedirs(out_dir, exist_ok=True)

    files = sorted(glob.glob(os.path.join(task_dir, "episode_*.hdf5")))
    print(f"[extract] found {len(files)} episodes in {task_dir}")
    for fp in files:
        extract(fp, out_dir, args.fps)


if __name__ == "__main__":
    main()
