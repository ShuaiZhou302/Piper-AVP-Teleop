#!/usr/bin/env python3
"""Visualize a UMI human AVP HDF5 episode as an annotated video."""
import argparse
import os

import h5py
import numpy as np
from PIL import Image, ImageDraw, ImageFont


def rpy_from_matrix_zyx(R):
    sy = -float(np.clip(R[2, 0], -1.0, 1.0))
    pitch = np.arcsin(sy)
    if abs(sy) < 0.9999:
        roll = np.arctan2(R[2, 1], R[2, 2])
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll = 0.0
        yaw = np.arctan2(-R[0, 1], R[1, 1])
    return np.rad2deg([roll, pitch, yaw])


def default_out_path(episode_path):
    stem, _ = os.path.splitext(os.path.abspath(episode_path))
    return stem + "_preview.mp4"


def load_font(size=22):
    for path in (
        "/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
    ):
        try:
            return ImageFont.truetype(path, size)
        except OSError:
            pass
    return ImageFont.load_default()


def draw_landmarks(draw, landmarks, color, radius=3):
    # WebXR landmarks are 3D AVP-world points, not image-space keypoints. We draw
    # a compact normalized XY diagnostic inset, useful for checking tracking.
    pts = np.asarray(landmarks, dtype=float)
    if pts.shape != (25, 3) or not np.isfinite(pts).all() or np.allclose(pts, 0.0):
        return
    xy = pts[:, :2]
    center = np.nanmean(xy, axis=0)
    xy = xy - center
    scale = max(float(np.nanmax(np.abs(xy))), 1e-3)
    xy = xy / scale
    origin = np.array([95, 380], dtype=float)
    pix = origin + xy * np.array([55, -55], dtype=float)
    for x, y in pix:
        draw.ellipse((x - radius, y - radius, x + radius, y + radius), fill=color)


def derive_fresh_masks(landmarks, stale_threshold=45):
    fresh = np.zeros((landmarks.shape[0],), dtype=bool)
    stale = 0
    prev = None
    for i in range(landmarks.shape[0]):
        lm = landmarks[i]
        if np.all(lm == 0):
            prev = None
            stale = stale_threshold + 1
            fresh[i] = False
            continue
        if prev is not None and np.array_equal(lm, prev):
            stale += 1
        else:
            stale = 0
        prev = lm.copy()
        fresh[i] = stale < stale_threshold
    return fresh


def pinch_distances(landmarks):
    thumb = landmarks[:, 4, :]
    middle = landmarks[:, 14, :]
    return np.linalg.norm(thumb - middle, axis=1)


def annotate_frame(img, idx, total, head_rel, left_lm, right_lm, font, tracking=None):
    canvas = Image.fromarray(img.astype(np.uint8, copy=False)).convert("RGB")
    draw = ImageDraw.Draw(canvas, "RGBA")
    w, _ = canvas.size

    xyz = head_rel[:3, 3]
    rpy = rpy_from_matrix_zyx(head_rel[:3, :3])
    lines = [
        f"frame {idx + 1}/{total}",
        f"head rel xyz {xyz[0]:+.3f} {xyz[1]:+.3f} {xyz[2]:+.3f} m",
        f"head rel rpy {rpy[0]:+.1f} {rpy[1]:+.1f} {rpy[2]:+.1f} deg",
    ]
    if tracking is not None:
        suffix = " derived" if tracking.get("derived") else ""
        lines.append(
            f"fresh L/R {int(tracking['left_fresh'])}/{int(tracking['right_fresh'])}  "
            f"pinch {tracking['pinch'][0]:.3f}/{tracking['pinch'][1]:.3f} m{suffix}"
        )

    draw.rectangle((0, 0, w, 122 if tracking is not None else 94), fill=(0, 0, 0, 145))
    for i, line in enumerate(lines):
        draw.text((14, 10 + i * 27), line, fill=(235, 245, 255), font=font)

    draw.rectangle((10, 300, 180, 470), fill=(0, 0, 0, 125))
    draw.text((20, 310), "hand lm diag", fill=(235, 235, 235), font=font)
    draw_landmarks(draw, left_lm, color=(80, 180, 255, 230))
    draw_landmarks(draw, right_lm, color=(255, 130, 90, 230))
    draw.text((20, 435), "L blue  R orange", fill=(235, 235, 235), font=font)
    return np.asarray(canvas)


def write_video(frames, out_path, fps):
    try:
        import cv2  # noqa: WPS433
    except Exception as e:
        raise RuntimeError("OpenCV is required to write mp4 previews") from e

    if not frames:
        raise ValueError("no frames to write")
    h, w = frames[0].shape[:2]
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(out_path, fourcc, float(fps), (w, h))
    if not writer.isOpened():
        raise RuntimeError(f"failed to open video writer: {out_path}")
    try:
        for frame in frames:
            writer.write(frame[:, :, ::-1])
    finally:
        writer.release()


def main():
    p = argparse.ArgumentParser(description="Create an annotated mp4 from UMI human HDF5.")
    p.add_argument("episode", help="Path to episode_*.hdf5")
    p.add_argument("--out", default=None)
    p.add_argument("--fps", type=float, default=None)
    p.add_argument("--stride", type=int, default=1)
    p.add_argument("--max_frames", type=int, default=0)
    args = p.parse_args()

    out = args.out or default_out_path(args.episode)
    font = load_font()
    frames = []
    with h5py.File(args.episode, "r") as f:
        images = f["observations/images/cam_front"]
        head_rel = f["observations/avp/head/relative_matrix"]
        left_lm = f["observations/avp/left_landmarks"]
        right_lm = f["observations/avp/right_landmarks"]
        tr = f.get("observations/avp/tracking")
        left_fresh = tr.get("left_hand_fresh") if tr is not None else None
        right_fresh = tr.get("right_hand_fresh") if tr is not None else None
        pinch = tr.get("pinch_distance") if tr is not None else None
        derived_tracking = False
        if left_fresh is None or right_fresh is None or pinch is None:
            fps_for_stale = float(f.attrs.get("frame_rate", 30.0))
            stale_repeat_s = float(f.attrs.get("stale_repeat_s", 1.5))
            stale_threshold = max(3, int(round(stale_repeat_s * fps_for_stale)))
            left_lm_all = left_lm[:]
            right_lm_all = right_lm[:]
            left_fresh = derive_fresh_masks(left_lm_all, stale_threshold=stale_threshold)
            right_fresh = derive_fresh_masks(right_lm_all, stale_threshold=stale_threshold)
            pinch = np.stack(
                [pinch_distances(left_lm_all), pinch_distances(right_lm_all)], axis=1
            )
            derived_tracking = True
        total = images.shape[0]
        fps = args.fps or float(f.attrs.get("frame_rate", 30.0))
        indices = range(0, total, max(1, args.stride))
        for n, i in enumerate(indices):
            if args.max_frames > 0 and n >= args.max_frames:
                break
            tracking = None
            if left_fresh is not None and right_fresh is not None and pinch is not None:
                tracking = {
                    "left_fresh": bool(left_fresh[i]),
                    "right_fresh": bool(right_fresh[i]),
                    "pinch": pinch[i],
                    "derived": derived_tracking,
                }
            frames.append(
                annotate_frame(
                    images[i],
                    i,
                    total,
                    head_rel[i],
                    left_lm[i],
                    right_lm[i],
                    font,
                    tracking=tracking,
                )
            )

    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    write_video(frames, out, fps / max(1, args.stride))
    print(f"[visualize] wrote {len(frames)} frames -> {out}")


if __name__ == "__main__":
    main()
