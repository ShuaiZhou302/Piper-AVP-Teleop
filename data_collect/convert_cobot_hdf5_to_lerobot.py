#!/usr/bin/env python3
"""Convert Cobot AVP Teleop HDF5 episodes to a LeRobot v3-style dataset.

The source HDF5 files used here store each camera as a single lossless MP4 byte
blob under observations/images/<cam>. LeRobot stores visual observations as
MP4 files, so this converter extracts those blobs directly instead of decoding
and re-encoding pixels. Numeric streams are written to parquet.

This script intentionally avoids importing lerobot so it can run on cluster
nodes that only have h5py, pandas, pyarrow and ffmpeg/ffprobe available.
"""

from __future__ import annotations

import argparse
import json
import re
import shutil
import subprocess
from pathlib import Path

import h5py
import numpy as np
import pandas as pd
import pyarrow as pa
import pyarrow.parquet as pq


TASK_DIR_RE = re.compile(r"^(?!.*INCOMPLETE)(?!.*BACKUP).+")
EP_RE = re.compile(r"episode_(\d+)\.hdf5$")
CAMERAS = ("cam_front", "cam_left", "cam_right")
ARMS = ("left", "right", "mid")
HYBRID_TARGET_NOTE = (
    "Recommended VLA action target: left/right use unified teleop IK EEF pose, "
    "mid uses unified cam_front camera pose; use only left/right grippers."
)
FPS_DEFAULT = 30
CHUNK_SIZE = 1000


def episode_sort_key(path: Path) -> int:
    m = EP_RE.match(path.name)
    if not m:
        return 10**12
    return int(m.group(1))


def find_episodes(root: Path) -> list[Path]:
    files: list[Path] = []
    for task_dir in sorted(p for p in root.iterdir() if p.is_dir()):
        if not TASK_DIR_RE.match(task_dir.name):
            continue
        files.extend(sorted(task_dir.glob("episode_*.hdf5"), key=episode_sort_key))
    return files


def read_mp4_bytes(ds) -> bytes:
    item = ds[0]
    if isinstance(item, bytes):
        return item
    return np.asarray(item, dtype=np.uint8).tobytes()


def probe_video(path: Path) -> dict:
    cmd = [
        "ffprobe", "-v", "error",
        "-select_streams", "v:0",
        "-show_entries",
        "stream=codec_name,width,height,pix_fmt,r_frame_rate,avg_frame_rate",
        "-of", "json",
        str(path),
    ]
    try:
        out = subprocess.check_output(cmd, text=True)
        streams = json.loads(out).get("streams", [])
        return streams[0] if streams else {}
    except Exception:
        return {}


def vector_feature(size: int, names=None, dtype="float32") -> dict:
    return {"dtype": dtype, "shape": [size], "names": names}


def matrix_feature(shape: list[int], names=None, dtype="float32") -> dict:
    return {"dtype": dtype, "shape": shape, "names": names}


def video_feature(height: int, width: int, channels: int = 3, info: dict | None = None) -> dict:
    ft = {
        "dtype": "video",
        "shape": [height, width, channels],
        "names": ["height", "width", "channel"],
    }
    if info:
        ft["info"] = info
    return ft


def default_features(height: int, width: int, video_infos: dict[str, dict]) -> dict:
    q_names = [f"{arm}.joint_{i}" for arm in ARMS for i in range(6)] + [
        f"{arm}.gripper" for arm in ARMS
    ]
    action_names = q_names
    features = {
        "observation.state": vector_feature(21, q_names),
        "observation.qvel": vector_feature(21, q_names),
        "observation.effort": vector_feature(21, q_names),
        "action": vector_feature(21, action_names),
        "base_action": vector_feature(2, ["linear_x", "angular_z"]),
        "action.hybrid.left_ee_pose_in_unified.quat": vector_feature(
            7, ["x", "y", "z", "qx", "qy", "qz", "qw"]
        ),
        "action.hybrid.right_ee_pose_in_unified.quat": vector_feature(
            7, ["x", "y", "z", "qx", "qy", "qz", "qw"]
        ),
        "action.hybrid.mid_camera_pose_in_unified.quat": vector_feature(
            7, ["x", "y", "z", "qx", "qy", "qz", "qw"]
        ),
        "action.hybrid.gripper": vector_feature(2, ["left.gripper", "right.gripper"]),
    }
    for cam in CAMERAS:
        features[f"observation.images.{cam}"] = video_feature(
            height, width, 3, video_infos.get(cam)
        )
        features[f"observation.camera_pose_in_unified.quat.{cam}"] = vector_feature(
            7, ["x", "y", "z", "qx", "qy", "qz", "qw"]
        )
        features[f"observation.camera_pose_in_unified.rpy.{cam}"] = vector_feature(
            6, ["x", "y", "z", "roll", "pitch", "yaw"]
        )
        features[f"observation.camera_pose_in_unified.matrix.{cam}"] = matrix_feature(
            [4, 4], ["row", "col"]
        )
    for arm in ARMS:
        features[f"observation.ee_pose_quat.{arm}"] = vector_feature(
            7, ["x", "y", "z", "qx", "qy", "qz", "qw"]
        )
        features[f"observation.ee_pose_rpy.{arm}"] = vector_feature(
            6, ["x", "y", "z", "roll", "pitch", "yaw"]
        )
        features[f"observation.ee_pose_in_unified.quat.{arm}"] = vector_feature(
            7, ["x", "y", "z", "qx", "qy", "qz", "qw"]
        )
        features[f"observation.ee_pose_in_unified.rpy.{arm}"] = vector_feature(
            6, ["x", "y", "z", "roll", "pitch", "yaw"]
        )
        features[f"observation.ee_pose_in_unified.matrix.{arm}"] = matrix_feature(
            [4, 4], ["row", "col"]
        )
    return features


def base_meta_features() -> dict:
    return {
        "timestamp": {"dtype": "float32", "shape": [1], "names": None},
        "frame_index": {"dtype": "int64", "shape": [1], "names": None},
        "episode_index": {"dtype": "int64", "shape": [1], "names": None},
        "index": {"dtype": "int64", "shape": [1], "names": None},
        "task_index": {"dtype": "int64", "shape": [1], "names": None},
    }


def to_table(rows: dict[str, list]) -> pa.Table:
    return pa.Table.from_pydict(rows)


def write_data_parquet(rows: dict[str, list], out_path: Path) -> None:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    table = to_table(rows)
    pq.write_table(table, out_path, compression="snappy")


def write_episodes_parquet(rows: list[dict], out_path: Path) -> None:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    table = pa.Table.from_pylist(rows)
    pq.write_table(table, out_path, compression="snappy")


def append_array(rows: dict[str, list], key: str, arr, dtype=np.float32) -> None:
    rows[key].append(np.asarray(arr, dtype=dtype).tolist())


def empty_rows(row_keys: list[str]) -> dict[str, list]:
    return {key: [] for key in row_keys}


def json_safe(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace")
    return value


def read_camera_info(f) -> dict:
    if "camera_info" not in f:
        return {}
    out = {}
    for cam in CAMERAS:
        path = f"camera_info/{cam}"
        if path not in f:
            continue
        g = f[path]
        cam_info = {"attrs": {}}
        for key in ("K", "D", "R", "P"):
            if key in g:
                cam_info[key] = np.asarray(g[key][...]).tolist()
        for key, value in g.attrs.items():
            cam_info["attrs"][key] = json_safe(value)
        out[cam] = cam_info
    return out


def read_pose_group_attrs(f, group_name: str) -> dict:
    if group_name not in f:
        return {}
    return {key: json_safe(value) for key, value in f[group_name].attrs.items()}


def convert(args) -> None:
    src_root = Path(args.src).expanduser().resolve()
    out_root = Path(args.out).expanduser().resolve()
    if out_root.exists() and args.overwrite:
        shutil.rmtree(out_root)
    out_root.mkdir(parents=True, exist_ok=not args.overwrite)

    episodes = find_episodes(src_root)
    if args.max_episodes:
        episodes = episodes[: args.max_episodes]
    if not episodes:
        raise SystemExit(f"No episode_*.hdf5 found under {src_root}")

    tasks: dict[str, int] = {}
    episode_rows: list[dict] = []
    data_rows: dict[str, list] = {}
    video_infos: dict[str, dict] = {}
    camera_info: dict = {}
    pose_frame_metadata: dict = {}
    global_index = 0
    total_frames = 0
    data_file_index = 0
    episodes_in_data_file = 0
    fps = args.fps

    print(f"[lerobot-convert] src={src_root}")
    print(f"[lerobot-convert] out={out_root}")
    print(f"[lerobot-convert] episodes={len(episodes)}")

    # Discover shape/fps and seed feature keys from the first file.
    with h5py.File(episodes[0], "r") as f0:
        fps = int(f0.attrs.get("frame_rate", fps or FPS_DEFAULT))
        shape = np.asarray(f0["observations/images/cam_front"].attrs["shape"], dtype=int)
        _, height, width, channels = shape.tolist()
        camera_info = read_camera_info(f0)
        pose_frame_metadata = {
            "ee_pose_in_unified": read_pose_group_attrs(
                f0, "observations/ee_pose_in_unified"
            ),
            "camera_pose_in_unified": read_pose_group_attrs(
                f0, "observations/camera_pose_in_unified"
            ),
            "raw_driver_ee_pose_quat": read_pose_group_attrs(
                f0, "observations/ee_pose_quat"
            ),
            "raw_driver_ee_pose_rpy": read_pose_group_attrs(
                f0, "observations/ee_pose_rpy"
            ),
        }

    row_keys = [
        "timestamp", "frame_index", "episode_index", "index", "task_index",
        "observation.state", "observation.qvel", "observation.effort",
        "action", "base_action",
        "action.hybrid.left_ee_pose_in_unified.quat",
        "action.hybrid.right_ee_pose_in_unified.quat",
        "action.hybrid.mid_camera_pose_in_unified.quat",
        "action.hybrid.gripper",
    ]
    for cam in CAMERAS:
        row_keys.extend([
            f"observation.camera_pose_in_unified.quat.{cam}",
            f"observation.camera_pose_in_unified.rpy.{cam}",
            f"observation.camera_pose_in_unified.matrix.{cam}",
        ])
    for arm in ARMS:
        row_keys.extend([
            f"observation.ee_pose_quat.{arm}",
            f"observation.ee_pose_rpy.{arm}",
            f"observation.ee_pose_in_unified.quat.{arm}",
            f"observation.ee_pose_in_unified.rpy.{arm}",
            f"observation.ee_pose_in_unified.matrix.{arm}",
        ])
    data_rows = empty_rows(row_keys)

    for ep_idx, ep_path in enumerate(episodes):
        data_chunk_idx = data_file_index // CHUNK_SIZE
        data_file_idx = data_file_index % CHUNK_SIZE
        with h5py.File(ep_path, "r") as f:
            task = str(f.attrs.get("task_description", "")) or str(f.attrs.get("task_name", ep_path.parent.name))
            if task not in tasks:
                tasks[task] = len(tasks)
            task_idx = tasks[task]
            length = int(f["action"].shape[0])
            fps = int(f.attrs.get("frame_rate", fps))

            chunk_idx = ep_idx // CHUNK_SIZE
            file_idx = ep_idx % CHUNK_SIZE

            for cam in CAMERAS:
                vkey = f"observation.images.{cam}"
                vid_dir = out_root / "videos" / vkey / f"chunk-{chunk_idx:03d}"
                vid_dir.mkdir(parents=True, exist_ok=True)
                vid_path = vid_dir / f"file-{file_idx:03d}.mp4"
                if not vid_path.exists() or args.overwrite_videos:
                    vid_path.write_bytes(read_mp4_bytes(f[f"observations/images/{cam}"]))
                if ep_idx == 0:
                    info = probe_video(vid_path)
                    if info:
                        video_infos[cam] = {f"video.{k}": v for k, v in info.items()}

            qpos = f["observations/qpos"]
            qvel = f["observations/qvel"]
            effort = f["observations/effort"]
            action = f["action"]
            base_action = f["base_action"]
            ee_quat = {arm: f[f"observations/ee_pose_quat/{arm}"] for arm in ARMS}
            ee_rpy = {arm: f[f"observations/ee_pose_rpy/{arm}"] for arm in ARMS}
            ee_uq = {arm: f[f"observations/ee_pose_in_unified/{arm}/quat"] for arm in ARMS}
            ee_ur = {arm: f[f"observations/ee_pose_in_unified/{arm}/rpy"] for arm in ARMS}
            ee_um = {arm: f[f"observations/ee_pose_in_unified/{arm}/matrix"] for arm in ARMS}
            cam_uq = {
                cam: f[f"observations/camera_pose_in_unified/{cam}/quat"]
                for cam in CAMERAS
            }
            cam_ur = {
                cam: f[f"observations/camera_pose_in_unified/{cam}/rpy"]
                for cam in CAMERAS
            }
            cam_um = {
                cam: f[f"observations/camera_pose_in_unified/{cam}/matrix"]
                for cam in CAMERAS
            }

            for t in range(length):
                action_t = action[t]
                data_rows["timestamp"].append(np.float32(t / fps).item())
                data_rows["frame_index"].append(t)
                data_rows["episode_index"].append(ep_idx)
                data_rows["index"].append(global_index)
                data_rows["task_index"].append(task_idx)
                append_array(data_rows, "observation.state", qpos[t])
                append_array(data_rows, "observation.qvel", qvel[t])
                append_array(data_rows, "observation.effort", effort[t])
                append_array(data_rows, "action", action_t)
                append_array(data_rows, "base_action", base_action[t])
                append_array(
                    data_rows,
                    "action.hybrid.left_ee_pose_in_unified.quat",
                    ee_uq["left"][t],
                )
                append_array(
                    data_rows,
                    "action.hybrid.right_ee_pose_in_unified.quat",
                    ee_uq["right"][t],
                )
                append_array(
                    data_rows,
                    "action.hybrid.mid_camera_pose_in_unified.quat",
                    cam_uq["cam_front"][t],
                )
                append_array(
                    data_rows,
                    "action.hybrid.gripper",
                    [action_t[6], action_t[13]],
                )
                for cam in CAMERAS:
                    append_array(
                        data_rows,
                        f"observation.camera_pose_in_unified.quat.{cam}",
                        cam_uq[cam][t],
                    )
                    append_array(
                        data_rows,
                        f"observation.camera_pose_in_unified.rpy.{cam}",
                        cam_ur[cam][t],
                    )
                    append_array(
                        data_rows,
                        f"observation.camera_pose_in_unified.matrix.{cam}",
                        cam_um[cam][t],
                    )
                for arm in ARMS:
                    append_array(data_rows, f"observation.ee_pose_quat.{arm}", ee_quat[arm][t])
                    append_array(data_rows, f"observation.ee_pose_rpy.{arm}", ee_rpy[arm][t])
                    append_array(data_rows, f"observation.ee_pose_in_unified.quat.{arm}", ee_uq[arm][t])
                    append_array(data_rows, f"observation.ee_pose_in_unified.rpy.{arm}", ee_ur[arm][t])
                    append_array(data_rows, f"observation.ee_pose_in_unified.matrix.{arm}", ee_um[arm][t])
                global_index += 1

            ep_row = {
                "episode_index": ep_idx,
                "tasks": [task],
                "length": length,
                "dataset_from_index": total_frames,
                "dataset_to_index": total_frames + length,
                "data/chunk_index": data_chunk_idx,
                "data/file_index": data_file_idx,
            }
            for cam in CAMERAS:
                vkey = f"observation.images.{cam}"
                ep_row[f"videos/{vkey}/chunk_index"] = chunk_idx
                ep_row[f"videos/{vkey}/file_index"] = file_idx
            episode_rows.append(ep_row)
            total_frames += length
            episodes_in_data_file += 1

        if (ep_idx + 1) % 25 == 0 or ep_idx + 1 == len(episodes):
            print(f"[lerobot-convert] {ep_idx + 1}/{len(episodes)} episodes, frames={total_frames}")

        should_flush = (
            episodes_in_data_file >= args.data_episodes_per_file
            or ep_idx + 1 == len(episodes)
        )
        if should_flush:
            data_path = (
                out_root
                / "data"
                / f"chunk-{data_chunk_idx:03d}"
                / f"file-{data_file_idx:03d}.parquet"
            )
            write_data_parquet(data_rows, data_path)
            data_rows = empty_rows(row_keys)
            data_file_index += 1
            episodes_in_data_file = 0

    features = default_features(height, width, video_infos)
    features.update(base_meta_features())
    info = {
        "codebase_version": "v3.0",
        "fps": int(fps),
        "features": features,
        "total_episodes": len(episodes),
        "total_frames": int(total_frames),
        "total_tasks": len(tasks),
        "chunks_size": CHUNK_SIZE,
        "data_files_size_in_mb": 100000,
        "video_files_size_in_mb": 100000,
        "data_path": "data/chunk-{chunk_index:03d}/file-{file_index:03d}.parquet",
        "video_path": "videos/{video_key}/chunk-{chunk_index:03d}/file-{file_index:03d}.mp4",
        "robot_type": "cobot_piper_3arm",
        "action_convention": {
            "summary": HYBRID_TARGET_NOTE,
            "left": "action.hybrid.left_ee_pose_in_unified.quat",
            "right": "action.hybrid.right_ee_pose_in_unified.quat",
            "mid": "action.hybrid.mid_camera_pose_in_unified.quat",
            "gripper": "action.hybrid.gripper",
            "frame": "unified",
            "quaternion_order": "xyzw",
            "position_unit": "m",
        },
        "pose_frame_metadata": pose_frame_metadata,
        "splits": {"train": f"0:{len(episodes)}"},
    }
    meta = out_root / "meta"
    meta.mkdir(parents=True, exist_ok=True)
    (meta / "info.json").write_text(json.dumps(info, indent=2), encoding="utf-8")
    (meta / "camera_info.json").write_text(
        json.dumps(camera_info, indent=2), encoding="utf-8"
    )

    tasks_df = pd.DataFrame(
        {"task_index": list(tasks.values())},
        index=pd.Index(list(tasks.keys()), name="task"),
    )
    tasks_df.to_parquet(meta / "tasks.parquet")
    write_episodes_parquet(episode_rows, meta / "episodes" / "chunk-000" / "file-000.parquet")

    # Stats are optional for reading; write an empty placeholder to make the
    # dataset self-describing without spending another pass over ~1M frames.
    (meta / "stats.json").write_text("{}\n", encoding="utf-8")
    (out_root / "README.md").write_text(
        "# Cobot AVP Teleop LeRobot Dataset\n\n"
        "Converted from Piper-AVP-Teleop HDF5 episodes. Videos are extracted "
        "from lossless libx264rgb MP4 blobs stored in the original HDF5 files.\n\n"
        "Recommended action convention: left/right arms use unified-frame EEF "
        "pose, the mid arm uses unified-frame cam_front camera pose, and only "
        "left/right grippers are used. Camera intrinsics live in "
        "`meta/camera_info.json`; per-frame camera extrinsics live in "
        "`observation.camera_pose_in_unified.*`.\n",
        encoding="utf-8",
    )
    print(f"[lerobot-convert] DONE out={out_root}")
    print(f"[lerobot-convert] total_episodes={len(episodes)} total_frames={total_frames}")


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--src", required=True, help="Root containing task/episode_*.hdf5")
    parser.add_argument("--out", required=True, help="Output LeRobot dataset root")
    parser.add_argument("--fps", type=int, default=None)
    parser.add_argument("--max_episodes", type=int, default=None)
    parser.add_argument(
        "--data_episodes_per_file",
        type=int,
        default=50,
        help="Flush numeric rows every N episodes to keep memory bounded.",
    )
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--overwrite_videos", action="store_true")
    return parser.parse_args()


if __name__ == "__main__":
    convert(parse_args())
