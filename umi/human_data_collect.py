#!/usr/bin/env python3
"""Collect human-side UMI data from AVP pose + external ROS camera.

This records the camera mounted near/on the AVP together with WebXR headset and
hand tracking. It does not command any robot.
"""
import argparse
import os
import sys
import time
from enum import Enum
from multiprocessing import shared_memory

import h5py
import numpy as np
from PIL import Image, ImageDraw, ImageFont


HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(HERE)
AVP_DIR = os.path.join(REPO_ROOT, "avp")
if AVP_DIR not in sys.path:
    sys.path.insert(0, AVP_DIR)

import rospy  # noqa: E402
from sensor_msgs.msg import CameraInfo, Image as ImageMsg  # noqa: E402


IMG_SHAPE = (480, 640, 3)
SHM_NAME = "umi_human_collect_shm"
THUMB = 4
MIDDLE = 14
PINCH_CLOSE = 0.03
PINCH_OPEN = 0.04


class State(Enum):
    IDLE = "IDLE"
    RECORDING = "RECORDING"
    STOPPING = "STOPPING"


class HandFreshness:
    """Conservative stale detector for WebXR hand landmarks.

    A stationary tracked hand normally still has tiny floating-point jitter. We
    only flag stale when landmarks are all-zero or bitwise-identical for a long
    run of samples. This is a heuristic, not an Apple-provided confidence score.
    """

    def __init__(self, stale_threshold=45):
        self._prev = None
        self._stale = 0
        self._threshold = stale_threshold
        self._all_zero = True

    def is_fresh(self, landmarks):
        if np.all(landmarks == 0):
            self._prev = None
            self._stale = self._threshold + 1
            self._all_zero = True
            return False
        self._all_zero = False
        if self._prev is not None and np.array_equal(landmarks, self._prev):
            self._stale += 1
        else:
            self._stale = 0
        self._prev = landmarks.copy()
        return self._stale < self._threshold

    @property
    def stale_count(self):
        return self._stale

    @property
    def all_zero(self):
        return self._all_zero


class RecordGesture:
    """Both-hand pinch hold to start/stop recording."""

    def __init__(self, start_hold_s=1.0, stop_hold_s=2.0):
        self.state = State.IDLE
        self.start_hold_s = float(start_hold_s)
        self.stop_hold_s = float(stop_hold_s)
        self._left_closed = True
        self._right_closed = True
        self._both_closed = True
        self._hold_started = None
        self._stopping_started = None

    @staticmethod
    def _closed(was_closed, dist):
        if was_closed:
            return dist < PINCH_OPEN
        return dist < PINCH_CLOSE

    def update(self, left_dist, right_dist):
        now = time.monotonic()
        left_closed = self._closed(self._left_closed, left_dist)
        right_closed = self._closed(self._right_closed, right_dist)
        both_closed = left_closed and right_closed
        both_rising = both_closed and not self._both_closed

        if self.state is State.IDLE:
            if both_rising:
                self._hold_started = now
            elif not both_closed:
                self._hold_started = None
            if (
                both_closed
                and self._hold_started is not None
                and now - self._hold_started >= self.start_hold_s
            ):
                self.state = State.RECORDING
                self._hold_started = None
        elif self.state is State.RECORDING:
            if both_rising:
                self.state = State.STOPPING
                self._stopping_started = now
        elif self.state is State.STOPPING:
            if not both_closed:
                self.state = State.RECORDING
                self._stopping_started = None
            elif now - self._stopping_started >= self.stop_hold_s:
                self.state = State.IDLE

        self._left_closed = left_closed
        self._right_closed = right_closed
        self._both_closed = both_closed
        return self.state


def load_open_television():
    old_argv = sys.argv[:]
    try:
        sys.argv = [sys.argv[0]]
        from tele_vision import OpenTeleVision  # noqa: WPS433
    finally:
        sys.argv = old_argv
    return OpenTeleVision


def make_shm(name):
    size = int(np.prod(IMG_SHAPE))
    try:
        shm = shared_memory.SharedMemory(create=True, size=size, name=name)
    except FileExistsError:
        old = shared_memory.SharedMemory(name=name)
        old.close()
        old.unlink()
        shm = shared_memory.SharedMemory(create=True, size=size, name=name)
    arr = np.ndarray(IMG_SHAPE, dtype=np.uint8, buffer=shm.buf)
    arr[:] = 0
    return shm


def valid_pose(T):
    T = np.asarray(T)
    return (
        T.shape == (4, 4)
        and np.isfinite(T).all()
        and not np.allclose(T, 0.0)
        and np.linalg.norm(T[:3, :3]) > 0.1
    )


def decode_ros_image(msg, target_shape=None):
    enc = msg.encoding.lower()
    channels = {
        "rgb8": 3,
        "bgr8": 3,
        "rgba8": 4,
        "bgra8": 4,
        "mono8": 1,
    }.get(enc)
    if channels is None:
        return None

    data = np.frombuffer(msg.data, dtype=np.uint8)
    rows = data.reshape(msg.height, msg.step)
    packed = rows[:, : msg.width * channels].reshape(msg.height, msg.width, channels)
    if enc == "rgb8":
        rgb = packed
    elif enc == "bgr8":
        rgb = packed[:, :, ::-1]
    elif enc == "rgba8":
        rgb = packed[:, :, :3]
    elif enc == "bgra8":
        rgb = packed[:, :, :3][:, :, ::-1]
    else:
        rgb = np.repeat(packed, 3, axis=2)

    if target_shape is not None and rgb.shape[:2] != tuple(target_shape):
        rgb = np.asarray(
            Image.fromarray(rgb).resize((target_shape[1], target_shape[0]), Image.BILINEAR)
        )
    return rgb.astype(np.uint8, copy=False)


def matrix_relative(origin, T):
    return np.linalg.inv(origin) @ T


def mkdir_parent(path):
    parent = os.path.dirname(os.path.abspath(path))
    if parent and not os.path.isdir(parent):
        os.makedirs(parent)


class HumanDataCollector:
    def __init__(self, args):
        self.args = args
        self.latest_image = None
        self.latest_image_stamp = 0.0
        self.latest_info = None
        self.origin_head = None
        self.last_engaged_count = 0
        self.rows = []

        self.shm = make_shm(args.shm_name)
        self.img_view = np.ndarray(IMG_SHAPE, dtype=np.uint8, buffer=self.shm.buf)
        OpenTeleVision = load_open_television()
        self.vr = OpenTeleVision(
            IMG_SHAPE[:2],
            args.shm_name,
            stereo=False,
            cert_file=args.cert_file,
            key_file=args.key_file,
            image_kwargs={
                "aspect": args.panel_aspect,
                "height": args.panel_height,
                "position": [0, args.panel_y, args.panel_z],
                "rotation": [0, 0, 0],
            },
            stream_fps=args.stream_fps,
            image_quality=args.image_quality,
        )

        rospy.init_node("umi_human_data_collect", anonymous=True)
        rospy.Subscriber(
            args.camera_topic,
            ImageMsg,
            self._image_cb,
            queue_size=1,
            buff_size=2 ** 24,
        )
        if args.camera_info_topic:
            rospy.Subscriber(args.camera_info_topic, CameraInfo, self._info_cb, queue_size=1)

        stale_threshold = max(3, int(round(args.stale_repeat_s * args.frame_rate)))
        self.gesture = RecordGesture(args.start_hold_s, args.stop_hold_s)
        self._l_fresh = HandFreshness(stale_threshold=stale_threshold)
        self._r_fresh = HandFreshness(stale_threshold=stale_threshold)

        try:
            self.font_big = ImageFont.truetype(
                "/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf", 36
            )
            self.font = ImageFont.truetype(
                "/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf", 22
            )
        except OSError:
            self.font_big = ImageFont.load_default()
            self.font = ImageFont.load_default()

    def _image_cb(self, msg):
        rgb = decode_ros_image(msg, target_shape=IMG_SHAPE[:2])
        if rgb is not None:
            self.latest_image = rgb.copy()
            self.latest_image_stamp = msg.header.stamp.to_sec()

    def _info_cb(self, msg):
        self.latest_info = msg

    def pinch_distances(self):
        ll = self.vr.left_landmarks
        rl = self.vr.right_landmarks
        l_ok = self._l_fresh.is_fresh(ll)
        r_ok = self._r_fresh.is_fresh(rl)
        stale_open = 1.0
        l_dist = float(np.linalg.norm(ll[THUMB] - ll[MIDDLE])) if l_ok else stale_open
        r_dist = float(np.linalg.norm(rl[THUMB] - rl[MIDDLE])) if r_ok else stale_open
        return {
            "left_pinch_dist": l_dist,
            "right_pinch_dist": r_dist,
            "left_hand_fresh": bool(l_ok),
            "right_hand_fresh": bool(r_ok),
            "left_stale_count": int(self._l_fresh.stale_count),
            "right_stale_count": int(self._r_fresh.stale_count),
            "left_all_zero": bool(self._l_fresh.all_zero),
            "right_all_zero": bool(self._r_fresh.all_zero),
        }

    def draw_hud(self, state, count, head_ok):
        if self.latest_image is None:
            canvas = Image.new("RGB", (IMG_SHAPE[1], IMG_SHAPE[0]), (18, 18, 24))
        else:
            canvas = Image.fromarray(self.latest_image.copy())
        draw = ImageDraw.Draw(canvas, "RGBA")
        draw.rectangle((0, 0, IMG_SHAPE[1], 46), fill=(0, 0, 0, 130))
        if state is State.RECORDING:
            text = f"REC {count:05d}"
            fill = (255, 80, 80)
        elif state is State.STOPPING:
            text = f"STOP? hold {count:05d}"
            fill = (255, 180, 80)
        else:
            text = "IDLE hold both pinch to record"
            fill = (225, 225, 225)
        if not head_ok:
            text += " | waiting pose"
        draw.text((14, 10), text, fill=fill, font=self.font)
        self.img_view[:] = np.asarray(canvas)

    def sample_row(self, tracking, state):
        head = self.vr.head_matrix
        if not valid_pose(head) or self.latest_image is None:
            return None
        if self.origin_head is None:
            self.origin_head = head.copy()
            print("[human-collect] head origin locked.", flush=True)
        return {
            "wall_time": time.time(),
            "image_stamp": float(self.latest_image_stamp),
            "image": self.latest_image.copy(),
            "head_matrix": head.copy(),
            "head_relative_matrix": matrix_relative(self.origin_head, head),
            "left_hand_matrix": self.vr.left_hand.copy(),
            "right_hand_matrix": self.vr.right_hand.copy(),
            "left_landmarks": self.vr.left_landmarks.copy(),
            "right_landmarks": self.vr.right_landmarks.copy(),
            "left_hand_fresh": tracking["left_hand_fresh"],
            "right_hand_fresh": tracking["right_hand_fresh"],
            "left_pinch_dist": tracking["left_pinch_dist"],
            "right_pinch_dist": tracking["right_pinch_dist"],
            "left_stale_count": tracking["left_stale_count"],
            "right_stale_count": tracking["right_stale_count"],
            "left_all_zero": tracking["left_all_zero"],
            "right_all_zero": tracking["right_all_zero"],
            "gesture_state": state.value,
        }

    def run(self):
        print("=" * 72)
        print("UMI human data collect")
        print(f"camera_topic = {self.args.camera_topic}")
        print(f"output       = {self.args.output}")
        print("On AVP Safari open the Vuer URL printed by OpenTeleVision and enter VR.")
        if self.args.start_immediately:
            print("start mode   = immediate")
            self.gesture.state = State.RECORDING
        else:
            print(
                f"start mode   = both pinch {self.args.start_hold_s:.1f}s to start, "
                f"{self.args.stop_hold_s:.1f}s to stop"
            )
        print("=" * 72)

        rate = rospy.Rate(self.args.frame_rate)
        last_print = 0.0
        while not rospy.is_shutdown():
            tracking = self.pinch_distances()
            prev = self.gesture.state
            state = self.gesture.update(
                tracking["left_pinch_dist"], tracking["right_pinch_dist"]
            )
            if self.args.start_immediately:
                state = self.gesture.state

            if prev is State.IDLE and state is State.RECORDING:
                self.origin_head = None
                self.rows = []
                self.last_engaged_count = 0
                print("[human-collect] RECORDING started.", flush=True)

            row = None
            if state in (State.RECORDING, State.STOPPING):
                row = self.sample_row(tracking, state)
                if row is not None:
                    self.rows.append(row)
                    if state is State.RECORDING:
                        self.last_engaged_count = len(self.rows)

            head_ok = valid_pose(self.vr.head_matrix)
            self.draw_hud(state, len(self.rows), head_ok)

            if len(self.rows) >= self.args.max_timesteps:
                print("[human-collect] max_timesteps reached.", flush=True)
                break
            if (
                self.args.duration_s > 0
                and len(self.rows) >= int(self.args.duration_s * self.args.frame_rate)
            ):
                print("[human-collect] duration reached.", flush=True)
                break
            if prev in (State.RECORDING, State.STOPPING) and state is State.IDLE:
                print("[human-collect] stop confirmed.", flush=True)
                break

            now = time.monotonic()
            if now - last_print > 1.0:
                print(
                    f"[human-collect] {state.value:<9} frames={len(self.rows)} "
                    f"L={tracking['left_pinch_dist']:.3f} "
                    f"R={tracking['right_pinch_dist']:.3f} "
                    f"fresh=({int(tracking['left_hand_fresh'])},"
                    f"{int(tracking['right_hand_fresh'])})",
                    flush=True,
                )
                last_print = now
            rate.sleep()

        self.trim_and_save()

    def trim_and_save(self):
        if self.last_engaged_count and self.last_engaged_count < len(self.rows):
            drop = len(self.rows) - self.last_engaged_count
            self.rows = self.rows[: self.last_engaged_count]
            print(f"[human-collect] trimmed {drop} STOPPING frames.", flush=True)
        if not self.rows:
            print("[human-collect] no frames recorded; nothing saved.", flush=True)
            return

        mkdir_parent(self.args.output)
        images = np.stack([r["image"] for r in self.rows], axis=0)
        head = np.stack([r["head_matrix"] for r in self.rows], axis=0)
        head_rel = np.stack([r["head_relative_matrix"] for r in self.rows], axis=0)
        left_hand = np.stack([r["left_hand_matrix"] for r in self.rows], axis=0)
        right_hand = np.stack([r["right_hand_matrix"] for r in self.rows], axis=0)
        left_lm = np.stack([r["left_landmarks"] for r in self.rows], axis=0)
        right_lm = np.stack([r["right_landmarks"] for r in self.rows], axis=0)
        left_fresh = np.asarray([r["left_hand_fresh"] for r in self.rows], dtype=np.bool_)
        right_fresh = np.asarray([r["right_hand_fresh"] for r in self.rows], dtype=np.bool_)
        pinch_dist = np.asarray(
            [[r["left_pinch_dist"], r["right_pinch_dist"]] for r in self.rows],
            dtype=float,
        )
        stale_count = np.asarray(
            [[r["left_stale_count"], r["right_stale_count"]] for r in self.rows],
            dtype=np.int32,
        )
        all_zero = np.asarray(
            [[r["left_all_zero"], r["right_all_zero"]] for r in self.rows],
            dtype=np.bool_,
        )
        state_map = {State.RECORDING.value: 1, State.STOPPING.value: 2}
        gesture_state = np.asarray(
            [state_map.get(r["gesture_state"], 0) for r in self.rows], dtype=np.int8
        )
        wall_time = np.asarray([r["wall_time"] for r in self.rows], dtype=float)
        image_stamp = np.asarray([r["image_stamp"] for r in self.rows], dtype=float)

        t0 = time.time()
        with h5py.File(self.args.output, "w") as f:
            f.attrs["schema"] = "umi_human_avp_v1"
            f.attrs["camera_topic"] = self.args.camera_topic
            f.attrs["frame_rate"] = float(self.args.frame_rate)
            f.attrs["pose_source"] = "Apple Vision Pro WebXR via Vuer"
            f.attrs["task_name"] = self.args.task_name
            f.attrs["task_description"] = self.args.task_description
            f.attrs["episode_idx"] = int(self.args.episode_idx)
            f.attrs["stale_repeat_s"] = float(self.args.stale_repeat_s)
            f.create_dataset("timestamp/wall_time", data=wall_time)
            f.create_dataset("timestamp/camera_ros_stamp", data=image_stamp)

            obs = f.create_group("observations")
            img_grp = obs.create_group("images")
            img_grp.create_dataset(
                "cam_front",
                data=images,
                chunks=(1, IMG_SHAPE[0], IMG_SHAPE[1], IMG_SHAPE[2]),
                compression=None if self.args.compression == "none" else self.args.compression,
            )
            avp = obs.create_group("avp")
            head_grp = avp.create_group("head")
            left_grp = avp.create_group("left_hand")
            right_grp = avp.create_group("right_hand")
            head_grp.create_dataset("matrix", data=head)
            head_grp.create_dataset("relative_matrix", data=head_rel)
            left_grp.create_dataset("matrix", data=left_hand)
            right_grp.create_dataset("matrix", data=right_hand)
            avp.create_dataset("left_landmarks", data=left_lm)
            avp.create_dataset("right_landmarks", data=right_lm)
            tracking_grp = avp.create_group("tracking")
            tracking_grp.create_dataset("left_hand_fresh", data=left_fresh)
            tracking_grp.create_dataset("right_hand_fresh", data=right_fresh)
            tracking_grp.create_dataset("pinch_distance", data=pinch_dist)
            tracking_grp.create_dataset("stale_count", data=stale_count)
            tracking_grp.create_dataset("all_zero", data=all_zero)
            tracking_grp.create_dataset("gesture_state", data=gesture_state)
            head_grp["matrix"].attrs["frame"] = "avp_world"
            head_grp["relative_matrix"].attrs["frame"] = "head_origin_at_record_start"
            avp["left_landmarks"].attrs["shape_per_timestep"] = "25x3"
            avp["right_landmarks"].attrs["shape_per_timestep"] = "25x3"
            tracking_grp["pinch_distance"].attrs["columns"] = "left,right"
            tracking_grp["stale_count"].attrs["columns"] = "left,right"
            tracking_grp["all_zero"].attrs["columns"] = "left,right"
            tracking_grp["gesture_state"].attrs["mapping"] = "0=IDLE,1=RECORDING,2=STOPPING"
            tracking_grp.attrs["freshness_source"] = (
                "Derived in collector: false for all-zero landmarks or repeated "
                "identical landmarks for >= stale_repeat_s seconds. This uses "
                "bitwise equality, not a low-motion threshold, so a real still "
                "hand should remain fresh if WebXR continues updating jitter. "
                "Vuer/WebXR does not currently provide an explicit confidence "
                "mask here."
            )

            if self.latest_info is not None:
                ci = f.create_group("camera_info/cam_front")
                ci.create_dataset("K", data=np.asarray(self.latest_info.K, dtype=float))
                ci.create_dataset("D", data=np.asarray(self.latest_info.D, dtype=float))
                ci.create_dataset("R", data=np.asarray(self.latest_info.R, dtype=float))
                ci.create_dataset("P", data=np.asarray(self.latest_info.P, dtype=float))
                ci.attrs["width"] = self.latest_info.width
                ci.attrs["height"] = self.latest_info.height
                ci.attrs["distortion_model"] = self.latest_info.distortion_model
        print(
            f"[human-collect] saved {len(self.rows)} frames to {self.args.output} "
            f"in {time.time() - t0:.1f}s",
            flush=True,
        )

    def close(self):
        try:
            self.shm.close()
            self.shm.unlink()
        except Exception:
            pass


def get_args():
    p = argparse.ArgumentParser(description="Collect AVP human hand/head pose + camera.")
    p.add_argument("--camera_topic", default="/camera_f/color/image_raw")
    p.add_argument("--camera_info_topic", default="/camera_f/color/camera_info")
    p.add_argument("--output", default="umi/human_data/episode_0.hdf5")
    p.add_argument("--task_name", default="umi_human")
    p.add_argument("--task_description", default="")
    p.add_argument("--episode_idx", type=int, default=0)
    p.add_argument("--frame_rate", type=float, default=30.0)
    p.add_argument("--max_timesteps", type=int, default=1500)
    p.add_argument("--duration_s", type=float, default=0.0)
    p.add_argument("--start_immediately", action="store_true")
    p.add_argument("--start_hold_s", type=float, default=1.0)
    p.add_argument("--stop_hold_s", type=float, default=2.0)
    p.add_argument("--stale_repeat_s", type=float, default=1.5)
    p.add_argument("--compression", default="none", choices=("none", "gzip", "lzf"))
    p.add_argument("--stream_fps", type=float, default=24.0)
    p.add_argument("--image_quality", type=int, default=30)
    p.add_argument("--panel_height", type=float, default=2.4)
    p.add_argument("--panel_aspect", type=float, default=1.6)
    p.add_argument("--panel_y", type=float, default=-0.25)
    p.add_argument("--panel_z", type=float, default=-2.0)
    p.add_argument("--shm_name", default=SHM_NAME)
    p.add_argument("--cert_file", default=os.path.join(AVP_DIR, "cert.pem"))
    p.add_argument("--key_file", default=os.path.join(AVP_DIR, "key.pem"))
    return p.parse_args()


def main():
    args = get_args()
    collector = HumanDataCollector(args)
    try:
        collector.run()
    except KeyboardInterrupt:
        print("\n[human-collect] interrupted; saving partial episode.")
        collector.trim_and_save()
    finally:
        collector.close()


if __name__ == "__main__":
    main()
