#!/usr/bin/env python3
"""UMI AVP viewer: external camera video + AVP head relative pose HUD.

This is intentionally read-only with respect to the robot. It subscribes to one
ROS camera topic, pushes that image into the existing Vuer/OpenTeleVision AVP
scene, and overlays the headset pose measured by WebXR.
"""
import argparse
import os
import socket
import sys
import time
from multiprocessing import shared_memory

import numpy as np
from PIL import Image, ImageDraw, ImageFont


HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(HERE)
AVP_DIR = os.path.join(REPO_ROOT, "avp")
if AVP_DIR not in sys.path:
    sys.path.insert(0, AVP_DIR)

import rospy  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402
from sensor_msgs.msg import Image as ImageMsg  # noqa: E402


IMG_SHAPE = (480, 640, 3)
SHM_NAME = "umi_avp_viewer_shm"
THUMB = 4
MIDDLE = 14
PINCH_CLOSE = 0.03

# AVP/WebXR world: +x right, +y up, +z back.
# UMI/Piper display frame: +x forward, +y left, +z up.
R_AVP_TO_UMI = np.array(
    [
        [0, 0, -1],
        [-1, 0, 0],
        [0, 1, 0],
    ],
    dtype=float,
)


class HandFreshness:
    """Detect stale WebXR hand landmarks when hands leave the AVP camera FOV."""

    def __init__(self, stale_threshold=10):
        self._prev = None
        self._stale = 0
        self._threshold = stale_threshold

    def is_fresh(self, landmarks):
        if np.all(landmarks == 0):
            self._prev = None
            self._stale = self._threshold + 1
            return False
        if self._prev is not None and np.array_equal(landmarks, self._prev):
            self._stale += 1
        else:
            self._stale = 0
        self._prev = landmarks.copy()
        return self._stale < self._threshold


def load_open_television():
    """Import Vuer after our argparse pass so params_proto cannot grab --help."""
    old_argv = sys.argv[:]
    try:
        sys.argv = [sys.argv[0]]
        from tele_vision import OpenTeleVision  # noqa: WPS433
    finally:
        sys.argv = old_argv
    return OpenTeleVision


def make_shm(name=SHM_NAME):
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


def guess_lan_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except OSError:
        return "<PC_LAN_IP>"


def valid_pose(T):
    T = np.asarray(T)
    return (
        T.shape == (4, 4)
        and np.isfinite(T).all()
        and not np.allclose(T, 0.0)
        and np.linalg.norm(T[:3, :3]) > 0.1
    )


def decode_ros_image(msg, target_shape=IMG_SHAPE[:2]):
    """Decode common ROS Image encodings into RGB uint8."""
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

    row_bytes = msg.step
    data = np.frombuffer(msg.data, dtype=np.uint8)
    rows = data.reshape(msg.height, row_bytes)
    packed = rows[:, : msg.width * channels].reshape(msg.height, msg.width, channels)

    if enc == "rgb8":
        rgb = packed
    elif enc == "bgr8":
        rgb = packed[:, :, ::-1]
    elif enc == "rgba8":
        rgb = packed[:, :, :3]
    elif enc == "bgra8":
        rgb = packed[:, :, :3][:, :, ::-1]
    else:  # mono8
        rgb = np.repeat(packed, 3, axis=2)

    if target_shape is not None and rgb.shape[:2] != tuple(target_shape):
        rgb = np.asarray(
            Image.fromarray(rgb).resize((target_shape[1], target_shape[0]), Image.BILINEAR)
        )
    return rgb.astype(np.uint8, copy=False)


def rpy_from_matrix_zyx(R):
    """Return roll, pitch, yaw for R = Rz(yaw) * Ry(pitch) * Rx(roll)."""
    sy = -float(np.clip(R[2, 0], -1.0, 1.0))
    pitch = np.arcsin(sy)
    if abs(sy) < 0.9999:
        roll = np.arctan2(R[2, 1], R[2, 2])
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll = 0.0
        yaw = np.arctan2(-R[0, 1], R[1, 1])
    return np.array([roll, pitch, yaw], dtype=float)


def quat_xyzw_from_matrix(R):
    """Convert a 3x3 rotation matrix to xyzw quaternion."""
    R = np.asarray(R, dtype=float)
    tr = float(np.trace(R))
    if tr > 0.0:
        s = np.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s
    else:
        i = int(np.argmax(np.diag(R)))
        if i == 0:
            s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
            qw = (R[2, 1] - R[1, 2]) / s
            qx = 0.25 * s
            qy = (R[0, 1] + R[1, 0]) / s
            qz = (R[0, 2] + R[2, 0]) / s
        elif i == 1:
            s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
            qw = (R[0, 2] - R[2, 0]) / s
            qx = (R[0, 1] + R[1, 0]) / s
            qy = 0.25 * s
            qz = (R[1, 2] + R[2, 1]) / s
        else:
            s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
            qw = (R[1, 0] - R[0, 1]) / s
            qx = (R[0, 2] + R[2, 0]) / s
            qy = (R[1, 2] + R[2, 1]) / s
            qz = 0.25 * s
    q = np.array([qx, qy, qz, qw], dtype=float)
    n = np.linalg.norm(q)
    return q / n if n > 0.0 else np.array([0.0, 0.0, 0.0, 1.0])


def fill_pose_msg(msg, stamp, frame_id, xyz, R):
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.pose.position.x = float(xyz[0])
    msg.pose.position.y = float(xyz[1])
    msg.pose.position.z = float(xyz[2])
    qx, qy, qz, qw = quat_xyzw_from_matrix(R)
    msg.pose.orientation.x = float(qx)
    msg.pose.orientation.y = float(qy)
    msg.pose.orientation.z = float(qz)
    msg.pose.orientation.w = float(qw)
    return msg


class UmiAvpViewer:
    def __init__(self, args):
        self.args = args
        self.latest_image = None
        self.latest_image_time = None
        self.origin = None
        self.last_reset_time = 0.0
        self.reset_started_at = None

        self.shm = make_shm(args.shm_name)
        self.img_view = np.ndarray(IMG_SHAPE, dtype=np.uint8, buffer=self.shm.buf)
        OpenTeleVision = load_open_television()
        image_kwargs = {}
        if args.view_mode == "panel":
            image_kwargs = {
                "aspect": args.panel_aspect,
                "height": args.panel_height,
                "position": [args.panel_x, args.panel_y, args.panel_z],
                "rotation": [0, 0, 0],
            }
        self.vr = OpenTeleVision(
            IMG_SHAPE[:2],
            args.shm_name,
            stereo=False,
            cert_file=args.cert_file,
            key_file=args.key_file,
            image_kwargs=image_kwargs,
            stream_fps=args.stream_fps,
            image_quality=args.image_quality,
        )

        rospy.init_node("umi_avp_viewer", anonymous=True)
        rospy.Subscriber(
            args.camera_topic,
            ImageMsg,
            self._image_cb,
            queue_size=1,
            buff_size=2 ** 24,
        )
        self.raw_pose_pub = rospy.Publisher(args.raw_pose_topic, PoseStamped, queue_size=10)
        self.rel_pose_pub = rospy.Publisher(args.relative_pose_topic, PoseStamped, queue_size=10)

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
        rgb = decode_ros_image(msg)
        if rgb is not None:
            self.latest_image = rgb
            self.latest_image_time = time.monotonic()

    def maybe_reset_origin_from_pinch(self, head):
        ll = self.vr.left_landmarks
        rl = self.vr.right_landmarks
        if not hasattr(self, "_l_fresh"):
            self._l_fresh = HandFreshness()
            self._r_fresh = HandFreshness()
        l_ok = self._l_fresh.is_fresh(ll)
        r_ok = self._r_fresh.is_fresh(rl)
        both = False
        if l_ok and r_ok:
            l_pinch = float(np.linalg.norm(ll[THUMB] - ll[MIDDLE]))
            r_pinch = float(np.linalg.norm(rl[THUMB] - rl[MIDDLE]))
            both = l_pinch < PINCH_CLOSE and r_pinch < PINCH_CLOSE

        now = time.monotonic()
        if both:
            if self.reset_started_at is None:
                self.reset_started_at = now
            elif (
                now - self.reset_started_at >= self.args.reset_hold_s
                and now - self.last_reset_time > 2.0
            ):
                self.origin = head.copy()
                self.last_reset_time = now
                self.reset_started_at = None
                print("[umi-avp] origin reset from both-hand pinch.", flush=True)
        else:
            self.reset_started_at = None

    def relative_pose(self, head):
        if self.origin is None:
            self.origin = head.copy()
            print("[umi-avp] origin locked from first valid AVP head pose.", flush=True)

        delta_pos_avp = head[:3, 3] - self.origin[:3, 3]
        delta_R_avp = head[:3, :3] @ self.origin[:3, :3].T
        delta_pos_umi = R_AVP_TO_UMI @ delta_pos_avp
        delta_R_umi = R_AVP_TO_UMI @ delta_R_avp @ R_AVP_TO_UMI.T
        rpy_umi = np.rad2deg(rpy_from_matrix_zyx(delta_R_umi))
        return delta_pos_avp, delta_R_avp, delta_pos_umi, delta_R_umi, rpy_umi

    def publish_poses(self, head, delta_pos_umi, delta_R_umi):
        stamp = rospy.Time.now()
        raw = fill_pose_msg(PoseStamped(), stamp, "avp_world", head[:3, 3], head[:3, :3])
        rel = fill_pose_msg(
            PoseStamped(), stamp, "umi_head_origin", delta_pos_umi, delta_R_umi
        )
        self.raw_pose_pub.publish(raw)
        self.rel_pose_pub.publish(rel)

    def draw(self, head_valid, rel=None):
        if self.latest_image is None:
            canvas = Image.new("RGB", (IMG_SHAPE[1], IMG_SHAPE[0]), (18, 18, 24))
        else:
            canvas = Image.fromarray(self.latest_image.copy())

        draw = ImageDraw.Draw(canvas, "RGBA")
        age = None
        if self.latest_image_time is not None:
            age = time.monotonic() - self.latest_image_time
        if self.args.hud == "full":
            draw.rectangle((0, 0, IMG_SHAPE[1], 176), fill=(0, 0, 0, 150))
            draw.text((18, 12), "UMI AVP VIEW", fill=(70, 255, 120), font=self.font_big)
            cam_status = "camera: waiting" if age is None else f"camera: {age * 1000:4.0f} ms ago"
            pose_status = "pose: OK" if head_valid else "pose: waiting for WebXR"
            draw.text((20, 62), cam_status, fill=(240, 240, 240), font=self.font)
            draw.text((20, 90), pose_status, fill=(240, 240, 240), font=self.font)
            if rel is not None:
                delta_pos_avp, _, delta_pos_umi, _, rpy_umi = rel
                lines = [
                    "AVP rel xyz   "
                    f"{delta_pos_avp[0]:+6.3f} {delta_pos_avp[1]:+6.3f} {delta_pos_avp[2]:+6.3f} m",
                    "UMI rel xyz   "
                    f"{delta_pos_umi[0]:+6.3f} {delta_pos_umi[1]:+6.3f} {delta_pos_umi[2]:+6.3f} m",
                    "UMI rpy deg   "
                    f"{rpy_umi[0]:+6.1f} {rpy_umi[1]:+6.1f} {rpy_umi[2]:+6.1f}",
                ]
                for i, line in enumerate(lines):
                    draw.text((20, 118 + i * 25), line, fill=(160, 215, 255), font=self.font)
            else:
                draw.text(
                    (20, 124),
                    "Open AVP Safari, enter VR, then first valid pose locks origin.",
                    fill=(210, 210, 210),
                    font=self.font,
                )
            draw.rectangle((0, IMG_SHAPE[0] - 40, IMG_SHAPE[1], IMG_SHAPE[0]), fill=(0, 0, 0, 145))
            draw.text(
                (18, IMG_SHAPE[0] - 32),
                f"source: {self.args.camera_topic}   hold both pinch {self.args.reset_hold_s:.1f}s = reset origin",
                fill=(220, 220, 220),
                font=self.font,
            )
        elif self.args.hud == "minimal":
            draw.rectangle((0, 0, IMG_SHAPE[1], 32), fill=(0, 0, 0, 120))
            if rel is not None:
                _, _, delta_pos_umi, _, rpy_umi = rel
                text = (
                    f"xyz {delta_pos_umi[0]:+.2f} {delta_pos_umi[1]:+.2f} {delta_pos_umi[2]:+.2f}m  "
                    f"rpy {rpy_umi[0]:+.0f} {rpy_umi[1]:+.0f} {rpy_umi[2]:+.0f}deg"
                )
                fill = (180, 235, 255)
            else:
                text = "waiting for AVP pose"
                fill = (235, 235, 235)
            draw.text((10, 5), text, fill=fill, font=self.font)
        self.img_view[:] = np.asarray(canvas)

    def run(self):
        ip = self.args.host_ip or guess_lan_ip()
        print("=" * 72)
        print("UMI AVP viewer")
        print(f"camera_topic       = {self.args.camera_topic}")
        print(f"view_mode          = {self.args.view_mode}")
        print(f"hud                = {self.args.hud}")
        print(f"stream_fps         = {self.args.stream_fps}")
        print(f"image_quality      = {self.args.image_quality}")
        if self.args.view_mode == "panel":
            print(
                "panel              = "
                f"height {self.args.panel_height:.2f} m, "
                f"pos [{self.args.panel_x:.2f}, {self.args.panel_y:.2f}, "
                f"{self.args.panel_z:.2f}]"
            )
        print(f"raw_pose_topic     = {self.args.raw_pose_topic}")
        print(f"relative_pose_topic= {self.args.relative_pose_topic}")
        print("On AVP Safari open:")
        print(f"    https://{ip}:8012?ws=wss://{ip}:8012")
        print("Then tap 'Enter VR'.")
        print("=" * 72)

        rate = rospy.Rate(self.args.hz)
        while not rospy.is_shutdown():
            head = self.vr.head_matrix
            head_ok = valid_pose(head)
            rel = None
            if head_ok:
                self.maybe_reset_origin_from_pinch(head)
                rel = self.relative_pose(head)
                self.publish_poses(head, rel[2], rel[3])
            self.draw(head_ok, rel=rel)
            rate.sleep()

    def close(self):
        try:
            self.shm.close()
            self.shm.unlink()
        except Exception:
            pass


def get_args():
    p = argparse.ArgumentParser(
        description="Show an external camera feed in AVP and overlay AVP relative pose."
    )
    p.add_argument("--camera_topic", default="/camera_f/color/image_raw")
    p.add_argument("--raw_pose_topic", default="/umi/avp/head_pose")
    p.add_argument("--relative_pose_topic", default="/umi/avp/head_relative_pose")
    p.add_argument("--hz", type=float, default=30.0)
    p.add_argument("--stream_fps", type=float, default=30.0)
    p.add_argument("--image_quality", type=int, default=35)
    p.add_argument("--reset_hold_s", type=float, default=1.0)
    p.add_argument(
        "--view_mode",
        choices=("panel", "full"),
        default="panel",
        help="panel keeps the camera feed small; full uses the old large background.",
    )
    p.add_argument("--hud", choices=("minimal", "full", "none"), default="minimal")
    p.add_argument("--panel_height", type=float, default=2.4)
    p.add_argument("--panel_aspect", type=float, default=1.6)
    p.add_argument("--panel_x", type=float, default=0.0)
    p.add_argument("--panel_y", type=float, default=-0.25)
    p.add_argument("--panel_z", type=float, default=-2.0)
    p.add_argument("--host_ip", default=None, help="IP printed in the AVP Safari URL.")
    p.add_argument("--shm_name", default=SHM_NAME)
    p.add_argument("--cert_file", default=os.path.join(AVP_DIR, "cert.pem"))
    p.add_argument("--key_file", default=os.path.join(AVP_DIR, "key.pem"))
    return p.parse_args()


def main():
    args = get_args()
    viewer = UmiAvpViewer(args)
    try:
        viewer.run()
    except KeyboardInterrupt:
        print("\n[umi-avp] stopping.")
    finally:
        viewer.close()


if __name__ == "__main__":
    main()
