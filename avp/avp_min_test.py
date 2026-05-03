#!/usr/bin/env python3
"""
Minimal AVP head-pose reader via Vuer (WebXR).

Run:
  conda activate aloha
  cd .../Piper-AVP-Teleop/avp
  python avp_min_test.py

Then on AVP, open Safari and go to:
  https://<PC_IP>:8012?ws=wss://<PC_IP>:8012
Tap "Enter VR".  Move your head; this script prints head_matrix.

Uses the local tele_vision.py (Vuer/WebXR wrapper) sitting next to this file.
"""
import os
import sys
import time
import numpy as np
from multiprocessing import shared_memory
from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from tele_vision import OpenTeleVision  # noqa: E402

CERT = os.path.join(HERE, "cert.pem")
KEY = os.path.join(HERE, "key.pem")
SHM_NAME = "avp_min_test_shm"
IMG_SHAPE = (480, 640, 3)


def mat_to_yaw_pitch_roll_yup(R):
    """Decompose a 3x3 rotation as YXZ intrinsic Euler angles for a y-up world.

    yaw   : about world y-axis (turn head left/right)
    pitch : about local x-axis (nod up/down)
    roll  : about local z-axis (tilt head sideways)

    Returns (yaw, pitch, roll) in radians.
    """
    pitch = np.arcsin(-np.clip(R[1, 2], -1.0, 1.0))
    if abs(R[1, 2]) < 0.9999:  # not at gimbal lock
        yaw = np.arctan2(R[0, 2], R[2, 2])
        roll = np.arctan2(R[1, 0], R[1, 1])
    else:
        yaw = np.arctan2(-R[2, 0], R[0, 0])
        roll = 0.0
    return yaw, pitch, roll


def make_shm():
    size = int(np.prod(IMG_SHAPE))
    try:
        shm = shared_memory.SharedMemory(create=True, size=size, name=SHM_NAME)
    except FileExistsError:
        old = shared_memory.SharedMemory(name=SHM_NAME)
        old.close()
        old.unlink()
        shm = shared_memory.SharedMemory(create=True, size=size, name=SHM_NAME)
    img = np.ndarray(IMG_SHAPE, dtype=np.uint8, buffer=shm.buf)
    img[:] = 0
    return shm


def main():
    assert os.path.isfile(CERT) and os.path.isfile(KEY), (
        f"cert/key not found in {HERE}; run mkcert first."
    )
    shm = make_shm()
    vr = OpenTeleVision(
        IMG_SHAPE[:2], SHM_NAME, stereo=False, cert_file=CERT, key_file=KEY
    )

    print("=" * 60)
    print("Vuer server started on https://0.0.0.0:8012")
    print("On AVP Safari open:")
    print("    https://10.7.132.66:8012?ws=wss://10.7.132.66:8012")
    print("Then tap 'Enter VR'.")
    print("=" * 60)
    print("Reading head_matrix (Ctrl-C to stop)...\n")

    img_view = np.ndarray(IMG_SHAPE, dtype=np.uint8, buffer=shm.buf)
    try:
        font = ImageFont.truetype(
            "/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf", 36
        )
    except OSError:
        font = ImageFont.load_default()

    i = 0
    try:
        while True:
            H = vr.head_matrix
            L = vr.left_hand
            R = vr.right_hand
            x, y, z = H[:3, 3]
            yaw, pitch, roll = mat_to_yaw_pitch_roll_yup(H[:3, :3])
            yaw_d, pitch_d, roll_d = np.rad2deg([yaw, pitch, roll])
            i += 1

            # Terminal log (newlines, 5 Hz)
            print(
                f"[{i:5d}] head xyz=({x:+.3f},{y:+.3f},{z:+.3f}) "
                f"rpy=({roll_d:+6.1f},{pitch_d:+6.1f},{yaw_d:+6.1f})deg  "
                f"lw=({L[0,3]:+.2f},{L[1,3]:+.2f},{L[2,3]:+.2f}) "
                f"rw=({R[0,3]:+.2f},{R[1,3]:+.2f},{R[2,3]:+.2f})",
                flush=True,
            )

            # Render the same numbers into the in-VR background image (centered)
            canvas = Image.new("RGB", (IMG_SHAPE[1], IMG_SHAPE[0]), (20, 20, 30))
            draw = ImageDraw.Draw(canvas)
            CW = IMG_SHAPE[1]

            def draw_centered(text, y_pos, fill):
                w = draw.textlength(text, font=font)
                draw.text(((CW - w) / 2, y_pos), text, fill=fill, font=font)

            def draw_block(lines, y_start, line_h, fill):
                # Group-center: all lines share the x of the widest line so digits align.
                max_w = max(draw.textlength(t, font=font) for t in lines)
                x_left = (CW - max_w) / 2
                for k, t in enumerate(lines):
                    draw.text((x_left, y_start + k * line_h), t, fill=fill, font=font)

            draw_centered("AVP CONNECTED", 40, (0, 255, 0))
            draw_block(
                [f"x = {x:+.3f} m", f"y = {y:+.3f} m", f"z = {z:+.3f} m"],
                y_start=110, line_h=42, fill=(255, 255, 255),
            )
            draw_block(
                [
                    f"yaw   = {yaw_d:+6.1f} deg",
                    f"pitch = {pitch_d:+6.1f} deg",
                    f"roll  = {roll_d:+6.1f} deg",
                ],
                y_start=250, line_h=42, fill=(140, 200, 255),
            )
            draw_centered(f"frame #{i}", 400, (180, 180, 180))
            img_view[:] = np.array(canvas)

            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\n[avp] stopping.")
    finally:
        shm.close()
        try:
            shm.unlink()
        except FileNotFoundError:
            pass


if __name__ == "__main__":
    main()
