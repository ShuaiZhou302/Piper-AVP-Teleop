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

Reuses egox/data_collect/tele_vision.py (no duplication of Vuer plumbing).
"""
import os
import sys
import time
import numpy as np
from multiprocessing import shared_memory
from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
TELE_VISION_DIR = os.path.normpath(os.path.join(HERE, "..", "egox", "data_collect"))
sys.path.insert(0, TELE_VISION_DIR)

from tele_vision import OpenTeleVision  # noqa: E402

CERT = os.path.join(HERE, "cert.pem")
KEY = os.path.join(HERE, "key.pem")
SHM_NAME = "avp_min_test_shm"
IMG_SHAPE = (480, 640, 3)


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
            i += 1

            # Terminal log (newlines, 5 Hz)
            print(
                f"[{i:5d}] head=({x:+.3f},{y:+.3f},{z:+.3f}) "
                f"lwrist=({L[0,3]:+.2f},{L[1,3]:+.2f},{L[2,3]:+.2f}) "
                f"rwrist=({R[0,3]:+.2f},{R[1,3]:+.2f},{R[2,3]:+.2f})",
                flush=True,
            )

            # Render the same numbers into the in-VR background image
            canvas = Image.new("RGB", (IMG_SHAPE[1], IMG_SHAPE[0]), (20, 20, 30))
            draw = ImageDraw.Draw(canvas)
            draw.text((20, 30),  "AVP CONNECTED", fill=(0, 255, 0), font=font)
            draw.text((20, 100), f"head  x={x:+.3f}", fill=(255, 255, 255), font=font)
            draw.text((20, 150), f"      y={y:+.3f}", fill=(255, 255, 255), font=font)
            draw.text((20, 200), f"      z={z:+.3f}", fill=(255, 255, 255), font=font)
            draw.text((20, 280), f"frame #{i}", fill=(180, 180, 180), font=font)
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
