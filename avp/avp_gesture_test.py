#!/usr/bin/env python3
"""
Gesture state machine test (Step A of AVP -> Piper teleop plan).

Verifies pinch detection and IDLE / LOCKED / ENGAGED transitions in isolation,
before wiring the state machine into the actual arm controller.

Gestures (all rising-edge with Schmitt-trigger hysteresis):
  Left  thumb+middle pinch : IDLE    -> LOCKED
  Right thumb+middle pinch : LOCKED  -> ENGAGED   (1st pinch = start)
  Right thumb+middle pinch : ENGAGED -> LOCKED    (2nd pinch = pause/end)

Run:
  conda activate aloha
  cd .../Piper-AVP-Teleop/avp
  python avp_gesture_test.py

Then in AVP Safari open the URL printed below and tap "Enter VR".
The HUD shows the current state in big letters; the terminal logs every
state change (and a heartbeat once per second otherwise).
"""
import os
import sys
import time
from enum import Enum

import numpy as np
from multiprocessing import shared_memory
from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from tele_vision import OpenTeleVision  # noqa: E402

CERT = os.path.join(HERE, "cert.pem")
KEY = os.path.join(HERE, "key.pem")
SHM_NAME = "avp_gesture_test_shm"
IMG_SHAPE = (480, 640, 3)

# Vuer hand landmark indices: 4 = thumb tip, 14 = middle finger tip.
THUMB = 4
MIDDLE = 14

# Pinch thresholds (meters). Hysteresis avoids chatter on release.
PINCH_CLOSE = 0.03
PINCH_OPEN = 0.04


class State(Enum):
    IDLE = "IDLE"
    LOCKED = "LOCKED"
    ENGAGED = "ENGAGED"


class GestureStateMachine:
    """Toggle-style state machine driven by left/right thumb-middle pinch.

    IDLE     --[L pinch rising edge]--> LOCKED
    LOCKED   --[R pinch rising edge]--> ENGAGED
    ENGAGED  --[R pinch rising edge]--> LOCKED  (toggle, NOT deadman)

    Both pinches use Schmitt-trigger hysteresis (PINCH_CLOSE / PINCH_OPEN) so
    a wobbling distance near the threshold doesn't cause spurious transitions.
    Initial _was_closed = True forces user to open then close on first use,
    avoiding accidental triggers if the program starts mid-pinch.
    """

    def __init__(self):
        self.state = State.IDLE
        self._left_was_closed = True
        self._right_was_closed = True

    @staticmethod
    def _phys_closed(was_closed: bool, dist: float) -> bool:
        # Hysteresis: stay in current state until we cross the OTHER threshold.
        if was_closed:
            return dist < PINCH_OPEN
        return dist < PINCH_CLOSE

    def update(self, left_pinch_dist: float, right_pinch_dist: float) -> State:
        left_closed = self._phys_closed(self._left_was_closed, left_pinch_dist)
        right_closed = self._phys_closed(self._right_was_closed, right_pinch_dist)
        left_rising = left_closed and not self._left_was_closed
        right_rising = right_closed and not self._right_was_closed

        if self.state is State.IDLE:
            if left_rising:
                self.state = State.LOCKED
        elif self.state is State.LOCKED:
            if right_rising:
                self.state = State.ENGAGED
        elif self.state is State.ENGAGED:
            if right_rising:
                self.state = State.LOCKED

        self._left_was_closed = left_closed
        self._right_was_closed = right_closed
        return self.state


# ---------- Visuals (only used when running this file directly) ----------

STATE_COLOR = {
    State.IDLE:    (180, 180, 180),
    State.LOCKED:  (255, 200,   0),
    State.ENGAGED: (  0, 255,  80),
}

STATE_HINT = {
    State.IDLE:    ["Pinch L thumb+middle", "to LOCK"],
    State.LOCKED:  ["Pinch R thumb+middle", "to ENGAGE"],
    State.ENGAGED: ["Pinch R thumb+middle", "again to PAUSE"],
}


def _make_shm():
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
    shm = _make_shm()
    vr = OpenTeleVision(
        IMG_SHAPE[:2], SHM_NAME, stereo=False, cert_file=CERT, key_file=KEY
    )

    print("=" * 60)
    print("Vuer server started on https://0.0.0.0:8012")
    print("On AVP Safari open:")
    print("    https://10.7.132.66:8012?ws=wss://10.7.132.66:8012")
    print("Then tap 'Enter VR'.")
    print("=" * 60)
    print("Gesture state machine running (Ctrl-C to stop).\n")

    img_view = np.ndarray(IMG_SHAPE, dtype=np.uint8, buffer=shm.buf)
    try:
        font_big = ImageFont.truetype(
            "/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf", 60
        )
        font = ImageFont.truetype(
            "/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf", 30
        )
    except OSError:
        font_big = ImageFont.load_default()
        font = ImageFont.load_default()

    fsm = GestureStateMachine()
    last_state = None
    last_print = 0.0
    i = 0

    try:
        while True:
            i += 1
            L = vr.left_landmarks    # (25, 3)
            R = vr.right_landmarks
            l_pinch = float(np.linalg.norm(L[THUMB] - L[MIDDLE]))
            r_pinch = float(np.linalg.norm(R[THUMB] - R[MIDDLE]))

            prev = fsm.state
            state = fsm.update(l_pinch, r_pinch)

            now = time.monotonic()
            transitioned = state != prev
            if transitioned or (now - last_print) > 1.0:
                tag = " <- TRANSITION" if transitioned else ""
                print(
                    f"[{i:5d}] {state.value:<8} "
                    f"L pinch={l_pinch:.3f}m  R pinch={r_pinch:.3f}m{tag}",
                    flush=True,
                )
                last_print = now
            last_state = state

            # HUD
            canvas = Image.new("RGB", (IMG_SHAPE[1], IMG_SHAPE[0]), (20, 20, 30))
            draw = ImageDraw.Draw(canvas)
            CW = IMG_SHAPE[1]

            def centered(text, y, fnt, fill):
                w = draw.textlength(text, font=fnt)
                draw.text(((CW - w) / 2, y), text, fill=fill, font=fnt)

            centered(state.value, 60, font_big, STATE_COLOR[state])
            for k, line in enumerate(STATE_HINT[state]):
                centered(line, 150 + k * 40, font, (200, 200, 200))
            centered(f"L pinch = {l_pinch:.3f} m", 250, font, (255, 255, 255))
            centered(f"R pinch = {r_pinch:.3f} m", 295, font, (255, 255, 255))
            centered(f"frame #{i}", 400, font, (140, 140, 140))
            img_view[:] = np.array(canvas)

            time.sleep(0.05)  # 20 Hz keeps the state machine snappy
    except KeyboardInterrupt:
        print("\n[gesture] stopping.")
    finally:
        shm.close()
        try:
            shm.unlink()
        except FileNotFoundError:
            pass


if __name__ == "__main__":
    main()
