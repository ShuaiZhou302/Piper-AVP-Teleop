#!/usr/bin/env python3
"""
Gesture state machine test (Step A of AVP -> Piper teleop plan).

Verifies pinch detection and IDLE / LOCKED / ENGAGED transitions in isolation,
before wiring the state machine into the actual arm controller.

Gestures (all rising-edge with Schmitt-trigger hysteresis):
  Left  thumb+middle pinch : IDLE    -> LOCKED
  Right thumb+middle pinch : LOCKED  -> ARMED    (1st pinch = arm, safety)
  Right thumb+middle pinch : ARMED   -> ENGAGED  (2nd pinch within timeout = confirm)
  (timeout)                : ARMED   -> LOCKED   (no confirm -> auto-cancel)
  Right thumb+middle pinch : ENGAGED -> LOCKED   (single pinch = pause/end)
Two-pinch arm-then-confirm gate makes accidental engage much less likely;
disengage is single-pinch on purpose so user can pause quickly.

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
    IDLE = "IDLE"          # not engaged (also the paused state between episodes)
    ENGAGED = "ENGAGED"
    DISARMED = "DISARMED"  # both-hand pinch from ENGAGED; HOLD to confirm pause


# How long the operator must HOLD BOTH pinches closed (continuously) inside
# DISARMED to confirm pause. Released earlier -> bounce back to ENGAGED.
# Kept long on purpose: during teleop the two hands can drift close together
# and momentarily read as a both-hand pinch, so the hold guards against that.
DISARM_HOLD_S = 4.0


class HandFreshness:
    """Detect whether WebXR hand tracking is currently valid.

    Vuer / WebXR keeps reporting the LAST landmark frame when a hand leaves
    the AVP camera FOV — so a "frozen" hand looks just like a held pose.
    If we feed those stale values into the pinch distance, a hand that left
    the view while pinching (or one never tracked, all zeros) will keep
    triggering. We detect freshness by:
      * all-zeros = never tracked yet
      * bitwise-identical to previous frame for N consecutive reads = frozen

    Real WebXR hand tracking always jitters at FP precision; exact bit
    equality across multiple reads basically means no updates arrived.
    """

    def __init__(self, stale_threshold=10):
        self._prev = None
        self._stale = 0
        self._threshold = stale_threshold

    def is_fresh(self, landmarks: np.ndarray) -> bool:
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


class GestureStateMachine:
    """Both-hand-pinch toggle driven by left+right thumb-middle pinch.

    Trigger gesture is BOTH hands pinching (thumb+middle) at the same time.
    A single-hand pinch never triggers anything, so the natural one-hand
    gripper/manipulation motion during teleop won't false-trigger a pause.

    IDLE     --[both-pinch rising edge]--------> ENGAGED   (lock head origin + record)
    ENGAGED  --[both-pinch rising edge]--------> DISARMED  (start hold timer)
    DISARMED --[both held DISARM_HOLD_S]-------> IDLE      (long-hold confirms pause)
    DISARMED --[either hand released]----------> ENGAGED   (released early -> cancel)

    Each pinch uses Schmitt-trigger hysteresis (PINCH_CLOSE / PINCH_OPEN) so a
    wobbling distance near the threshold doesn't cause spurious transitions.
    Initial _was_closed = True forces user to open then close on first use,
    avoiding accidental triggers if the program starts mid-pinch.
    """

    def __init__(self, disarm_hold_s: float = DISARM_HOLD_S):
        self.state = State.IDLE
        self._left_was_closed = True
        self._right_was_closed = True
        self._both_was_closed = True   # both-pinch combined edge tracker
        self._disarm_at = 0.0          # DISARMED entry timestamp (hold-to-confirm start)
        self.disarm_hold_s = disarm_hold_s

    @staticmethod
    def _phys_closed(was_closed: bool, dist: float) -> bool:
        # Hysteresis: stay in current state until we cross the OTHER threshold.
        if was_closed:
            return dist < PINCH_OPEN
        return dist < PINCH_CLOSE

    def time_left_disarm_hold(self) -> float:
        """Seconds remaining of HOLD required to confirm pause. 0 outside DISARMED."""
        if self.state is State.DISARMED:
            return max(0.0, self.disarm_hold_s - (time.monotonic() - self._disarm_at))
        return 0.0

    def update(self, left_pinch_dist: float, right_pinch_dist: float) -> State:
        now = time.monotonic()
        left_closed = self._phys_closed(self._left_was_closed, left_pinch_dist)
        right_closed = self._phys_closed(self._right_was_closed, right_pinch_dist)
        both_closed = left_closed and right_closed
        both_rising = both_closed and not self._both_was_closed

        if self.state is State.IDLE:
            if both_rising:
                self.state = State.ENGAGED
        elif self.state is State.ENGAGED:
            if both_rising:
                self.state = State.DISARMED
                self._disarm_at = now
        elif self.state is State.DISARMED:
            # Either hand released -> false trigger, bounce back to teleop.
            if not both_closed:
                self.state = State.ENGAGED
            # Both held closed long enough -> confirmed pause.
            elif (now - self._disarm_at) >= self.disarm_hold_s:
                self.state = State.IDLE

        self._left_was_closed = left_closed
        self._right_was_closed = right_closed
        self._both_was_closed = both_closed
        return self.state


# ---------- Visuals (only used when running this file directly) ----------

STATE_COLOR = {
    State.IDLE:     (180, 180, 180),
    State.ENGAGED:  (  0, 255,  80),
    State.DISARMED: (255, 130,   0),   # orange: about to pause
}

STATE_HINT = {
    State.IDLE:     ["Pinch BOTH hands", "to ENGAGE"],
    State.ENGAGED:  ["Pinch BOTH hands", "to start PAUSE"],
    State.DISARMED: ["HOLD BOTH 4s to PAUSE", "(release = cancel)"],
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
    l_fresh = HandFreshness()
    r_fresh = HandFreshness()
    last_state = None
    last_print = 0.0
    i = 0
    STALE_OPEN = 1.0  # fake "fully open" pinch distance when hand is stale

    try:
        while True:
            i += 1
            L = vr.left_landmarks    # (25, 3)
            R = vr.right_landmarks
            l_ok = l_fresh.is_fresh(L)
            r_ok = r_fresh.is_fresh(R)
            l_pinch = float(np.linalg.norm(L[THUMB] - L[MIDDLE])) if l_ok else STALE_OPEN
            r_pinch = float(np.linalg.norm(R[THUMB] - R[MIDDLE])) if r_ok else STALE_OPEN

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
            l_col = (255, 255, 255) if l_ok else (255, 110, 110)
            r_col = (255, 255, 255) if r_ok else (255, 110, 110)
            l_tag = "" if l_ok else "  (stale)"
            r_tag = "" if r_ok else "  (stale)"
            centered(f"L pinch = {l_pinch:.3f} m{l_tag}", 250, font, l_col)
            centered(f"R pinch = {r_pinch:.3f} m{r_tag}", 295, font, r_col)
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
