import sys
import types
import unittest


vuer = types.ModuleType("vuer")
vuer.Vuer = object
schemas = types.ModuleType("vuer.schemas")
schemas.ImageBackground = object
schemas.Hands = object
sys.modules.setdefault("vuer", vuer)
sys.modules.setdefault("vuer.schemas", schemas)
sys.path.insert(0, "avp")

import avp_gesture_test as gestures  # noqa: E402


class GestureStateMachineTest(unittest.TestCase):
    def setUp(self):
        self._real_monotonic = gestures.time.monotonic
        self.now = 100.0
        gestures.time.monotonic = lambda: self.now

    def tearDown(self):
        gestures.time.monotonic = self._real_monotonic

    def test_both_pinch_requires_two_second_hold_to_engage(self):
        fsm = gestures.GestureStateMachine()

        self.assertIs(fsm.update(1.0, 1.0), gestures.State.IDLE)
        self.now = 101.0
        self.assertIs(fsm.update(0.02, 0.02), gestures.State.IDLE)
        self.now = 102.9
        self.assertIs(fsm.update(0.02, 0.02), gestures.State.IDLE)
        self.now = 103.0
        self.assertIs(fsm.update(0.02, 0.02), gestures.State.ENGAGED)

    def test_releasing_before_start_hold_cancels_engage(self):
        fsm = gestures.GestureStateMachine()

        self.assertIs(fsm.update(1.0, 1.0), gestures.State.IDLE)
        self.now = 101.0
        self.assertIs(fsm.update(0.02, 0.02), gestures.State.IDLE)
        self.now = 102.0
        self.assertIs(fsm.update(1.0, 1.0), gestures.State.IDLE)
        self.now = 102.5
        self.assertIs(fsm.update(0.02, 0.02), gestures.State.IDLE)
        self.now = 104.4
        self.assertIs(fsm.update(0.02, 0.02), gestures.State.IDLE)
        self.now = 104.5
        self.assertIs(fsm.update(0.02, 0.02), gestures.State.ENGAGED)

    def test_disarm_still_requires_four_second_hold(self):
        fsm = gestures.GestureStateMachine()

        self.assertIs(fsm.update(1.0, 1.0), gestures.State.IDLE)
        self.now = 101.0
        self.assertIs(fsm.update(0.02, 0.02), gestures.State.IDLE)
        self.now = 103.0
        self.assertIs(fsm.update(0.02, 0.02), gestures.State.ENGAGED)
        self.now = 104.0
        self.assertIs(fsm.update(1.0, 1.0), gestures.State.ENGAGED)
        self.now = 105.0
        self.assertIs(fsm.update(0.02, 0.02), gestures.State.DISARMED)
        self.now = 108.9
        self.assertIs(fsm.update(0.02, 0.02), gestures.State.DISARMED)
        self.now = 109.0
        self.assertIs(fsm.update(0.02, 0.02), gestures.State.IDLE)


if __name__ == "__main__":
    unittest.main()
