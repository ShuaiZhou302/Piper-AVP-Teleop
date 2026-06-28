#!/usr/bin/env python3
"""
Play back a recorded 3-arm HDF5 episode through all 3 puppet arms.

For sanity-checking collected data: drives each /master/joint_<arm> topic
with the recorded action stream so the puppets retrace the demo.

Prereq: inference-mode launch (all 3 arms in mode=1, motors auto-enabled),
e.g. `roslaunch multi_arm_launch_tools/launch/start_ms_piper_3arm.launch
       mode:=1 auto_enable:=true`. The collect-mode launch puts left/right
in master-slave teach mode and WILL NOT accept ROS commands on those arms.

Workflow:
  1. roslaunch ... start_ms_piper_3arm.launch mode:=1 auto_enable:=true
  2. python play_data.py /path/to/episode_N.hdf5

What it does:
  - Loads HDF5, reads action[T, 21] (left[0:7] + right[7:14] + mid[14:21])
  - Reads current /puppet/joint_<arm> for each arm
  - Ramps mid arm to INITIAL_ARM_JOINTS (known safe pose) and left/right
    arms to action[0]'s corresponding slices, simultaneously, at a bounded
    speed (default 0.3 rad/s -- gentle and safe).
  - Asks ENTER, then replays actions[0..T-1] at the recorded frame_rate.
"""
import argparse
import os
import sys

import h5py
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


# Must mirror teleop/eef_avp_control_singlearm.py:INITIAL_ARM_JOINTS.
# Edit both together if you change one.
INITIAL_ARM_JOINTS = (0.0463, 0.5300, -0.5562, 0.000, 0.2419, 0.0000)
INITIAL_GRIPPER = 0.1

ARM_ORDER = ("left", "right", "mid")          # MUST match collect_data_3arm.py
DOF_PER_ARM = 7                               # 6 joints + gripper
JOINT_NAMES = ["joint0", "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]


def slice_for(arm_idx):
    return slice(arm_idx * DOF_PER_ARM, (arm_idx + 1) * DOF_PER_ARM)


class Player:
    def __init__(self, args):
        self.args = args
        rospy.init_node("play_data_3arm", anonymous=True)
        self.puppet = {a: None for a in ARM_ORDER}
        self.pubs = {}
        for a in ARM_ORDER:
            rospy.Subscriber(
                f"/puppet/joint_{a}", JointState, self._make_cb(a),
                queue_size=50, tcp_nodelay=True,
            )
            self.pubs[a] = rospy.Publisher(
                f"/master/joint_{a}", JointState, queue_size=10,
            )

    def _make_cb(self, arm):
        def cb(msg):
            self.puppet[arm] = msg
        return cb

    def wait_feedback(self, timeout_s=10.0):
        deadline = rospy.Time.now() + rospy.Duration(timeout_s)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            ready = all(
                self.puppet[a] is not None and len(self.puppet[a].position) >= 7
                for a in ARM_ORDER
            )
            if ready:
                return True
            if rospy.Time.now() > deadline:
                return False
            rate.sleep()
        return False

    def publish(self, arm, seven):
        m = JointState()
        m.header = Header(stamp=rospy.Time.now())
        m.name = JOINT_NAMES
        m.position = list(seven)
        self.pubs[arm].publish(m)

    def ramp(self, targets, speed_rad_s, hz=30.0):
        """Ramp all 3 arms simultaneously from current /puppet to targets[a] (7-vec)."""
        currents = {a: np.asarray(self.puppet[a].position[:7], dtype=float)
                    for a in ARM_ORDER}
        # Duration based on the largest joint delta across all 3 arms (gripper ignored).
        max_delta = 0.0
        for a in ARM_ORDER:
            d = float(np.max(np.abs(targets[a][:6] - currents[a][:6])))
            max_delta = max(max_delta, d)
        duration = max(max_delta / speed_rad_s, 1.0)
        steps = max(1, int(duration * hz))
        rate = rospy.Rate(hz)
        print(f"[play] ramping to start: max joint delta = {max_delta:.3f} rad, "
              f"duration = {duration:.2f}s (speed_cap={speed_rad_s} rad/s)")
        for i in range(steps):
            if rospy.is_shutdown():
                return False
            alpha = (i + 1) / steps
            for a in ARM_ORDER:
                interp = (1.0 - alpha) * currents[a] + alpha * targets[a]
                self.publish(a, interp)
            rate.sleep()
        # Hold final pose briefly so arms settle before playback starts.
        for _ in range(int(hz * 0.5)):
            if rospy.is_shutdown():
                return False
            for a in ARM_ORDER:
                self.publish(a, targets[a])
            rate.sleep()
        return True

    def play(self, actions, frame_rate):
        T = actions.shape[0]
        print(f"[play] replaying {T} frames at {frame_rate} Hz "
              f"({T / frame_rate:.1f}s)")
        rate = rospy.Rate(frame_rate)
        for i in range(T):
            if rospy.is_shutdown():
                return
            for arm_idx, a in enumerate(ARM_ORDER):
                self.publish(a, actions[i, slice_for(arm_idx)])
            if i % max(1, int(frame_rate)) == 0:
                print(f"[play]   frame {i:5d}/{T}")
            rate.sleep()
        print("[play] done.")

    def run(self):
        with h5py.File(self.args.episode, "r") as f:
            actions = f["action"][:]
            fr_attr = f.attrs.get("frame_rate", 30)
            try:
                fr_from_file = int(fr_attr)
            except Exception:
                fr_from_file = 30
            task_name = str(f.attrs.get("task_name", "?"))
            task_desc = str(f.attrs.get("task_description", "?"))
            arm_order_attr = str(f.attrs.get("arm_order", ",".join(ARM_ORDER)))

        fr = self.args.frame_rate if self.args.frame_rate else fr_from_file
        T, N = actions.shape
        print("=" * 60)
        print(f"[play] {self.args.episode}")
        print(f"  task        : {task_name}")
        print(f"  desc        : {task_desc}")
        print(f"  arm_order   : {arm_order_attr}  (script expects {','.join(ARM_ORDER)})")
        print(f"  shape       : action = ({T}, {N})")
        print(f"  frame_rate  : {fr} Hz {'(override)' if self.args.frame_rate else '(from HDF5)'}")
        print(f"  ramp speed  : {self.args.ramp_speed_rad_s} rad/s")
        print("=" * 60)

        if arm_order_attr != ",".join(ARM_ORDER):
            print(f"[play] WARN: arm_order in HDF5 = '{arm_order_attr}', "
                  f"but this script assumes '{','.join(ARM_ORDER)}'. "
                  "Joint slices will be wrong. Abort.")
            sys.exit(1)

        print("[play] waiting for /puppet/joint_<arm> feedback (10s)...")
        if not self.wait_feedback(10.0):
            print("[play] timed out. Is start_ms_piper_3arm.launch (mode=1) running?")
            sys.exit(1)

        # Build start-pose targets.
        # Mid: hardcoded INITIAL_ARM_JOINTS (known safe).
        # Left/Right: action[0]'s arm slice (matches the recorded start exactly).
        targets = {}
        for arm_idx, a in enumerate(ARM_ORDER):
            if a == "mid":
                targets[a] = np.array(
                    list(INITIAL_ARM_JOINTS) + [INITIAL_GRIPPER],
                    dtype=float,
                )
            else:
                targets[a] = np.asarray(actions[0, slice_for(arm_idx)], dtype=float)

        print()
        for a in ARM_ORDER:
            cur = np.asarray(self.puppet[a].position[:7], dtype=float)
            print(f"  {a:5s}: current = {cur.round(3).tolist()}")
            print(f"  {a:5s}: target  = {targets[a].round(3).tolist()}")

        if not self.args.no_confirm:
            input("\n[play] Make sure all 3 arms are clear of obstacles. "
                  "Press ENTER to RAMP TO START POSE.")

        if not self.ramp(targets, speed_rad_s=self.args.ramp_speed_rad_s):
            print("[play] ramp interrupted, abort.")
            return

        if not self.args.no_confirm:
            input("\n[play] All arms at start pose. Press ENTER to BEGIN PLAYBACK.")

        self.play(actions, frame_rate=fr)


def get_args():
    p = argparse.ArgumentParser(description="Replay a recorded 3-arm episode.")
    p.add_argument("episode", type=str, help="Path to episode_*.hdf5")
    p.add_argument("--frame_rate", type=int, default=None,
                   help="Override playback fps. Default: read from HDF5 attrs, "
                        "fallback 30.")
    p.add_argument("--ramp_speed_rad_s", type=float, default=0.3,
                   help="Max joint speed (rad/s) during the ramp-to-start phase. "
                        "Default 0.3 ~= 17 deg/s -- gentle/safe.")
    p.add_argument("--no_confirm", action="store_true",
                   help="Skip ENTER prompts. Only use after you've verified the setup.")
    return p.parse_args()


def main():
    args = get_args()
    if not os.path.isfile(args.episode):
        print(f"[play] no such file: {args.episode}")
        sys.exit(1)
    Player(args).run()


if __name__ == "__main__":
    main()
