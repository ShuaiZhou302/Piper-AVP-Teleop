#!/usr/bin/env python3
"""
Replay a 3-arm episode by tracking recorded EE poses through IK.

This is different from play_data.py:
  * Reads observations/ee_pose_rpy/{left,right,mid} from the HDF5.
  * Solves IK for each arm every frame, using the same PinocchioIKSolver path
    as teleop/eef_avp_control_singlearm.py.
  * Uses action's 7th value for each arm as the gripper command.

Default mode is dry-run: solve the whole episode offline and print IK stats.
Use --execute to publish /master/joint_<arm> commands to the robot.
"""
import argparse
import os
import sys

import h5py
import numpy as np


ARM_ORDER = ("left", "right", "mid")
DOF_PER_ARM = 7
JOINT_NAMES = ["joint0", "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
INITIAL_ARM_JOINTS = (0.0463, 0.5300, -0.5562, 0.0000, 0.6500, 0.0000)
INITIAL_GRIPPER = 0.1
DEFAULT_URDF = (
    "/home/agilex/cobot_magic/Piper_ros_private-ros-noetic/src/piper_description/urdf/"
    "piper_description_new.urdf"
)


def slice_for(arm_idx):
    return slice(arm_idx * DOF_PER_ARM, (arm_idx + 1) * DOF_PER_ARM)


def extract_grippers(action_row):
    return {
        arm: float(action_row[arm_idx * DOF_PER_ARM + 6])
        for arm_idx, arm in enumerate(ARM_ORDER)
    }


def clip_joint_step(target6, previous6, max_step):
    if max_step is None or max_step <= 0:
        return np.asarray(target6, dtype=float)
    target6 = np.asarray(target6, dtype=float)
    previous6 = np.asarray(previous6, dtype=float)
    return np.clip(target6, previous6 - max_step, previous6 + max_step)


def command_from_ik_result(sol6, ok, gripper, previous_command, max_joint_step):
    previous_command = np.asarray(previous_command, dtype=float)
    if ok and sol6 is not None:
        joints = clip_joint_step(sol6, previous_command[:6], max_joint_step)
    else:
        joints = previous_command[:6].copy()
    return np.asarray(list(joints) + [float(gripper)], dtype=float)


def joint6_pose_to_ik_ee_pose(pose6):
    """Convert driver /puppet/end_pose joint6 frame into teleop IK's ee frame."""
    from tf.transformations import euler_from_matrix, euler_matrix  # noqa: WPS433

    pose6 = np.asarray(pose6, dtype=float)
    out = pose6.copy()
    joint6_T = euler_matrix(pose6[3], pose6[4], pose6[5])
    ee_off_T = euler_matrix(0.0, -np.pi / 2.0, 0.0)
    ik_ee_T = joint6_T @ ee_off_T
    out[3:6] = euler_from_matrix(ik_ee_T)
    return out


def normalize_pose_for_ik(pose6, input_frame):
    if input_frame == "joint6":
        return joint6_pose_to_ik_ee_pose(pose6)
    if input_frame == "ik_ee":
        return np.asarray(pose6, dtype=float)
    raise ValueError(f"unknown input_frame: {input_frame}")


def load_episode(path, prefer_pose="rpy"):
    with h5py.File(path, "r") as f:
        action = f["action"][:]
        frame_rate = int(f.attrs.get("frame_rate", 30))
        task_name = str(f.attrs.get("task_name", "?"))
        task_desc = str(f.attrs.get("task_description", "?"))
        arm_order = str(f.attrs.get("arm_order", ",".join(ARM_ORDER)))
        if arm_order != ",".join(ARM_ORDER):
            raise ValueError(
                "HDF5 arm_order is %r, script expects %r"
                % (arm_order, ",".join(ARM_ORDER))
            )

        poses = {}
        if prefer_pose != "rpy":
            raise ValueError("Only --pose rpy is supported for now; IK takes xyz+rpy.")
        for arm in ARM_ORDER:
            key = f"observations/ee_pose_rpy/{arm}"
            if key not in f:
                raise KeyError(f"missing HDF5 dataset: {key}")
            poses[arm] = f[key][:]

    T = action.shape[0]
    if action.shape[1:] != (len(ARM_ORDER) * DOF_PER_ARM,):
        raise ValueError(f"action shape should be (T, 21), got {action.shape}")
    for arm in ARM_ORDER:
        if poses[arm].shape[0] != T or poses[arm].shape[1:] != (6,):
            raise ValueError(f"{arm} pose shape should be ({T}, 6), got {poses[arm].shape}")

    return {
        "action": action,
        "poses": poses,
        "frame_rate": frame_rate,
        "task_name": task_name,
        "task_description": task_desc,
        "arm_order": arm_order,
    }


def import_ik_solver():
    here = os.path.dirname(os.path.abspath(__file__))
    teleop_dir = os.path.normpath(os.path.join(here, "..", "teleop"))
    if teleop_dir not in sys.path:
        sys.path.insert(0, teleop_dir)
    from eef_keyboard_control_singlearm import PinocchioIKSolver  # noqa: WPS433
    return PinocchioIKSolver


def solve_ik(ik, xyz, rpy, gripper, seed, allow_collision=True):
    return ik.solve(
        xyz, rpy, gripper=gripper, motorstate=seed,
        allow_collision=allow_collision,
    )


class EefIkPlanner:
    def __init__(
        self,
        urdf,
        max_joint_step,
        input_frame="joint6",
        seed_mode="initial",
        allow_collision=True,
    ):
        PinocchioIKSolver = import_ik_solver()
        self.ik = {arm: PinocchioIKSolver(urdf) for arm in ARM_ORDER}
        self.max_joint_step = max_joint_step
        self.input_frame = input_frame
        self.seed_mode = seed_mode
        self.allow_collision = allow_collision

    def initial_commands(self, action0):
        commands = {}
        grippers = extract_grippers(action0)
        for arm in ARM_ORDER:
            commands[arm] = np.asarray(
                list(INITIAL_ARM_JOINTS) + [grippers.get(arm, INITIAL_GRIPPER)],
                dtype=float,
            )
        return commands

    def solve_frame_command(self, arm, pose6, gripper, previous_command, max_joint_step):
        pose6 = normalize_pose_for_ik(pose6, self.input_frame)
        xyz = pose6[:3]
        rpy = pose6[3:6]
        seed = previous_command[:6] if self.seed_mode in ("initial", "last") else None
        sol6, ok, _ = solve_ik(
            self.ik[arm],
            xyz,
            rpy,
            gripper,
            seed,
            allow_collision=self.allow_collision,
        )
        cmd = command_from_ik_result(
            sol6=sol6,
            ok=ok,
            gripper=gripper,
            previous_command=previous_command,
            max_joint_step=max_joint_step,
        )
        return cmd, ok

    def plan(self, episode):
        action = episode["action"]
        poses = episode["poses"]
        T = action.shape[0]
        commands = np.zeros((T, len(ARM_ORDER) * DOF_PER_ARM), dtype=np.float32)
        initial = self.initial_commands(action[0])
        previous = {}
        failures = {arm: 0 for arm in ARM_ORDER}
        max_step_seen = {arm: 0.0 for arm in ARM_ORDER}
        ramp_delta = {arm: 0.0 for arm in ARM_ORDER}
        first_failure = {arm: None for arm in ARM_ORDER}

        # First solve the true first-frame EEF target without per-frame clipping.
        # The robot will slowly ramp to this command before playback starts.
        grippers0 = extract_grippers(action[0])
        for arm_idx, arm in enumerate(ARM_ORDER):
            cmd, ok = self.solve_frame_command(
                arm,
                poses[arm][0],
                grippers0[arm],
                initial[arm],
                max_joint_step=0.0,
            )
            if not ok:
                failures[arm] += 1
                first_failure[arm] = 0
            ramp_delta[arm] = float(np.max(np.abs(cmd[:6] - initial[arm][:6])))
            commands[0, slice_for(arm_idx)] = cmd
            previous[arm] = cmd

        for t in range(1, T):
            grippers = extract_grippers(action[t])
            for arm_idx, arm in enumerate(ARM_ORDER):
                cmd, ok = self.solve_frame_command(
                    arm,
                    poses[arm][t],
                    grippers[arm],
                    previous[arm],
                    max_joint_step=self.max_joint_step,
                )
                if not ok:
                    failures[arm] += 1
                    if first_failure[arm] is None:
                        first_failure[arm] = t
                max_step_seen[arm] = max(
                    max_step_seen[arm],
                    float(np.max(np.abs(cmd[:6] - previous[arm][:6]))),
                )
                commands[t, slice_for(arm_idx)] = cmd
                previous[arm] = cmd

        return commands, {
            "frames": T,
            "failures": failures,
            "first_failure": first_failure,
            "max_step_seen": max_step_seen,
            "ramp_delta": ramp_delta,
        }


class Player:
    def __init__(self):
        import rospy  # noqa: WPS433
        from sensor_msgs.msg import JointState  # noqa: WPS433

        self.rospy = rospy
        self.JointState = JointState
        rospy.init_node("play_data_eef_ik_3arm", anonymous=True)
        self.puppet = {arm: None for arm in ARM_ORDER}
        self.pubs = {}
        for arm in ARM_ORDER:
            rospy.Subscriber(
                f"/puppet/joint_{arm}", JointState, self._make_cb(arm),
                queue_size=50, tcp_nodelay=True,
            )
            self.pubs[arm] = rospy.Publisher(
                f"/master/joint_{arm}", JointState, queue_size=10,
            )

    def _make_cb(self, arm):
        def cb(msg):
            self.puppet[arm] = msg
        return cb

    def wait_feedback(self, timeout_s):
        deadline = self.rospy.Time.now() + self.rospy.Duration(timeout_s)
        rate = self.rospy.Rate(20)
        while not self.rospy.is_shutdown():
            ready = all(
                self.puppet[arm] is not None and len(self.puppet[arm].position) >= 7
                for arm in ARM_ORDER
            )
            if ready:
                return True
            if self.rospy.Time.now() > deadline:
                return False
            rate.sleep()
        return False

    def publish(self, arm, seven):
        from std_msgs.msg import Header  # noqa: WPS433

        msg = self.JointState()
        msg.header = Header(stamp=self.rospy.Time.now())
        msg.name = JOINT_NAMES
        msg.position = list(np.asarray(seven, dtype=float))
        self.pubs[arm].publish(msg)

    def ramp(self, targets, speed_rad_s, hz):
        currents = {
            arm: np.asarray(self.puppet[arm].position[:7], dtype=float)
            for arm in ARM_ORDER
        }
        max_delta = max(
            float(np.max(np.abs(targets[arm][:6] - currents[arm][:6])))
            for arm in ARM_ORDER
        )
        duration = max(max_delta / speed_rad_s, 1.0)
        steps = max(1, int(duration * hz))
        rate = self.rospy.Rate(hz)
        print(
            f"[eef-play] ramp: max_delta={max_delta:.3f} rad, "
            f"duration={duration:.2f}s"
        )
        for i in range(steps):
            if self.rospy.is_shutdown():
                return False
            alpha = (i + 1) / steps
            for arm in ARM_ORDER:
                cmd = (1.0 - alpha) * currents[arm] + alpha * targets[arm]
                self.publish(arm, cmd)
            rate.sleep()
        return True

    def play(self, commands, frame_rate):
        rate = self.rospy.Rate(frame_rate)
        T = commands.shape[0]
        print(f"[eef-play] replaying {T} frames @ {frame_rate} Hz")
        for t in range(T):
            if self.rospy.is_shutdown():
                return
            for arm_idx, arm in enumerate(ARM_ORDER):
                self.publish(arm, commands[t, slice_for(arm_idx)])
            if t % max(1, int(frame_rate)) == 0:
                print(f"[eef-play] frame {t:5d}/{T}")
            rate.sleep()
        print("[eef-play] done.")


def print_summary(args, episode, stats):
    print("=" * 60)
    print(f"[eef-play] episode    : {args.episode}")
    print(f"[eef-play] task       : {episode['task_name']}")
    print(f"[eef-play] desc       : {episode['task_description']}")
    print(f"[eef-play] frames     : {stats['frames']}")
    print(f"[eef-play] frame_rate : {args.frame_rate or episode['frame_rate']} Hz")
    print(f"[eef-play] max_step   : {args.max_joint_step} rad/frame")
    print(f"[eef-play] mode       : {'EXECUTE' if args.execute else 'dry-run'}")
    for arm in ARM_ORDER:
        print(
            f"[eef-play] {arm:5s}: ik_fail={stats['failures'][arm]} "
            f"first_fail={stats['first_failure'][arm]} "
            f"ramp_delta={stats['ramp_delta'][arm]:.4f} "
            f"max_step={stats['max_step_seen'][arm]:.4f}"
        )
    print("=" * 60)


def get_args():
    parser = argparse.ArgumentParser(
        description="Replay recorded EE pose through IK for all 3 arms."
    )
    parser.add_argument("episode", type=str, help="Path to episode_*.hdf5")
    parser.add_argument("--urdf", type=str, default=DEFAULT_URDF)
    parser.add_argument("--frame_rate", type=int, default=None)
    parser.add_argument("--max_joint_step", type=float, default=0.05,
                        help="Per-frame joint step cap after IK. 0 disables clipping.")
    parser.add_argument("--input_frame", choices=("joint6", "ik_ee"), default="joint6",
                        help="/puppet/end_pose is joint6. Default converts it to the "
                             "teleop IK ee frame before solving.")
    parser.add_argument("--allow_collision", action="store_true",
                        help="Deprecated compatibility flag. Collision is ignored by default.")
    parser.add_argument("--respect_collision", action="store_true",
                        help="Reject IK solutions flagged as collision. Default is to ignore "
                             "the collision flag because the current model can false-positive "
                             "on real reachable Piper poses.")
    parser.add_argument("--ramp_speed_rad_s", type=float, default=0.25)
    parser.add_argument("--execute", action="store_true",
                        help="Actually publish /master/joint_<arm>. Default is dry-run.")
    parser.add_argument("--no_confirm", action="store_true",
                        help="Skip ENTER prompts when --execute is used.")
    parser.add_argument("--save_commands", type=str, default=None,
                        help="Optional .npy path to save planned joint commands.")
    return parser.parse_args()


def main():
    args = get_args()
    if not os.path.isfile(args.episode):
        raise SystemExit(f"[eef-play] no such file: {args.episode}")

    episode = load_episode(args.episode)
    planner = EefIkPlanner(
        args.urdf,
        max_joint_step=args.max_joint_step,
        input_frame=args.input_frame,
        allow_collision=not args.respect_collision,
    )
    commands, stats = planner.plan(episode)
    print_summary(args, episode, stats)

    if args.save_commands:
        np.save(args.save_commands, commands)
        print(f"[eef-play] saved commands: {args.save_commands}")

    if not args.execute:
        print("[eef-play] dry-run only. Add --execute to publish robot commands.")
        return

    frame_rate = args.frame_rate or episode["frame_rate"]
    player = Player()
    print("[eef-play] waiting for /puppet/joint_<arm> feedback...")
    if not player.wait_feedback(10.0):
        raise SystemExit("[eef-play] timed out waiting for puppet feedback.")

    start_targets = {
        arm: commands[0, slice_for(arm_idx)]
        for arm_idx, arm in enumerate(ARM_ORDER)
    }
    for arm in ARM_ORDER:
        current = np.asarray(player.puppet[arm].position[:7], dtype=float)
        print(f"  {arm:5s} current: {current.round(3).tolist()}")
        print(f"  {arm:5s} target : {start_targets[arm].round(3).tolist()}")

    if not args.no_confirm:
        input("\n[eef-play] Clear the workspace. Press ENTER to ramp to start.")
    if not player.ramp(start_targets, args.ramp_speed_rad_s, hz=frame_rate):
        return
    if not args.no_confirm:
        input("\n[eef-play] At start pose. Press ENTER to play EEF-IK trajectory.")
    player.play(commands, frame_rate=frame_rate)


if __name__ == "__main__":
    main()
