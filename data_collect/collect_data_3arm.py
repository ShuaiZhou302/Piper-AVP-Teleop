#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
"""
3-arm data collection driven by AVP teleop gesture state.

Differences vs collect_data_shuai.py (cobot_magic/collect_data):
  * Records LEFT + RIGHT + MID arms (21 joint dims, 21 master/action dims).
  * Adds /puppet/end_pose_<arm> for each arm — stored both as quaternion (7d)
    and as RPY (6d).
  * Episode lifecycle is driven by /teleop/state (published by
    eef_avp_control_singlearm.py), NOT keyboard input:
       wait until state == ENGAGED  -> start recording
       on ENGAGED -> IDLE           -> stop, save, exit
       on max_timesteps reached     -> stop, save, exit
  * task_name / task_description / dataset_dir come from CLI (set in .sh),
    no per-episode prompts.

HDF5 layout (action/observation 3-arm order: LEFT, RIGHT, MID):
  observations/
    qpos                : (T, 21)   left puppet 7 + right 7 + mid 7
    qvel                : (T, 21)
    effort              : (T, 21)
    ee_pose_quat/{left,right,mid} : (T, 7)   x, y, z, qx, qy, qz, qw
    ee_pose_rpy/{left,right,mid}  : (T, 6)   x, y, z, roll, pitch, yaw
    images/{cam_front,cam_left,cam_right} : (T, 480, 640, 3) uint8
  action                : (T, 21)   left master 7 + right 7 + mid 7
  base_action           : (T, 2)    (linear x, angular z), recorded for compatibility
  camera_info/{cam_front,cam_left,cam_right} : static intrinsics (one copy, not per-frame)
    K(9) intrinsic 3x3, D distortion, R(9) rectification, P(12) projection;
    width/height/distortion_model as group attrs
"""
import argparse
import collections
import os
import sys
import time
from collections import deque

import cv2
import dm_env
import h5py
import numpy as np

# IMPORTANT: import conda pinocchio before rospy/tf. rospy/tf can load the
# system libstdc++; if that happens first, conda pinocchio/casadi may fail with
# missing GLIBCXX_3.4.29 during camera-pose post-processing.
import pinocchio as _pinocchio_libstdcpp_guard  # noqa: F401
import rospy
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, JointState, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion


ARM_ORDER = ("left", "right", "mid")  # canonical concatenation order
CAM_ORDER = ("cam_front", "cam_left", "cam_right")
# Which arm each camera is wrist-mounted on (mid arm carries the front cam).
CAM_TO_ARM = {"cam_front": "mid", "cam_left": "left", "cam_right": "right"}
DOF_PER_ARM = 7  # 6 joints + gripper
EE_QUAT_DIM = 7
EE_RPY_DIM = 6
IMG_H, IMG_W = 480, 640


def camera_fov_from_intrinsics(K, width, height):
    """Return (horizontal_fov_rad, vertical_fov_rad) from pinhole intrinsics."""
    K = np.asarray(K, dtype=float).reshape(-1)
    fx = float(K[0])
    fy = float(K[4])
    if fx <= 0.0 or fy <= 0.0 or width <= 0 or height <= 0:
        raise ValueError(f"bad camera intrinsics/fov inputs: fx={fx}, fy={fy}, "
                         f"width={width}, height={height}")
    return (
        float(2.0 * np.arctan(float(width) / (2.0 * fx))),
        float(2.0 * np.arctan(float(height) / (2.0 * fy))),
    )


def write_camera_fov_attrs(root):
    """Write hfov/vfov attrs under camera_info/<cam> when intrinsics exist."""
    if "camera_info" not in root:
        return
    ci_grp = root["camera_info"]
    for cam in CAM_ORDER:
        if cam not in ci_grp or "K" not in ci_grp[cam]:
            continue
        g = ci_grp[cam]
        width = int(g.attrs.get("width", 0))
        height = int(g.attrs.get("height", 0))
        if width <= 0:
            width = IMG_W
        if height <= 0:
            height = IMG_H
        try:
            hfov, vfov = camera_fov_from_intrinsics(g["K"][...], width, height)
        except Exception as e:
            print(f"\033[33m[postprocess] WARN: FOV skipped for {cam}: {e}\033[0m")
            continue
        g.attrs["horizontal_fov_rad"] = hfov
        g.attrs["vertical_fov_rad"] = vfov
        g.attrs["horizontal_fov_deg"] = float(np.degrees(hfov))
        g.attrs["vertical_fov_deg"] = float(np.degrees(vfov))


def _compute_camera_poses_in_unified(qpos_array):
    """Post-process step: per-frame camera pose in the unified arm frame.

    Args:
      qpos_array: (T, 21) float, layout = left[0:7] right[7:14] mid[14:21].

    Returns:
      (poses_per_cam, summary) where poses_per_cam maps cam_name -> dict with:
        "quat":   (T, 7)  [x, y, z, qx, qy, qz, qw]
        "rpy":    (T, 6)  [x, y, z, roll, pitch, yaw]
        "matrix": (T, 4, 4)
      summary is provenance info (calibration files used) for HDF5 attrs.

      Returns (None, None) if calibration is missing — collection still
      succeeds without the derived dataset.
    """
    # Import lazily so a missing calibration / pinocchio install doesn't break
    # the data-collection process itself (we'd rather save without the derived
    # field than abort the episode).
    try:
        teleop_dir = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "teleop"
        )
        if teleop_dir not in sys.path:
            sys.path.insert(0, teleop_dir)
        from arm_camera_in_unified import (  # noqa: E402
            WristCameraUnifiedConverter, matrix_to_quat_xyz, matrix_to_rpy_xyz,
        )
    except Exception as e:
        print(f"\033[33m[postprocess] WARN: camera_pose_in_unified DISABLED "
              f"(import failed: {e})\033[0m")
        return None, None

    try:
        conv = WristCameraUnifiedConverter()
    except Exception as e:
        print(f"\033[33m[postprocess] WARN: camera_pose_in_unified DISABLED "
              f"(converter init failed: {e})\033[0m")
        return None, None

    arr = np.asarray(qpos_array, dtype=float)
    T = arr.shape[0]
    # left=[0:7], right=[7:14], mid=[14:21] -> ARM_ORDER index map
    arm_slice = {"left": slice(0, 7), "right": slice(7, 14), "mid": slice(14, 21)}

    poses_per_cam = {}
    for cam, arm in CAM_TO_ARM.items():
        joint_arm = arr[:, arm_slice[arm]]  # (T, 7), only first 6 used by FK
        Ts = conv.camera_poses_in_unified_batch(arm, joint_arm)  # (T, 4, 4)
        quats = np.empty((T, 7), dtype=float)
        rpys = np.empty((T, 6), dtype=float)
        for t in range(T):
            quats[t] = matrix_to_quat_xyz(Ts[t])
            rpys[t] = matrix_to_rpy_xyz(Ts[t])
        poses_per_cam[cam] = {"quat": quats, "rpy": rpys, "matrix": Ts}

    summary = conv.summary()
    return poses_per_cam, summary


def _compute_eef_poses_in_unified(qpos_array):
    """Post-process per-frame teleop IK EE pose in the unified arm frame."""
    try:
        teleop_dir = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "teleop"
        )
        if teleop_dir not in sys.path:
            sys.path.insert(0, teleop_dir)
        from unified_eef_pose import (  # noqa: E402
            UnifiedEefConverter, matrix_to_quat_xyz, matrix_to_rpy_xyz,
        )
    except Exception as e:
        print(f"\033[33m[postprocess] WARN: ee_pose_in_unified DISABLED "
              f"(import failed: {e})\033[0m")
        return None, None

    try:
        conv = UnifiedEefConverter()
    except Exception as e:
        print(f"\033[33m[postprocess] WARN: ee_pose_in_unified DISABLED "
              f"(converter init failed: {e})\033[0m")
        return None, None

    arr = np.asarray(qpos_array, dtype=float)
    T = arr.shape[0]
    arm_slice = {"left": slice(0, 7), "right": slice(7, 14), "mid": slice(14, 21)}

    poses_per_arm = {}
    for arm in ARM_ORDER:
        Ts = conv.eef_poses_in_unified_batch(arm, arr[:, arm_slice[arm]])
        quats = np.empty((T, 7), dtype=float)
        rpys = np.empty((T, 6), dtype=float)
        for t in range(T):
            quats[t] = matrix_to_quat_xyz(Ts[t])
            rpys[t] = matrix_to_rpy_xyz(Ts[t])
        poses_per_arm[arm] = {"quat": quats, "rpy": rpys, "matrix": Ts}

    return poses_per_arm, conv.summary()


def write_camera_poses_in_unified(root, cam_poses, cam_pose_summary, overwrite=True):
    """Write observations/camera_pose_in_unified into an open HDF5 root."""
    obs_grp = root["observations"]
    group_name = "camera_pose_in_unified"
    if group_name in obs_grp:
        if not overwrite:
            raise RuntimeError(f"observations/{group_name} already exists")
        del obs_grp[group_name]

    cp_grp = obs_grp.create_group(group_name)
    cp_grp.attrs["frame"] = "unified (mid_base orientation, origin 25cm below mid_base z)"
    if cam_pose_summary:
        cp_grp.attrs["extrinsics_json"] = cam_pose_summary["extrinsics_json"]
        cp_grp.attrs["urdf"] = cam_pose_summary["urdf"]
        cp_grp.attrs["unified_origin_in_mid_m"] = \
            np.asarray(cam_pose_summary["unified_origin_in_mid_m"])
        for arm, src in cam_pose_summary["handeye_per_arm"].items():
            cp_grp.attrs[f"handeye_{arm}"] = src
    for cam in CAM_ORDER:
        g = cp_grp.create_group(cam)
        g.attrs["wrist_arm"] = CAM_TO_ARM[cam]
        g.create_dataset("quat",   data=cam_poses[cam]["quat"])
        g.create_dataset("rpy",    data=cam_poses[cam]["rpy"])
        g.create_dataset("matrix", data=cam_poses[cam]["matrix"])


def write_eef_poses_in_unified(root, eef_poses, eef_pose_summary, overwrite=True):
    """Write observations/ee_pose_in_unified into an open HDF5 root."""
    obs_grp = root["observations"]
    group_name = "ee_pose_in_unified"
    if group_name in obs_grp:
        if not overwrite:
            raise RuntimeError(f"observations/{group_name} already exists")
        del obs_grp[group_name]

    ee_grp = obs_grp.create_group(group_name)
    ee_grp.attrs["frame"] = "unified (mid_base orientation, origin 25cm below mid_base z)"
    ee_grp.attrs["eef_frame"] = "teleop_ik_ee"
    if eef_pose_summary:
        ee_grp.attrs["extrinsics_json"] = eef_pose_summary["extrinsics_json"]
        ee_grp.attrs["urdf"] = eef_pose_summary["urdf"]
        ee_grp.attrs["unified_origin_in_mid_m"] = \
            np.asarray(eef_pose_summary["unified_origin_in_mid_m"])
    for arm in ARM_ORDER:
        g = ee_grp.create_group(arm)
        g.attrs["source"] = "qpos_fk"
        g.create_dataset("quat", data=eef_poses[arm]["quat"])
        g.create_dataset("rpy", data=eef_poses[arm]["rpy"])
        g.create_dataset("matrix", data=eef_poses[arm]["matrix"])


def annotate_raw_driver_ee_pose_groups(root):
    """Mark legacy driver EE pose groups as raw joint6-frame data."""
    if "observations" not in root:
        return
    obs_grp = root["observations"]
    warning = (
        "Raw Piper driver /puppet/end_pose joint6 frame. Do not compare directly "
        "with camera_pose_in_unified; use observations/ee_pose_in_unified for "
        "camera/EEF alignment and VLA training targets."
    )
    for group_name, repr_name, aligned_name in (
        ("ee_pose_quat", "xyz_qxyzw", "quat"),
        ("ee_pose_rpy", "xyz_rpy", "rpy"),
    ):
        if group_name not in obs_grp:
            continue
        group = obs_grp[group_name]
        group.attrs["frame"] = "raw_driver_joint6"
        group.attrs["source"] = "/puppet/end_pose_<arm>"
        group.attrs["representation"] = repr_name
        group.attrs["warning"] = warning
        group.attrs["preferred_aligned_group"] = "observations/ee_pose_in_unified"
        for arm in ARM_ORDER:
            if arm in group:
                group[arm].attrs["frame"] = "raw_driver_joint6"
                group[arm].attrs["source"] = "/puppet/end_pose_<arm>"
                group[arm].attrs["preferred_aligned_dataset"] = (
                    f"observations/ee_pose_in_unified/{arm}/{aligned_name}"
                )


def save_data(args, timesteps, actions, dataset_path, cam_info=None):
    """Mirror layout of collect_data_shuai.py with 3-arm extensions."""
    data_size = len(actions)
    data_dict = {
        "/observations/qpos": [],
        "/observations/qvel": [],
        "/observations/effort": [],
        "/action": [],
        "/base_action": [],
    }
    for cam in CAM_ORDER:
        data_dict[f"/observations/images/{cam}"] = []
    for arm in ARM_ORDER:
        data_dict[f"/observations/ee_pose_quat/{arm}"] = []
        data_dict[f"/observations/ee_pose_rpy/{arm}"] = []

    while actions:
        action = actions.pop(0)
        ts = timesteps.pop(0)
        obs = ts.observation
        data_dict["/observations/qpos"].append(obs["qpos"])
        data_dict["/observations/qvel"].append(obs["qvel"])
        data_dict["/observations/effort"].append(obs["effort"])
        data_dict["/action"].append(action)
        data_dict["/base_action"].append(obs["base_vel"])
        for cam in CAM_ORDER:
            data_dict[f"/observations/images/{cam}"].append(obs["images"][cam])
        for arm in ARM_ORDER:
            data_dict[f"/observations/ee_pose_quat/{arm}"].append(obs["ee_pose_quat"][arm])
            data_dict[f"/observations/ee_pose_rpy/{arm}"].append(obs["ee_pose_rpy"][arm])

    # ---- POST-PROCESS: per-frame camera pose in unified arm frame ----
    # Done AFTER collection (so the real-time teleop loop isn't perturbed)
    # but BEFORE HDF5 write so derived poses live in the same file.
    qpos_array = np.asarray(data_dict["/observations/qpos"], dtype=float)
    cam_poses, cam_pose_summary = _compute_camera_poses_in_unified(qpos_array)
    eef_poses, eef_pose_summary = _compute_eef_poses_in_unified(qpos_array)

    t0 = time.time()
    with h5py.File(dataset_path + ".hdf5", "w", rdcc_nbytes=1024 ** 2 * 2) as root:
        root.attrs["sim"] = False
        root.attrs["compress"] = False
        root.attrs["task_name"] = args.task_name
        root.attrs["task_description"] = args.task_description
        root.attrs["arm_order"] = ",".join(ARM_ORDER)
        root.attrs["frame_rate"] = int(args.frame_rate)

        obs_grp = root.create_group("observations")
        images = obs_grp.create_group("images")
        for cam in CAM_ORDER:
            images.create_dataset(
                cam, (data_size, IMG_H, IMG_W, 3),
                dtype="uint8", chunks=(1, IMG_H, IMG_W, 3),
            )

        ee_q = obs_grp.create_group("ee_pose_quat")
        ee_r = obs_grp.create_group("ee_pose_rpy")
        for arm in ARM_ORDER:
            ee_q.create_dataset(arm, (data_size, EE_QUAT_DIM))
            ee_r.create_dataset(arm, (data_size, EE_RPY_DIM))

        N = len(ARM_ORDER) * DOF_PER_ARM
        obs_grp.create_dataset("qpos",   (data_size, N))
        obs_grp.create_dataset("qvel",   (data_size, N))
        obs_grp.create_dataset("effort", (data_size, N))
        root.create_dataset("action",      (data_size, N))
        root.create_dataset("base_action", (data_size, 2))

        for name, array in data_dict.items():
            root[name][...] = array

        annotate_raw_driver_ee_pose_groups(root)

        # Camera intrinsics — one static copy per camera (NOT per-frame).
        # K(9) = 3x3 intrinsic matrix, D = distortion coeffs, R(9) = rectification,
        # P(12) = 3x4 projection. width/height/distortion_model as group attrs.
        if cam_info:
            ci_grp = root.create_group("camera_info")
            for cam in CAM_ORDER:
                msg = cam_info.get(cam)
                if msg is None:
                    print(f"\033[33m[collect] WARN: no CameraInfo for {cam}; "
                          f"intrinsics not saved.\033[0m")
                    continue
                g = ci_grp.create_group(cam)
                g.create_dataset("K", data=np.array(msg.K, dtype=float))
                g.create_dataset("D", data=np.array(msg.D, dtype=float))
                g.create_dataset("R", data=np.array(msg.R, dtype=float))
                g.create_dataset("P", data=np.array(msg.P, dtype=float))
                g.attrs["width"] = msg.width
                g.attrs["height"] = msg.height
                g.attrs["distortion_model"] = msg.distortion_model
                try:
                    hfov, vfov = camera_fov_from_intrinsics(msg.K, msg.width, msg.height)
                    g.attrs["horizontal_fov_rad"] = hfov
                    g.attrs["vertical_fov_rad"] = vfov
                    g.attrs["horizontal_fov_deg"] = float(np.degrees(hfov))
                    g.attrs["vertical_fov_deg"] = float(np.degrees(vfov))
                except Exception as e:
                    print(f"\033[33m[collect] WARN: FOV skipped for {cam}: {e}\033[0m")

        # Camera pose in unified arm frame (post-processed, per-frame).
        # Layout: observations/camera_pose_in_unified/<cam>/{quat,rpy,matrix}
        # quat = (x,y,z,qx,qy,qz,qw), rpy = (x,y,z,r,p,y), matrix = 4x4 SE(3).
        if cam_poses is not None:
            write_camera_poses_in_unified(root, cam_poses, cam_pose_summary, overwrite=False)

        # Teleop IK EE pose in unified arm frame. This is derived from qpos/FK,
        # so original qpos/action/driver EE poses remain untouched.
        if eef_poses is not None:
            write_eef_poses_in_unified(root, eef_poses, eef_pose_summary, overwrite=False)
    print(f"\033[32m\nSaving: {time.time() - t0:.1f} secs. {dataset_path}\033[0m\n")


class RosOperator:
    def __init__(self, args):
        self.args = args
        self.bridge = CvBridge()

        # Per-arm deques
        self.puppet_q  = {a: deque() for a in ARM_ORDER}  # JointState (state)
        self.master_q  = {a: deque() for a in ARM_ORDER}  # JointState (action src)
        self.ee_pose   = {a: deque() for a in ARM_ORDER}  # PoseStamped

        # Per-camera deques (color)
        self.img       = {c: deque() for c in CAM_ORDER}
        self.img_depth = {c: deque() for c in CAM_ORDER}

        # Camera intrinsics (CameraInfo) — constant, so we only keep the latest
        # message per camera and write a single copy into the HDF5, NOT per-frame.
        self.cam_info  = {c: None for c in CAM_ORDER}

        self.base = deque()
        self.teleop_state = "IDLE"
        self.teleop_state_stamp = time.monotonic()

        self.init_ros()

    # ------------- ROS plumbing -------------
    def _push(self, dq, msg, cap=2000):
        if len(dq) >= cap:
            dq.popleft()
        dq.append(msg)

    def init_ros(self):
        rospy.init_node("record_episodes_3arm", anonymous=True)
        A = self.args

        # Joint states (3 arms x {puppet, master})
        rospy.Subscriber(A.puppet_topic_left,  JointState,
                         lambda m: self._push(self.puppet_q["left"], m),
                         queue_size=1000, tcp_nodelay=True)
        rospy.Subscriber(A.puppet_topic_right, JointState,
                         lambda m: self._push(self.puppet_q["right"], m),
                         queue_size=1000, tcp_nodelay=True)
        rospy.Subscriber(A.puppet_topic_mid,   JointState,
                         lambda m: self._push(self.puppet_q["mid"], m),
                         queue_size=1000, tcp_nodelay=True)
        rospy.Subscriber(A.master_topic_left,  JointState,
                         lambda m: self._push(self.master_q["left"], m),
                         queue_size=1000, tcp_nodelay=True)
        rospy.Subscriber(A.master_topic_right, JointState,
                         lambda m: self._push(self.master_q["right"], m),
                         queue_size=1000, tcp_nodelay=True)
        rospy.Subscriber(A.master_topic_mid,   JointState,
                         lambda m: self._push(self.master_q["mid"], m),
                         queue_size=1000, tcp_nodelay=True)

        # EE pose (PoseStamped)
        rospy.Subscriber(A.ee_topic_left,  PoseStamped,
                         lambda m: self._push(self.ee_pose["left"], m),
                         queue_size=1000, tcp_nodelay=True)
        rospy.Subscriber(A.ee_topic_right, PoseStamped,
                         lambda m: self._push(self.ee_pose["right"], m),
                         queue_size=1000, tcp_nodelay=True)
        rospy.Subscriber(A.ee_topic_mid,   PoseStamped,
                         lambda m: self._push(self.ee_pose["mid"], m),
                         queue_size=1000, tcp_nodelay=True)

        # Cameras (3 color)
        rospy.Subscriber(A.img_front_topic, Image,
                         lambda m: self._push(self.img["cam_front"], m),
                         queue_size=1000, tcp_nodelay=True)
        rospy.Subscriber(A.img_left_topic,  Image,
                         lambda m: self._push(self.img["cam_left"], m),
                         queue_size=1000, tcp_nodelay=True)
        rospy.Subscriber(A.img_right_topic, Image,
                         lambda m: self._push(self.img["cam_right"], m),
                         queue_size=1000, tcp_nodelay=True)

        # Camera intrinsics (one CameraInfo per color stream; constant, latest kept)
        rospy.Subscriber(A.info_front_topic, CameraInfo,
                         lambda m: self._set_info("cam_front", m), queue_size=1)
        rospy.Subscriber(A.info_left_topic,  CameraInfo,
                         lambda m: self._set_info("cam_left", m),  queue_size=1)
        rospy.Subscriber(A.info_right_topic, CameraInfo,
                         lambda m: self._set_info("cam_right", m), queue_size=1)

        # Base (for compat — may be all-zero if no base running)
        rospy.Subscriber(A.robot_base_topic, Odometry,
                         lambda m: self._push(self.base, m),
                         queue_size=1000, tcp_nodelay=True)

        # AVP teleop state (drives episode lifecycle)
        rospy.Subscriber(A.teleop_state_topic, String, self._state_cb,
                         queue_size=10, tcp_nodelay=True)

    def _state_cb(self, msg):
        self.teleop_state = msg.data
        self.teleop_state_stamp = time.monotonic()

    def _set_info(self, cam, msg):
        # Intrinsics are constant; just keep the most recent CameraInfo.
        self.cam_info[cam] = msg

    # ------------- Frame sync -------------
    def get_frame(self):
        # All required deques must be non-empty
        for dq in (
            *self.img.values(),
            *self.puppet_q.values(),
            *self.master_q.values(),
            *self.ee_pose.values(),
        ):
            if not dq:
                return None

        # Sync stamp = oldest "latest" across all required streams.
        latest = [
            *[dq[-1].header.stamp.to_sec() for dq in self.img.values()],
            *[dq[-1].header.stamp.to_sec() for dq in self.puppet_q.values()],
            *[dq[-1].header.stamp.to_sec() for dq in self.master_q.values()],
            *[dq[-1].header.stamp.to_sec() for dq in self.ee_pose.values()],
        ]
        frame_time = min(latest)

        # Wait until every stream has caught up to frame_time
        for dq in (
            *self.img.values(),
            *self.puppet_q.values(),
            *self.master_q.values(),
            *self.ee_pose.values(),
        ):
            if dq[-1].header.stamp.to_sec() < frame_time:
                return None

        # Pop oldest entries < frame_time, take first >= frame_time
        def pop_to(dq):
            while dq[0].header.stamp.to_sec() < frame_time:
                dq.popleft()
            return dq.popleft()

        img_msgs = {c: pop_to(self.img[c]) for c in CAM_ORDER}
        pup_msgs = {a: pop_to(self.puppet_q[a]) for a in ARM_ORDER}
        mst_msgs = {a: pop_to(self.master_q[a]) for a in ARM_ORDER}
        ee_msgs  = {a: pop_to(self.ee_pose[a])  for a in ARM_ORDER}

        # Base is optional — keep last if exists, else zeros
        base_lin_x = 0.0
        base_ang_z = 0.0
        if self.base:
            # We don't strict-sync base; just grab latest.
            while len(self.base) > 1:
                self.base.popleft()
            b = self.base[-1]
            base_lin_x = b.twist.twist.linear.x
            base_ang_z = b.twist.twist.angular.z

        return img_msgs, pup_msgs, mst_msgs, ee_msgs, (base_lin_x, base_ang_z)

    # ------------- Helpers -------------
    @staticmethod
    def _ee_to_quat_rpy(ps: PoseStamped):
        p = ps.pose.position
        q = ps.pose.orientation
        quat7 = np.array([p.x, p.y, p.z, q.x, q.y, q.z, q.w], dtype=float)
        rpy = euler_from_quaternion([q.x, q.y, q.z, q.w])  # roll, pitch, yaw
        rpy6 = np.array([p.x, p.y, p.z, rpy[0], rpy[1], rpy[2]], dtype=float)
        return quat7, rpy6

    @staticmethod
    def _arm_vec(per_arm_dict, attr):
        """Concatenate per-arm 7-vectors in ARM_ORDER."""
        parts = []
        for a in ARM_ORDER:
            vec = np.asarray(getattr(per_arm_dict[a], attr), dtype=float)
            # JointState position has 7 entries (6 joints + gripper)
            parts.append(vec[:DOF_PER_ARM])
        return np.concatenate(parts, axis=0)

    # ------------- Main process -------------
    def wait_for_engage(self, poll_hz=20.0, stale_after_s=5.0):
        rate = rospy.Rate(poll_hz)
        printed = False
        while not rospy.is_shutdown():
            stale = (time.monotonic() - self.teleop_state_stamp) > stale_after_s
            if not printed:
                print(
                    f"[collect] waiting for AVP ENGAGED (current state={self.teleop_state}"
                    f"{', STALE' if stale else ''})..."
                )
                printed = True
            if self.teleop_state == "ENGAGED":
                return True
            rate.sleep()
        return False

    def process(self):
        ok = self.wait_for_engage()
        if not ok:
            return [], []

        print(
            f"\033[32m[collect] ENGAGED detected — recording up to "
            f"{self.args.max_timesteps} frames at {self.args.frame_rate} Hz\033[0m"
        )

        timesteps = []
        actions = []
        count = 0
        # Track the count value of the LAST frame recorded while state was
        # ENGAGED. After the loop we trim everything after this point so the
        # trailing ~4 s of "both-hands held to confirm pause" never make it
        # into the saved episode. Mid-episode DISARMED bounces (false-trigger
        # that goes back to ENGAGED) are NOT trimmed because last_engaged_count
        # advances every time we're in ENGAGED again.
        last_engaged_count = 0
        rate = rospy.Rate(self.args.frame_rate)
        sync_warned = False

        # States during which we KEEP recording. DISARMED is the 4-second
        # "pending pause" window (both hands held) — if the user releases
        # early the FSM bounces back to ENGAGED and we keep the frames.
        RECORDING_STATES = {"ENGAGED", "DISARMED"}

        while count < self.args.max_timesteps + 1 and not rospy.is_shutdown():
            # Stop on confirmed pause (IDLE) or any other non-recording state.
            if self.teleop_state not in RECORDING_STATES:
                print(
                    f"[collect] state -> {self.teleop_state} after {count} frames "
                    f"-> stop recording."
                )
                break

            frame = self.get_frame()
            if frame is None:
                if not sync_warned:
                    print("[collect] syn fail (waiting for first synced frame)")
                    sync_warned = True
                rate.sleep()
                continue
            sync_warned = False

            img_msgs, pup_msgs, mst_msgs, ee_msgs, base_vel = frame
            count += 1
            if self.teleop_state == "ENGAGED":
                last_engaged_count = count

            # Images
            image_dict = {
                c: self.bridge.imgmsg_to_cv2(img_msgs[c], "passthrough")
                for c in CAM_ORDER
            }

            # EE pose per arm
            ee_quat_dict = {}
            ee_rpy_dict  = {}
            for a in ARM_ORDER:
                q7, r6 = self._ee_to_quat_rpy(ee_msgs[a])
                ee_quat_dict[a] = q7
                ee_rpy_dict[a]  = r6

            # Joint states concatenated in ARM_ORDER
            qpos   = self._arm_vec(pup_msgs, "position")
            qvel   = self._arm_vec(pup_msgs, "velocity")
            effort = self._arm_vec(pup_msgs, "effort")

            obs = collections.OrderedDict()
            obs["images"] = image_dict
            obs["ee_pose_quat"] = ee_quat_dict
            obs["ee_pose_rpy"]  = ee_rpy_dict
            obs["qpos"]   = qpos
            obs["qvel"]   = qvel
            obs["effort"] = effort
            obs["base_vel"] = [base_vel[0], base_vel[1]]

            if count == 1:
                timesteps.append(dm_env.TimeStep(
                    step_type=dm_env.StepType.FIRST,
                    reward=None, discount=None, observation=obs))
                continue

            ts = dm_env.TimeStep(
                step_type=dm_env.StepType.MID,
                reward=None, discount=None, observation=obs)

            # Action = concatenated master positions (the actually-executed commands:
            # physical master arms for L/R, AVP teleop output for mid).
            action = self._arm_vec(mst_msgs, "position")
            actions.append(action)
            timesteps.append(ts)

            if count % 30 == 0:
                print(f"[collect] frame {count} (state={self.teleop_state})")
            rate.sleep()

        # Trim trailing frames that were recorded during DISARMED (i.e., the
        # hand-hold window at the end of the episode). Keep up to and including
        # the last ENGAGED frame.
        #   timesteps layout: [FIRST(count=1), MID(count=2), ..., MID(count=N)]
        #   actions   layout: [action(count=2), ...,           action(count=N)]
        if last_engaged_count > 0 and last_engaged_count < count:
            drop_ts = count - last_engaged_count
            drop_act = drop_ts  # actions track MID timesteps 1:1 after the first FIRST
            timesteps = timesteps[:last_engaged_count]
            actions = actions[: max(0, last_engaged_count - 1)]
            print(
                f"[collect] trimmed trailing {drop_ts} DISARMED frames "
                f"(hand-hold window); kept {len(actions)} actions, "
                f"{len(timesteps)} timesteps"
            )
        else:
            print(f"[collect] recorded {len(actions)} actions, {len(timesteps)} timesteps")
        return timesteps, actions


def get_arguments():
    p = argparse.ArgumentParser()
    p.add_argument("--dataset_dir",      type=str, default="./data")
    p.add_argument("--task_name",        type=str, default="three_arm_task")
    p.add_argument("--task_description", type=str, default="")
    p.add_argument("--episode_idx",      type=int, default=0)
    p.add_argument("--max_timesteps",    type=int, default=1500)
    p.add_argument("--frame_rate",       type=int, default=30)

    # Camera color topics
    p.add_argument("--img_front_topic", type=str, default="/camera_f/color/image_raw")
    p.add_argument("--img_left_topic",  type=str, default="/camera_l/color/image_raw")
    p.add_argument("--img_right_topic", type=str, default="/camera_r/color/image_raw")

    # Camera intrinsics topics (CameraInfo; stored once, not per-frame)
    p.add_argument("--info_front_topic", type=str, default="/camera_f/color/camera_info")
    p.add_argument("--info_left_topic",  type=str, default="/camera_l/color/camera_info")
    p.add_argument("--info_right_topic", type=str, default="/camera_r/color/camera_info")

    # Joint topics (puppet = arm state, master = command source / action)
    p.add_argument("--puppet_topic_left",  type=str, default="/puppet/joint_left")
    p.add_argument("--puppet_topic_right", type=str, default="/puppet/joint_right")
    p.add_argument("--puppet_topic_mid",   type=str, default="/puppet/joint_mid")
    p.add_argument("--master_topic_left",  type=str, default="/master/joint_left")
    p.add_argument("--master_topic_right", type=str, default="/master/joint_right")
    p.add_argument("--master_topic_mid",   type=str, default="/master/joint_mid")

    # EE pose topics
    p.add_argument("--ee_topic_left",  type=str, default="/puppet/end_pose_left")
    p.add_argument("--ee_topic_right", type=str, default="/puppet/end_pose_right")
    p.add_argument("--ee_topic_mid",   type=str, default="/puppet/end_pose_mid")

    # Base
    p.add_argument("--robot_base_topic", type=str, default="/odom")

    # AVP teleop state
    p.add_argument("--teleop_state_topic", type=str, default="/teleop/state")

    return p.parse_args()


def main():
    args = get_arguments()
    op = RosOperator(args)
    timesteps, actions = op.process()

    dataset_dir = os.path.join(args.dataset_dir, args.task_name)
    if len(actions) == 0:
        print("\033[31m[collect] no actions recorded — skip save.\033[0m")
        sys.exit(1)
    if not os.path.exists(dataset_dir):
        os.makedirs(dataset_dir)
    dataset_path = os.path.join(dataset_dir, f"episode_{args.episode_idx}")
    save_data(args, timesteps, actions, dataset_path, op.cam_info)


if __name__ == "__main__":
    main()
