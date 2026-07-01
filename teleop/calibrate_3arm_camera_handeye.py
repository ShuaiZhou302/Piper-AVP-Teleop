#!/usr/bin/env python3
"""
Hand-eye calibration for ONE Piper wrist camera at a time.

For each capture:
  1. find checkerboard corners in the live image (cv2.findChessboardCorners)
  2. cv2.solvePnP using the camera intrinsics -> T_cam_from_board
  3. pinocchio FK on current joint state -> T_base_from_wrist (== ee frame)

After N >= 3 samples (recommended >= 15), run cv2.calibrateHandEye(...) to
solve AX = XB, where X = T_wrist_from_camera (the fixed mounting transform
we are after). Tries all 5 OpenCV methods, picks the lowest AX-XB residual.

Pre-conditions:
  * inference-mode launch (start_ms_piper_3arm.launch mode:=1 auto_enable:=true)
  * multi_camera_shuai.launch publishing /camera_<f|l|r>/color/{image_raw,camera_info}
  * a checkerboard taped flat to a rigid surface, FIXED in the workspace and
    visible to the chosen arm's wrist camera across the calibration poses

Run:
  conda activate aloha
  cd .../Piper-AVP-Teleop/teleop
  python calibrate_3arm_camera_handeye.py --arm m \
         --board_cols 7 --board_rows 5 --square_size_m 0.025 --samples 20
"""
import argparse
import json
import os
import select
import sys
import termios
import time
import tty
from datetime import datetime

import cv2
import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

# Import this first; pulls in casadi/pinocchio before rospy (libstdc++ ordering).
from eef_keyboard_control_singlearm import PinocchioIKSolver  # noqa: E402

import pinocchio as pin  # noqa: E402
import rospy  # noqa: E402
from cv_bridge import CvBridge  # noqa: E402
from sensor_msgs.msg import CameraInfo, Image, JointState  # noqa: E402
from std_msgs.msg import Header  # noqa: E402
from tf.transformations import euler_from_matrix  # noqa: E402


JOINT_NAMES = ["joint0", "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
INITIAL_ARM_JOINTS = (0.0463, 0.5300, -0.5562, 0.0000, 0.6500, 0.0000)
INITIAL_GRIPPER = 0.1

ARM_CHOICES = {
    "l": "left",  "left":  "left",
    "m": "mid",   "mid":   "mid",
    "r": "right", "right": "right",
}
# camera_l / camera_f / camera_r — mid uses the front-camera slot
ARM_CAM_SHORT = {"left": "l", "mid": "f", "right": "r"}


def resolve_arm(arg):
    if arg is None:
        while True:
            x = input("Arm (l/m/r): ").strip().lower()
            if x in ARM_CHOICES:
                return ARM_CHOICES[x]
            print("please input l / m / r")
    key = arg.strip().lower()
    if key not in ARM_CHOICES:
        raise SystemExit(f"Invalid --arm: {arg}")
    return ARM_CHOICES[key]


def build_board_points(cols, rows, square_size_m):
    """3D coords of the inner corners in the board frame (Z=0 plane)."""
    pts = np.zeros((rows * cols, 3), dtype=np.float32)
    pts[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    pts *= float(square_size_m)
    return pts


class RawKeyboard:
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, *_):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

    def read_key(self, timeout=0.05):
        ready, _, _ = select.select([sys.stdin], [], [], timeout)
        return sys.stdin.read(1) if ready else None


class HandEyeCalibrator:
    def __init__(self, args):
        self.args = args
        self.arm = args.arm
        self.teach_mode = bool(args.teach_mode)
        rospy.init_node(f"calibrate_handeye_{self.arm}", anonymous=True)
        self.ik = PinocchioIKSolver(args.urdf)
        self.bridge = CvBridge()

        self.joint = None
        self.image = None
        self._joint_stamp = 0.0
        self._image_stamp = 0.0
        self.K = None
        self.D = None

        rospy.Subscriber(
            f"/puppet/joint_{self.arm}", JointState,
            self._joint_cb, queue_size=50, tcp_nodelay=True,
        )
        cam_short = ARM_CAM_SHORT[self.arm]
        rospy.Subscriber(
            f"/camera_{cam_short}/color/image_raw", Image,
            self._image_cb, queue_size=1, tcp_nodelay=True, buff_size=2 ** 24,
        )
        rospy.Subscriber(
            f"/camera_{cam_short}/color/camera_info", CameraInfo,
            self._info_cb, queue_size=10,
        )
        # In teach mode the user moves the arm by hand (motors off, mode=0).
        # We never publish commands in that mode, so skip the publisher.
        self.pub = (
            None if self.teach_mode
            else rospy.Publisher(f"/master/joint_{self.arm}", JointState, queue_size=10)
        )

        self.target_xyz = None
        self.target_rpy = None
        self.target_q = None
        self.gripper = float(args.gripper)

        self.board_pts = build_board_points(
            args.board_cols, args.board_rows, args.square_size_m,
        )
        # cv2.calibrateHandEye input buffers
        self.R_base_from_wrist_list = []
        self.t_base_from_wrist_list = []
        self.R_cam_from_board_list = []
        self.t_cam_from_board_list = []
        self.sample_meta = []
        # counts the total c presses (success + miss); used to name miss PNGs.
        self.attempt_idx = 0

    # ---------- ROS callbacks ----------
    def _joint_cb(self, msg):
        self.joint = msg
        self._joint_stamp = msg.header.stamp.to_sec() if msg.header.stamp else time.time()

    def _image_cb(self, msg):
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self._image_stamp = msg.header.stamp.to_sec() if msg.header.stamp else time.time()
        except Exception as e:
            rospy.logwarn_throttle(2.0, f"[cam] cv_bridge error: {e}")

    def _info_cb(self, msg):
        if self.K is None:
            self.K = np.asarray(msg.K, dtype=np.float64).reshape(3, 3)
            self.D = np.asarray(msg.D, dtype=np.float64).reshape(-1)

    # ---------- ROS plumbing ----------
    def wait_feedback(self, timeout=15.0):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            ready = (
                self.joint is not None
                and len(self.joint.position) >= 7
                and self.image is not None
                and self.K is not None
            )
            if ready:
                return True
            if rospy.Time.now() > deadline:
                return False
            rate.sleep()
        return False

    def publish(self, q7):
        msg = JointState()
        msg.header = Header(stamp=rospy.Time.now())
        msg.name = JOINT_NAMES
        msg.position = list(q7)
        self.pub.publish(msg)

    def fk(self, q6):
        q = np.asarray(q6, dtype=float)
        pin.framesForwardKinematics(self.ik.model, self.ik.data, q)
        se3 = self.ik.data.oMf[self.ik.ee_frame_id]
        T = np.eye(4)
        T[:3, :3] = np.asarray(se3.rotation, dtype=float)
        T[:3, 3] = np.asarray(se3.translation, dtype=float).flatten()
        return T

    def set_active_from_feedback(self):
        q6 = np.asarray(self.joint.position[:6], dtype=float)
        T = self.fk(q6)
        self.target_xyz = T[:3, 3].copy()
        self.target_rpy = np.asarray(euler_from_matrix(T), dtype=float)
        self.target_q = np.asarray(self.joint.position[:7], dtype=float)
        self.gripper = float(self.target_q[6])
        self.ik.init_data = q6.copy()
        self.ik.history_data = q6.copy()

    def solve_and_publish(self):
        seed = self.target_q[:6] if self.target_q is not None else None
        sol6, ok, msg = self.ik.solve(
            self.target_xyz, self.target_rpy,
            gripper=self.gripper, motorstate=seed,
        )
        if not ok:
            print(f"\n[ik] FAIL: {msg}")
            return False
        self.target_q = np.asarray(list(sol6) + [self.gripper], dtype=float)
        self.publish(self.target_q)
        return True

    def ramp_to(self, target7, label):
        cur = np.asarray(self.joint.position[:7], dtype=float)
        tgt = np.asarray(target7, dtype=float)
        max_d = float(np.max(np.abs(tgt[:6] - cur[:6])))
        duration = max(max_d / self.args.ramp_speed_rad_s, 1.0)
        steps = max(1, int(duration * self.args.hz))
        rate = rospy.Rate(self.args.hz)
        print(f"\n[ramp] {label}: {duration:.2f}s (max_delta={max_d:.3f} rad)")
        for i in range(steps):
            if rospy.is_shutdown():
                return False
            alpha = (i + 1) / steps
            self.publish((1.0 - alpha) * cur + alpha * tgt)
            rate.sleep()
        for _ in range(int(self.args.hz * 0.5)):
            if rospy.is_shutdown():
                return False
            self.publish(tgt)
            rate.sleep()
        return True

    def reset_arm(self):
        reset7 = list(INITIAL_ARM_JOINTS) + [INITIAL_GRIPPER]
        ok = self.ramp_to(reset7, "reset_to_INITIAL")
        if ok:
            self.set_active_from_feedback()
        return ok

    # ---------- Vision side ----------
    def _find_corners(self, gray, pattern):
        """Detect chessboard corners with a STABLE ordering.

        We use cv2.findChessboardCorners (classic) instead of SB because the
        classic detector's corner ordering is much more reliable across
        moderate camera rolls. SB is more robust to blur but can return the
        corner list FLIPPED at extreme rolls, which silently destroys
        hand-eye calibration (each sample is internally consistent, but the
        board frame disagrees between samples).
        Returns (corners, used_pattern, "classic") or (None, None, None).
        """
        flags = (cv2.CALIB_CB_ADAPTIVE_THRESH
                 + cv2.CALIB_CB_NORMALIZE_IMAGE
                 + cv2.CALIB_CB_FILTER_QUADS)
        for pat in (pattern, (pattern[1], pattern[0])):  # try swapped too
            found, corners = cv2.findChessboardCorners(gray, pat, flags=flags)
            if found:
                corners = cv2.cornerSubPix(
                    gray, corners, (11, 11), (-1, -1),
                    (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3),
                )
                return corners, pat, "classic"
        return None, None, None

    def detect_and_solve(self):
        if self.image is None or self.K is None:
            return None
        gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        pattern_cfg = (self.args.board_cols, self.args.board_rows)
        corners, pat_used, method = self._find_corners(gray, pattern_cfg)
        if corners is None:
            return None
        if pat_used != pattern_cfg:
            print(f"[detect] note: pattern {pat_used} matched instead of "
                  f"configured {pattern_cfg}. If this keeps happening, swap "
                  f"--board_cols / --board_rows.")
        # Build 3D points for the pattern that actually matched.
        board_pts = build_board_points(pat_used[0], pat_used[1],
                                       self.args.square_size_m)
        ok, rvec, tvec = cv2.solvePnP(
            board_pts, corners, self.K, self.D, flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not ok:
            return None
        R, _ = cv2.Rodrigues(rvec)
        reproj, _ = cv2.projectPoints(board_pts, rvec, tvec, self.K, self.D)
        err = np.linalg.norm(corners.reshape(-1, 2) - reproj.reshape(-1, 2), axis=1)
        rms = float(np.sqrt(np.mean(err ** 2)))
        annotated = self.image.copy()
        cv2.drawChessboardCorners(annotated, pat_used, corners, True)
        # Stamp the detector used + RMS on the image for easy inspection
        cv2.putText(annotated, f"{method} {pat_used} rms={rms:.2f}px",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (0, 255, 0), 2, cv2.LINE_AA)
        return R, tvec.flatten(), rms, annotated

    def try_capture(self):
        if self.joint is None or self.K is None:
            print("\n[cap] missing joint or intrinsics, skip")
            return
        if self.image is None:
            print("\n[cap] no image yet, skip")
            return
        # Settle: in teach mode, /puppet/joint updates fast but image stream has
        # ~30-100ms latency from the camera. Brief sleep so the next image+joint
        # we grab are both from "now", not from when the arm was still moving.
        time.sleep(0.25)
        self.attempt_idx += 1
        # Diagnose pose stability: warn if image and joint stamps are far apart
        # (means user moved the arm faster than the image stream caught up).
        dt_img_joint = self._joint_stamp - self._image_stamp
        if abs(dt_img_joint) > 0.15:
            print(f"\n[cap] WARNING image/joint stamp diff = {dt_img_joint*1000:+.0f} ms "
                  f"(arm may have moved between image and joint). Wait longer before c.")
        result = self.detect_and_solve()
        if result is None:
            # Save the raw frame so the operator can see WHY detection failed
            # (board out of frame, too tilted, motion blur, glare, etc.).
            miss_path = os.path.join(
                self.args.out_dir,
                f"handeye_miss_{self.arm}_attempt{self.attempt_idx:03d}.png",
            )
            cv2.imwrite(miss_path, self.image)
            print(f"\n[cap] checkerboard NOT detected (attempt {self.attempt_idx}). "
                  f"Saved raw frame -> {miss_path}\n"
                  f"      Open the PNG to see what the camera saw; "
                  f"adjust pose so the whole 7x5 grid is visible.")
            return
        R_cam_from_board, t_cam_from_board, rms, annotated = result
        q6 = np.asarray(self.joint.position[:6], dtype=float)
        T_base_from_wrist = self.fk(q6)
        self.R_base_from_wrist_list.append(T_base_from_wrist[:3, :3])
        self.t_base_from_wrist_list.append(T_base_from_wrist[:3, 3])
        self.R_cam_from_board_list.append(R_cam_from_board)
        self.t_cam_from_board_list.append(t_cam_from_board)
        idx = len(self.R_base_from_wrist_list)
        self.sample_meta.append({
            "idx": idx,
            "rms_reproj_px": rms,
            "joint_rad": q6.tolist(),
            "R_cam_from_board": R_cam_from_board.tolist(),
            "t_cam_from_board_m": t_cam_from_board.tolist(),
        })
        img_path = os.path.join(self.args.out_dir,
                                f"handeye_capture_{self.arm}_{idx:03d}.png")
        raw_path = os.path.join(self.args.out_dir,
                                f"handeye_raw_{self.arm}_{idx:03d}.png")
        cv2.imwrite(img_path, annotated)
        cv2.imwrite(raw_path, self.image)
        print(f"\n[cap] {idx}/{self.args.samples} OK  rms={rms:.3f} px  ({img_path})")

    # ---------- Hand-eye solve ----------
    def _rotation_axis_diversity(self):
        """Print a quick test of how much the wrist *rotation* varies between
        samples. Hand-eye needs the rotation axes of A_ij (wrist_i->wrist_j)
        to span more than a single direction; pure-translation motions are
        degenerate and produce garbage X."""
        Rs = self.R_base_from_wrist_list
        n = len(Rs)
        axes = []
        angles = []
        for i in range(n):
            for j in range(i + 1, n):
                R = Rs[i].T @ Rs[j]
                tr = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
                ang = float(np.arccos(tr))
                if ang < 1e-3:  # too small, axis undefined
                    continue
                w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
                if np.linalg.norm(w) < 1e-9:
                    continue
                axes.append(w / np.linalg.norm(w))
                angles.append(ang)
        if not axes:
            print("[diag] WARNING no rotation between sample pairs detected")
            return
        A = np.asarray(axes)
        ang = np.asarray(angles)
        s = np.linalg.svd(A, compute_uv=False)
        med = float(np.median(np.rad2deg(ang)))
        print(f"[diag] rotation between pairs: median = {med:.1f} deg, "
              f"max = {np.rad2deg(ang.max()):.1f} deg, pairs = {len(axes)}")
        print(f"[diag] rotation axis SVD = {s.round(3).tolist()}  "
              "(want all 3 values > 1.0; smallest near 0 means rotation is "
              "around essentially one axis -> hand-eye degenerate)")

    def calibrate(self):
        n = len(self.R_base_from_wrist_list)
        if n < 3:
            print(f"\n[fit] need >= 3 samples, have {n}; abort")
            return None
        self._rotation_axis_diversity()
        methods = [
            ("TSAI", cv2.CALIB_HAND_EYE_TSAI),
            ("PARK", cv2.CALIB_HAND_EYE_PARK),
            ("HORAUD", cv2.CALIB_HAND_EYE_HORAUD),
            ("ANDREFF", cv2.CALIB_HAND_EYE_ANDREFF),
            ("DANIILIDIS", cv2.CALIB_HAND_EYE_DANIILIDIS),
        ]
        results = {}
        for name, m in methods:
            try:
                R, t = cv2.calibrateHandEye(
                    self.R_base_from_wrist_list, self.t_base_from_wrist_list,
                    self.R_cam_from_board_list, self.t_cam_from_board_list,
                    method=m,
                )
            except cv2.error as e:
                print(f"[fit] {name:11s}  CRASHED: {str(e).splitlines()[-1]}")
                continue
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = np.asarray(t, dtype=float).flatten()
            resid = self._handeye_residual(T)
            results[name] = (T, resid)
            print(f"[fit] {name:11s}  residual = {resid:.5f}")
        if not results:
            print("[fit] all 5 methods failed; nothing to save")
            return None
        best = min(results, key=lambda nm: results[nm][1])
        T_best, resid_best = results[best]
        print(f"\n[fit] best = {best}  (residual {resid_best:.5f})")

        # Outlier rejection: project board origin to base, drop samples
        # whose board pose disagrees > 4 robust-std from the median.
        outliers, devs, thresh = self._find_outliers(T_best, k_mad=4.0)
        Bs = self._board_in_base(T_best)
        spread = float(np.std(np.linalg.norm(Bs - Bs.mean(0), axis=1)))
        print(f"\n[diag] board_in_base recovered spread = {spread*100:.2f} cm "
              f"(should be < 1 cm for good calibration)")
        for i, dev in enumerate(devs):
            tag = "  <-- OUTLIER" if (i + 1) in [o + 1 for o in outliers] else ""
            print(f"  #{i+1:02d} dev={dev*100:6.2f} cm{tag}")
        if outliers:
            keep = [i for i in range(len(self.R_base_from_wrist_list))
                    if i not in outliers]
            print(f"\n[fit] dropping {len(outliers)} outliers, "
                  f"refitting with {len(keep)} samples...")
            results2 = {}
            Rw = [self.R_base_from_wrist_list[i] for i in keep]
            tw = [self.t_base_from_wrist_list[i] for i in keep]
            Rc = [self.R_cam_from_board_list[i] for i in keep]
            tc = [self.t_cam_from_board_list[i] for i in keep]
            for name, m in methods:
                try:
                    R, t = cv2.calibrateHandEye(Rw, tw, Rc, tc, method=m)
                except cv2.error as e:
                    print(f"[fit2] {name:11s}  CRASHED: {str(e).splitlines()[-1]}")
                    continue
                T = np.eye(4); T[:3, :3] = R; T[:3, 3] = np.asarray(t).flatten()
                resid = self._handeye_residual(T, idxs=keep)
                results2[name] = (T, resid)
                print(f"[fit2] {name:11s}  residual = {resid:.5f}  "
                      f"|t|={np.linalg.norm(T[:3,3])*100:.1f}cm")
            if results2:
                best2 = min(results2, key=lambda nm: results2[nm][1])
                T_best2, resid_best2 = results2[best2]
                Bs2 = self._board_in_base(T_best2, idxs=keep)
                spread2 = float(np.std(np.linalg.norm(Bs2 - Bs2.mean(0), axis=1)))
                print(f"\n[fit2] best = {best2}  (residual {resid_best2:.5f}, "
                      f"board spread {spread2*100:.2f}cm)")
                if resid_best2 < resid_best:
                    results = results2
                    best, T_best, resid_best = best2, T_best2, resid_best2
        return T_best, best, resid_best, results

    def _handeye_residual(self, X, idxs=None):
        """RMS frobenius norm of (A_ij X - X B_ij) over all pose pairs."""
        if idxs is None:
            idxs = list(range(len(self.R_base_from_wrist_list)))
        Tw, Tc = [], []
        for i in idxs:
            tw = np.eye(4); tw[:3, :3] = self.R_base_from_wrist_list[i]
            tw[:3, 3] = self.t_base_from_wrist_list[i]
            tc = np.eye(4); tc[:3, :3] = self.R_cam_from_board_list[i]
            tc[:3, 3] = self.t_cam_from_board_list[i]
            Tw.append(tw); Tc.append(tc)
        errs = []
        n = len(Tw)
        for i in range(n):
            for j in range(i + 1, n):
                A = np.linalg.inv(Tw[i]) @ Tw[j]
                B = Tc[i] @ np.linalg.inv(Tc[j])
                errs.append(float(np.linalg.norm(A @ X - X @ B)))
        return float(np.sqrt(np.mean(np.square(errs)))) if errs else float("inf")

    def _board_in_base(self, X, idxs=None):
        """Each sample's recovered board ORIGIN in base frame. Should be
        identical across samples for a stationary board; spread tells us
        the calibration quality directly in meters."""
        if idxs is None:
            idxs = list(range(len(self.R_base_from_wrist_list)))
        Bs = []
        for i in idxs:
            Tw = np.eye(4); Tw[:3, :3] = self.R_base_from_wrist_list[i]
            Tw[:3, 3] = self.t_base_from_wrist_list[i]
            Tcb = np.eye(4); Tcb[:3, :3] = self.R_cam_from_board_list[i]
            Tcb[:3, 3] = self.t_cam_from_board_list[i]
            Bs.append((Tw @ X @ Tcb)[:3, 3])
        return np.array(Bs)

    def _find_outliers(self, X, k_mad=4.0):
        """Return sample indices whose board_in_base position is > k * MAD
        from the median (i.e. their detection or joint reading disagrees
        with the rest). MAD is more robust than std when there ARE outliers.
        """
        idxs = list(range(len(self.R_base_from_wrist_list)))
        Bs = self._board_in_base(X, idxs)
        median = np.median(Bs, axis=0)
        devs = np.linalg.norm(Bs - median, axis=1)
        mad = np.median(np.abs(devs - np.median(devs))) + 1e-6
        # convert MAD to robust std estimate (×1.4826)
        robust_std = 1.4826 * mad
        threshold = k_mad * robust_std
        return [idxs[i] for i, d in enumerate(devs) if d > threshold], devs, threshold

    def save_result(self, T, method, residual, all_results):
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        base = os.path.join(self.args.out_dir, f"handeye_{self.arm}_{ts}")
        payload = {
            "arm": self.arm,
            "method_chosen": method,
            "residual_axb": float(residual),
            "num_samples": int(len(self.sample_meta)),
            "T_wrist_from_camera": {
                "matrix": T.tolist(),
                "translation_m": T[:3, 3].tolist(),
                "rotation_rpy_rad": [float(x) for x in euler_from_matrix(T)],
            },
            "board": {
                "cols_inner": int(self.args.board_cols),
                "rows_inner": int(self.args.board_rows),
                "square_size_m": float(self.args.square_size_m),
            },
            "intrinsics": {
                "K": self.K.tolist(),
                "D": self.D.tolist(),
                "image_topic": f"/camera_{ARM_CAM_SHORT[self.arm]}/color/image_raw",
            },
            "samples": self.sample_meta,
            "all_methods": {
                name: {
                    "matrix": all_results[name][0].tolist(),
                    "residual_axb": float(all_results[name][1]),
                } for name in all_results
            },
        }
        with open(base + ".json", "w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2)
        with open(base + ".yaml", "w", encoding="utf-8") as f:
            yaml.safe_dump({
                "arm": self.arm,
                "method": method,
                "residual_axb": float(residual),
                "num_samples": int(len(self.sample_meta)),
                "T_wrist_from_camera": T.tolist(),
                "board_cols_inner": int(self.args.board_cols),
                "board_rows_inner": int(self.args.board_rows),
                "square_size_m": float(self.args.square_size_m),
            }, f, sort_keys=False)
        print(f"\n[save] {base}.json")
        print(f"[save] {base}.yaml")
        return base

    # ---------- Loop ----------
    def print_help(self):
        print(
            "\nControls:\n"
            "  w/s : x + / -   (in arm base frame, meters)\n"
            "  a/d : y + / -\n"
            "  r/f : z + / -\n"
            "  u/o : roll  - / +\n"
            "  i/k : pitch + / -\n"
            "  j/l : yaw   + / -\n"
            "  c   : CAPTURE this pose (try to detect board)\n"
            "  z   : RESET arm to INITIAL_ARM_JOINTS\n"
            "  ?   : show this help\n"
            "  q   : quit + run hand-eye solver\n"
            f"\n  pos_step = {self.args.pos_step} m, rpy_step = {self.args.rpy_step} rad\n"
            "  Vary the camera pose between captures (different distances + angles).\n"
            "  Keep the board STATIONARY. Aim for >= 15 valid captures.\n"
        )

    def run(self):
        os.makedirs(self.args.out_dir, exist_ok=True)
        print(f"\n[setup] arm={self.arm}, "
              f"board {self.args.board_cols}x{self.args.board_rows} inner corners "
              f"@ {self.args.square_size_m*1000:.0f}mm, target {self.args.samples} samples")
        if self.teach_mode:
            print("[setup] TEACH MODE — motors off, move the arm by hand. "
                  "Jog keys are disabled; only c / z / q / ? are active.")
        print("[setup] waiting for puppet/joint, camera image, camera_info...")
        if not self.wait_feedback():
            print("[setup] timed out. Is the right launch + multi_camera running?")
            return
        print(f"[setup] K = {self.K.flatten().round(3).tolist()}")
        print(f"[setup] D = {self.D.round(4).tolist()}")

        # In active mode we ramp the motor-on arm back to a known anchor first;
        # in teach mode the operator already holds the arm wherever they want.
        if not self.teach_mode:
            if not self.reset_arm():
                print("[setup] reset failed, abort")
                return

        self.print_help()
        with RawKeyboard() as kb:
            while not rospy.is_shutdown():
                k = kb.read_key()
                if k is None:
                    continue
                k = k.lower()
                if k == "q":
                    break
                if k == "?":
                    self.print_help(); continue
                if k == "c":
                    self.try_capture()
                    if len(self.R_base_from_wrist_list) >= self.args.samples:
                        print(f"\n[cap] reached {self.args.samples} samples; finishing.")
                        break
                    continue
                if k == "z":
                    if self.teach_mode:
                        print("\n[teach] motors are off; reset not available. "
                              "Move the arm back by hand if needed.")
                    else:
                        self.reset_arm()
                    continue

                # Jog keys (motor-on only). In teach mode we just ignore.
                if self.teach_mode:
                    continue

                step_p = self.args.pos_step
                step_r = self.args.rpy_step
                if k == "w":   self.target_xyz[0] += step_p
                elif k == "s": self.target_xyz[0] -= step_p
                elif k == "a": self.target_xyz[1] += step_p
                elif k == "d": self.target_xyz[1] -= step_p
                elif k == "r": self.target_xyz[2] += step_p
                elif k == "f": self.target_xyz[2] -= step_p
                elif k == "u": self.target_rpy[0] -= step_r
                elif k == "o": self.target_rpy[0] += step_r
                elif k == "i": self.target_rpy[1] += step_r
                elif k == "k": self.target_rpy[1] -= step_r
                elif k == "j": self.target_rpy[2] += step_r
                elif k == "l": self.target_rpy[2] += step_r * -1
                else:
                    continue
                self.solve_and_publish()

        print(f"\n[done] {len(self.R_base_from_wrist_list)} samples collected")
        if len(self.R_base_from_wrist_list) < 3:
            print("[done] < 3 samples, not enough to solve; exit")
            return
        out = self.calibrate()
        if out is None:
            return
        T, method, resid, all_results = out
        self.save_result(T, method, resid, all_results)


DEFAULT_URDF = (
    "/home/agilex/cobot_magic/Piper_ros_private-ros-noetic/src/"
    "piper_description/urdf/piper_description_new.urdf"
)


def get_args():
    p = argparse.ArgumentParser(description="Hand-eye calibration for a Piper wrist camera")
    p.add_argument("--arm", type=str, default=None, help="l/m/r or left/mid/right")
    p.add_argument("--urdf", type=str, default=DEFAULT_URDF)
    p.add_argument("--out_dir", type=str,
                   default=os.path.join(HERE, "calibration_outputs"))
    p.add_argument("--samples", type=int, default=20,
                   help="target number of valid captures before auto-quit")
    p.add_argument("--board_cols", type=int, default=7,
                   help="inner corners along the LONGER side (8 squares -> 7)")
    p.add_argument("--board_rows", type=int, default=5,
                   help="inner corners along the SHORTER side (6 squares -> 5)")
    p.add_argument("--square_size_m", type=float, default=0.025,
                   help="checkerboard square edge in meters (A4 print = 0.025)")
    p.add_argument("--pos_step", type=float, default=0.01,
                   help="Cartesian xyz step per key press, meters")
    p.add_argument("--rpy_step", type=float, default=0.05,
                   help="orientation step per key press, radians")
    p.add_argument("--gripper", type=float, default=INITIAL_GRIPPER)
    p.add_argument("--ramp_speed_rad_s", type=float, default=0.3)
    p.add_argument("--hz", type=float, default=30.0)
    p.add_argument("--teach_mode", action="store_true",
                   help="Motors are off (mode=0 auto_enable=false). Skip publishing / "
                        "resetting; operator moves the arm by hand and presses c to "
                        "capture each pose. Highly recommended for hand-eye calibration "
                        "— faster + better pose diversity than keyboard jog.")
    return p.parse_args()


def main():
    args = get_args()
    args.arm = resolve_arm(args.arm)
    HandEyeCalibrator(args).run()


if __name__ == "__main__":
    main()
