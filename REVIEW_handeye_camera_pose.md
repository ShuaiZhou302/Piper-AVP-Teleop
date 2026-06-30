# Review: Hand-eye calibration + camera pose in HDF5

## What this PR does
Calibrates the three wrist-mounted RealSense cameras (`T_wrist_from_camera` per arm), then post-processes every collected episode so each frame ships with the camera pose expressed in the shared unified arm frame. Downstream consumers (point-cloud fusion, active perception) get a ready-to-use 6D camera pose per frame per camera without re-running FK.

## Pose chain
```
T_unified_from_camera(arm, q) = T_unified_from_arm_base[arm]   (touch-point extrinsics)
                              @ FK_arm(q)                       (pinocchio, ee frame)
                              @ T_wrist_from_camera[arm]        (this PR's hand-eye result)
```
Unified frame = mid_base orientation, origin 25 cm below mid_base along its Z (existing convention).

## Files

### New
| File | Purpose |
|---|---|
| `teleop/calibrate_3arm_camera_handeye.py` | One-arm-at-a-time hand-eye calibration. Keyboard jog, chessboard PnP, all 5 `cv2.calibrateHandEye` methods, MAD-based outlier rejection, refit. Saves JSON+YAML with full per-sample metadata. |
| `teleop/arm_camera_in_unified.py` | `WristCameraUnifiedConverter` — loads the touch-point extrinsics + latest hand-eye JSON per arm + Pinocchio URDF FK, exposes `camera_pose_in_unified(arm, q)` and a batch variant. |
| `data_collect/postprocess_camera_poses.py` | Offline post-process for old `episode_*.hdf5` files. Rewrites `observations/camera_pose_in_unified` using the same helper as new collection. |
| `teleop/calibration_outputs/handeye_{mid,left,right}_*.json/.yaml` | Calibration results. Auto-discovered by timestamp suffix. |

### Modified
- `data_collect/collect_data_3arm.py` — adds `_compute_camera_poses_in_unified()` helper and a post-process step inside `save_data()`. Writes a new HDF5 group `observations/camera_pose_in_unified/<cam>/{quat,rpy,matrix}`. Wrapped in try/except so a missing calibration or import failure WARNS but never blocks the episode save.
- 5 other files in the diff are unrelated prior work (AVP HUD, gesture FSM, video extract, etc.) — not the focus here.

## HDF5 addition (backward-compatible — old episodes simply lack this group)
```
observations/
  camera_pose_in_unified/
    attrs: frame, extrinsics_json, urdf, unified_origin_in_mid_m,
           handeye_mid, handeye_left, handeye_right
    cam_front/   wrist_arm=mid    quat(T,7)  rpy(T,6)  matrix(T,4,4)
    cam_left/    wrist_arm=left   ...
    cam_right/   wrist_arm=right  ...
```
- `quat` = (x, y, z, qx, qy, qz, qw)
- `rpy` = (x, y, z, roll, pitch, yaw)
- `matrix` = full 4×4 SE(3)

## Calibration quality (cross-arm cross-check)

| arm | \|t\| (cm) | xyz (m) | rpy (deg) | residual | board spread |
|---|---|---|---|---|---|
| MID   | 8.72 | (+0.034, +0.006, +0.080) | (-110, -4, -90) | 0.077 | 1.08 cm |
| LEFT  | 8.24 | (+0.048,  0.000, +0.067) | (-111, -2, -90) | 0.095 | 0.80 cm |
| RIGHT | 8.53 | (+0.053, +0.010, +0.066) | (-112, -3, -90) | 0.075 | 1.11 cm |

Cross-arm std: **7.8 mm in translation, 0.74° in rpy**. Same camera model, same mounting → near-identical extrinsics ⇒ strong cross-validation.

## Things to please look at carefully

1. **Import order before ROS/tf** — `collect_data_3arm.py` imports conda `pinocchio` before `rospy/tf`, and `arm_camera_in_unified.py` imports `pinocchio` before `tf.transformations`. If `tf` loads system `libstdc++` first, conda Pinocchio/CasADi can fail with missing `GLIBCXX_3.4.29`. This is an import-order problem inside the active `aloha` conda env, not the wrong Python.

2. **Post-process timing in `collect_data_3arm.py`** — runs *after* recording ends and *before* `h5py.File(...)` write, inside `save_data()`. Chosen to keep the real-time teleop loop untouched. Confirm this placement is acceptable (alternative would be a separate offline script).

3. **Camera-to-arm mapping** in `collect_data_3arm.py`:
   ```python
   CAM_TO_ARM = {"cam_front": "mid", "cam_left": "left", "cam_right": "right"}
   ```
   This must match the physical mounting AND the camera-launch topic prefixes (`/camera_{f,l,r}`). Please sanity-check vs. the hardware.

4. **qpos layout assumption** — `left[0:7] right[7:14] mid[14:21]`. Matches `ARM_ORDER = ("left","right","mid")`. Code uses `arm_slice` dict to extract per-arm joints — verify both halves agree.

5. **Outlier rejection in `calibrate_3arm_camera_handeye.py:~470`** — projects each sample's board origin to base via the candidate X, drops samples > 4 × MAD from the median, refits. Big quality lift (LEFT went 0.96 → 0.095 residual after dropping 2/20 samples). Worth confirming the MAD threshold (4.0) and that the refit logic actually replaces results only when residual improves.

6. **Classic `findChessboardCorners` not `findChessboardCornersSB`** in calibrate script — SB silently flips corner order at wrist roll > ~90°, destroying calibration. Classic is more deterministic but needs the operator to keep roll moderate. Comment in `_find_corners()` explains.

7. **Graceful degradation** — `_compute_camera_poses_in_unified()` returns `(None, None)` on any failure and `save_data()` skips the group. Please confirm this is the right level of strictness (vs. failing hard so a broken calibration doesn't silently produce data without the field).

## Out of scope for this review
- `data_collect/play_data_eef_ik.py` and `tests/test_play_data_eef_ik.py` — separate EE-pose replay line of work, can be reviewed together if convenient.
- `data_collect/hpc3_sample_review/` — local cache from HPC, not committed.
- The upload shell uses `${HPC3_REMOTE_PASS}` / prompt input and does not contain the literal password.
