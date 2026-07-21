# AGENTS.md

## Cursor Cloud specific instructions

This repo (`Piper Dual-Arm/3-Arm Teleop`) is a ROS Noetic robotics teleop + imitation-learning
data-collection stack for AgileX Piper arms. On real robots everything runs inside the conda env
`aloha` (Python 3.8) with a system ROS Noetic install; see `Readme.md` for the on-robot startup
sequence and frame conventions.

### What can and cannot run in the cloud VM

The cloud VM has NO ROS, NO robot arms, NO cameras, and NO Apple Vision Pro. So anything importing
`rospy` / `cv_bridge` / ROS message packages, the `.launch` files, the CAN/GoPro shell scripts, and
the AVP Vuer server cannot run here. Only the **pure-computation** paths run headless:
- FK / IK helpers and coordinate transforms: `teleop/piper_fk.py`, `teleop/arm_unified_coords.py`,
  `teleop/unified_eef_pose.py`, `teleop/arm_camera_in_unified.py`.
- Frame visualizer `teleop/frame_visualize.py` (file output only).

The env is set up with the **system Python 3.12** (no conda, no ROS). Deps install to the user site
(`~/.local`) via the update script. Versions differ from the on-robot `aloha` env; the pure kinematic
math is version-robust.

### Non-obvious gotchas

- **`tf.transformations` shim**: There is no ROS, so `tf.transformations` is provided by an
  auto-generated shim in the user site-packages `tf/` dir (created by the update script). It wraps the
  PyPI `transformations` package but re-orders quaternions to ROS `(x, y, z, w)`. The bare
  `transformations` package uses `(w, x, y, z)` — do NOT `import transformations` directly for
  quaternion work; always use `tf.transformations`. If imports of `tf` fail, re-run the update script.
- **FK needs the `piper_ros` submodule meshes**: `PiperFkModel` calls `pin.RobotWrapper.BuildFromURDF`,
  which builds the geometry model and therefore requires the STL meshes under
  `piper_ros/src/piper_description/meshes/`. The update script runs `git submodule update --init piper_ros`.
  Without those meshes you get `ValueError: Mesh package://piper_description/meshes/*.STL could not be found`.
- **Hardcoded extrinsics path**: `arm_unified_coords.py` `DEFAULT_EXTRINSICS_JSON` points at an on-robot
  absolute path (`/home/agilex/...`). When running unified-frame converters here, pass a local file,
  e.g. `UnifiedEefConverter(extrinsics_json="teleop/calibration_outputs/arm_extrinsics_touch_20260629_131357.json")`.

### Test / lint / run

- Tests (stdlib `unittest`, pytest-compatible) live in `tests/` and must run from the repo root:
  `python3 -m pytest tests/`. `tests/test_camera_fov.py` and `tests/test_postprocess_camera_poses.py`
  import `data_collect/collect_data_3arm.py`, which imports `rospy`; they FAIL in the cloud VM by
  design (ROS-only) — this is expected, not a regression. The other 5 files (13 tests) pass headless.
- No linter is configured in the repo (the `# noqa` comments have no committed flake8/ruff config).
  Use `python3 -m py_compile teleop/*.py data_collect/*.py avp/*.py tests/*.py` as a syntax baseline.
- `frame_visualize.py` has no display server; use `--save <path>.png` (matplotlib Agg) or
  `--html <path>.html` (plotly) for output — an interactive window will not open.
