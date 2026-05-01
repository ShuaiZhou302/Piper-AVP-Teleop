# Initial right-arm EE pose (Piper base frame) used as the AVP-teleop reset anchor.
# Format: (x, y, z, roll, pitch, yaw, gripper) — meters / radians / meters.
INITIAL_ARM_POSE = (0.065, -0.00, 0.38, 0.0, 0.0, 0.0, 0.1)

# this test we only teleop the right arm, so the left arm will stay still at its initlal pose. the right arm will be initialized with the above pose, and the teleop will be relative to this initial pose.

# TODO:
# 1. axis transformation from avp to piper
# 2. subscribe the piper right camera to the vr teleop topic, so i can see while doing teleoperation 
# 3. get pose from avp and call ik from /home/agilex/cobot_magic/aloha-devel/Piper-AVP-Teleop/teleop/eef_keyboard_control.py to get the joint angles, and then publish the joint angles to the piper right arm controller.
