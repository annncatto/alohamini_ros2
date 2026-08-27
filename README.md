# AlohaMini ROS 2

This workspace owns the authoritative ROS-facing robot description and the
ROS 2 algorithm interfaces for AlohaMini. It does not replace LeRobot.

LeRobot and ROS 2 are parallel upper layers over the same physical runtime:

- LeRobot owns teleoperation, recording, datasets, replay, policy training,
  VLA inference, action chunks, evaluation, camera observations, and local or
  optional Hugging Face assets.
- ROS 2 owns the authoritative description, TF, FK/IK, MoveIt, Servo,
  standard trajectory interfaces, Nav2, state estimation, diagnostics,
  simulation, and other algorithm interfaces.
- The Python 3.12 LeRobot runtime and ROS 2 Humble/Python 3.10 runtime remain
  isolated at a ZMQ boundary. `alohamini_lerobot_bridge` implements the current
  DEALER/multipart observation path and the explicitly enabled command path.

The verified physical Runtime/Host lives in `~/lerobot_alohamini`. It is the
only current authority for motor serial ownership, command execution, state
reads, and the hardware watchdog. See [docs/runtime_interfaces.md](docs/runtime_interfaces.md)
for the current bridge compatibility boundary.

Robot-description ownership is one-way: this repository publishes the public
description and exports. ManiSkill and RoboTwin keep their own environments,
tasks, controllers, and simulation adapters, and consume exported assets
without writing changes back into the authoritative description. See
[docs/description_ownership.md](docs/description_ownership.md).

## Current scope

The core packages are `alohamini_description`, `alohamini_calibration`,
`alohamini_camera`, `alohamini_moveit_config`, `alohamini_lerobot_bridge`,
`alohamini_joycon_teleop`, and `alohamini_bringup`. They establish one upstream location for the
CAD-derived whole-robot model, calibration assets, mesh assets, MoveIt
semantics, guarded physical interfaces, and Joy-Con teleoperation.
`~/alohamini_lidar_imu` remains the source of the ROS bridge and an alternative
experimental C++ `ros2_control` base backend; it is not the shared physical
Runtime authority.

The current whole-robot description uses `root` as its planning reference and
preserves RoboTwin/CAD arm link and joint coordinates. Its current AlohaMini2Pro
wheel CAD is exposed through continuous ROS wheel joints and the verified
LeRobot runtime wheel ordering. `base_link` follows REP-103 (+x forward, +y
left, +z up); the preserved CAD subtree is attached through the fixed
`base_cad_link` adapter.

Build with:

```bash
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
ros2 launch alohamini_description description.launch.py
ros2 launch alohamini_moveit_config plan_only.launch.py
ros2 launch alohamini_joycon_teleop preview.launch.py

# Unified physical runtime, TF and cameras; commands remain disabled by default:
ros2 launch alohamini_bringup hardware.launch.py host:=127.0.0.1

# Add MoveIt and Joy-Con without starting another Bridge or state publisher:
ros2 launch alohamini_bringup hardware.launch.py host:=127.0.0.1 \
  enable_moveit:=true enable_joycon:=true use_rviz:=true
```

Offline validation:

```bash
ros2 run alohamini_validation validate_assets
ros2 launch alohamini_description view_model.launch.py

# Headless TF check, in two terminals:
ros2 launch alohamini_description view_model.launch.py use_gui:=false use_rviz:=false
ros2 run alohamini_validation validate_tf

# MoveIt service check, in two terminals:
ros2 launch alohamini_moveit_config plan_only.launch.py use_rviz:=false
ros2 run alohamini_validation validate_moveit
```
