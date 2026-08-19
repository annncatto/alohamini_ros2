# alohamini_joycon_teleop

Joy-Con Cartesian teleoperation through the authoritative AlohaMini ROS 2
description, MoveIt IK, and the guarded LeRobot bridge. The native reader only
reads HID input and publishes JSON on localhost; it imports neither LeRobot nor
ROS and cannot connect to the robot Host.

Preview is the default and does not publish robot commands:

```bash
ros2 launch alohamini_joycon_teleop preview.launch.py
```

The two Joy-Cons must already be paired and connected. The installed native
Python can be overridden with `native_python:=...`.

## Controls

The left Joy-Con (L) drives the left arm and the right Joy-Con (R) drives the
right arm. The left Joy-Con uses its d-pad where the right one uses X/B/Y/A.

- Sticks (either hand, sums are clamped): base translation. Up = forward,
  left/right = strafe.
- R + right stick left/right: base turn (right = clockwise).
- L + left stick up/down: lift axis up / down. While L is held the left stick
  contributes no base translation, so lift and drive never mix accidentally.
  After release the node freezes the last commanded height and keeps holding
  it: the Host stops driving the lift when the trajectory stream ends, so the
  teleop re-commands whenever the measured lift sinks beyond
  `lift_hold_tolerance_m` (`lift_hold: true`).
- X (d-pad up) / B (d-pad down): that arm TCP forward / backward.
- Y (d-pad left) / A (d-pad right): that arm TCP left / right.
  All four directions are in the robot root frame (+x = robot left,
  -y = robot forward), which both arm base frames share.
- Shoulder + X / B (L + d-pad up/down on the left, R + X/B on the right): that
  arm TCP up / down in the world frame (+z / -z; both arm base frames are
  root-aligned so this is world vertical).
- ZL / ZR: toggle the corresponding gripper.
- SL or SR + rotate the controller: adjust that arm TCP orientation. The
  control is incremental (the controller's absolute attitude is irrelevant),
  so releasing and re-pressing SL/SR continues from the current gripper
  orientation. Without SL/SR, tilting does nothing.
- Capture / Home: re-latch the current TCP pose and re-level the orientation
  reference (logged in the terminal; the green/orange TCP marker updates).

Face buttons are hold-to-move: releasing them stops new TCP targets and cancels
the in-flight arm trajectory in hardware mode. When no stick input is active no
`/cmd_vel` is published, and the last active base command is followed by one
zero.

After a base move in preview the node refreshes both arm base poses via FK so
the held TCP target follows the moved base instead of jumping to the old world
position.

The terminal prints rate-limited status for both failures and successes (once
per second per arm): `[right] IK OK [kdl] max step 0.013 rad` and
`[right] IK failed: NO_IK_SOLUTION (unreachable pose) [kdl]`. The native reader
also prints raw stick values when a stick moves
(`[left] stick H=3580 V=2048`), so stick health is visible without extra
tools.

## TCP orientation and Joy-Con IMU

The native reader still runs the library's two-second gyro-bias calibration at
startup. It only needs the controllers to be **still** — any orientation works,
so flat on the desk is fine. The reader also watches for a frozen HID input
report (the native library's read thread can die silently on a Bluetooth
hiccup, which leaves sticks/buttons stuck) and reopens the device
automatically.

The original `joycon-robotics` library offers no per-axis remapping or filter
settings that make raw tilt control comfortable (its default `common_rad`
scaling amplifies roll/pitch by about 2.1x and yaw drifts), so attitude is
controlled as follows: while an SL/SR rail button of that controller is held,
the commanded TCP attitude is the pose at press time rotated by the Joy-Con
attitude change since then, stepped at `orientation_speed_rad_s`
(0.8 rad/s default). The controller's absolute attitude is irrelevant, so
releasing and re-pressing SL/SR always continues from the current gripper
orientation. At the first latch the orientation optionally levels toward the
classic level-grasp pose (`imu_latch_reference: level`); the default is
`current`, which keeps the robot's TCP attitude so control works directly
from the folded Home.

- `orientation_scale`: rotation sensitivity (1.0 = native scaling).
- `orientation_deadband_rad`: minimum applied attitude delta per axis.
- `max_orientation_delta_rad`: per-axis clamp of the attitude delta so large
  controller rotations stay inside the wrist joint limits.
- `orientation_speed_rad_s`: maximum TCP rotation rate toward the commanded
  attitude.

## IK solver and reachability

Both arms use `kdl_kinematics_plugin/KDLKinematicsPlugin` from
`alohamini_moveit_config/config/kinematics.yaml`, with a three-tier fallback
chain per request: KDL → LMA (`{side}_arm_lma` groups) → position-only LMA
(`{side}_arm_pos` groups). Offline sweeps (192 targets per arm around the
folded Home, plus tilted-wrist seeds) found LMA and KDL identical on ordinary
targets, but KDL's Newton-Raphson solver fails 100% at the raised-arm
configurations (elbow_flex ≈ 2 rad with the wrist near the shoulder-pan axis),
where LMA still converges — that is the "solver fails at the current arm
position" symptom. Longer timeouts do not help. The fallback groups live in
`alohamini_moveit_config/config/kinematics.yaml` and
`alohamini_description/srdf/alohamini2pro.srdf`.

Failed IK requests print a rate-limited warning in the terminal (once per
second per arm), e.g. `[right] IK failed: NO_IK_SOLUTION (unreachable pose)
[kdl]`. Two automatic degradations keep the arm moving:

- Near singular configurations (wrist center close to the shoulder-pan axis,
  e.g. TCP straight ahead) both solvers can only reach the target through an
  elbow branch flip; after two consecutive identical rejections the node
  allows one bounded larger step (`max_ik_jump_rad`, 1.6 rad default) and
  scales the trajectory duration to `arm_goal_max_velocity_rad_s`, so the arm
  crosses the branch boundary instead of sticking.
- When a position move still cannot satisfy the held TCP orientation, the node
  retries through the `{side}_arm_pos` planning groups (same chains with
  `position_only_ik: true` in `alohamini_moveit_config/config/kinematics.yaml`)
  and re-latches the achieved pose, so position teleoperation keeps working
  when the 6-DOF pose is unreachable.

## Troubleshooting: laggy robot following on real hardware

The bridge paces its command stream to the Host loop: it sends at most one
command per fresh Host observation (the Host consumes one command and answers
one observation request per loop iteration). This keeps the Host-side command
queue at one, so the robot follows the joystick with a constant one-loop lag
instead of accumulating stale commands — without modifying the Host. If the
robot still moves in visible steps, the Host loop itself is slow (e.g. camera
reads); run the Host with `--profile_timing` to measure it, but no latency
threshold applies to the network.

## Troubleshooting: flickering robot model in RViz

The preview RViz reads `/tf` and `/robot_description` from the shared ROS
domain. If the robot model flickers between two poses (or two geometries),
another ROS graph on the same LAN is publishing competing TF/URDF data — e.g.
a leftover `joint_state_publisher` + `robot_state_publisher` on the robot Pi
from `alohamini_description` or an older MoveIt demo. Check the publishers:

```bash
ros2 topic info /tf -v          # expect only alohamini_plan_only_robot_state_publisher
ros2 node list                  # unexpected /joint_state_publisher, /robot_state_publisher, /rviz
```

Fix by stopping that launch on the other machine, or isolate the preview on
its own domain (the Joy-Con ZMQ channel is unaffected by the ROS domain):

```bash
ROS_DOMAIN_ID=102 ros2 launch alohamini_joycon_teleop preview.launch.py
```

Hardware mode must keep sharing the domain with the local
`alohamini_lerobot_bridge`, so do not set a private domain there.

## Hardware mode

Hardware mode starts the existing bridge read-only and does not call its
`command_enable` service:

```bash
ros2 launch alohamini_joycon_teleop hardware.launch.py host:=ROBOT_IP
```

For a robot with its own machine-specific arm mapping (produced by
`ros2 run alohamini_calibration sync_arm_mapping`), pass the directory:

```bash
ros2 launch alohamini_joycon_teleop hardware.launch.py \
  host:=ROBOT_IP arm_mapping_dir:=$HOME/.config/alohamini/ROBOT_IP
```

An empty `arm_mapping_dir` keeps the installed default mapping.

The RViz started by this launch uses the Joy-Con config (robot model + TCP
markers) without the MoveIt interactive TCP marker, which would fight the
Joy-Con controller. Disable it with `joycon_rviz:=false`.

The teleop node logs hardware readiness (fresh measured `/joint_states`, arm
action servers up) every 10 s. By default it then enables the bridge commands
itself (`auto_enable_commands: true`); set it to false to keep the manual flow:

```bash
ros2 service call /alohamini_lerobot_bridge/command_enable \
  std_srvs/srv/SetBool '{data: true}'
```

Without enabled commands the bridge rejects every arm/gripper/lift goal (the
node reports `bridge rejected the arm goal (commands disabled?)`). Disable the
commands again before stopping the session.

After the bridge is enabled, the node homes both arms once to the preview
reference pose (`home_to_preview_pose: true`): it sends a direct joint-space
trajectory to the reference joint values; the bridge converts URDF radians to
encoder values through its own calibration mapping. The trajectory duration
scales with the joint distance, the goal is retried once per second until the
bridge accepts it, and homing never runs while the operator is actively moving
an arm. If a homing goal is rejected, the bridge prints the rejection reason
in its own terminal output.

The native reader must never be replaced by the older direct
`AlohaMiniClient` Joy-Con script, because that would create a second
port-5555 command owner.
