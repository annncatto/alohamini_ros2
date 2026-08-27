# alohamini_joycon_teleop

Joy-Con Cartesian teleoperation for AlohaMini. The realtime arm path is:

```text
one HID reader / Joy-Con
  -> raw accel + gyro + buttons + sticks (50 Hz, timestamped ZMQ)
  -> relative quaternion + faucet XYZ command
  -> exact URDF-derived DH FK/Jacobian + damped differential IK (50 Hz)
  -> standard arm JointJog
  -> guarded LeRobot bridge
  -> Host trajectory and tracking supervision
```

MoveIt KDL/LMA services and `FollowJointTrajectory` remain available for
planning and regression, but point-IK services and Action preemption are not in
the default realtime teleoperation loop.

## Preview and hardware

```bash
source /opt/ros/humble/setup.bash
source ~/alohamini_ros2/install/setup.bash

# No robot commands.
ros2 launch alohamini_joycon_teleop preview.launch.py

# Unified hardware graph. Replace ROBOT_IP with the Pi address.
ros2 launch alohamini_bringup hardware.launch.py host:=ROBOT_IP \
  enable_moveit:=true enable_joycon:=true use_rviz:=true
```

Hardware commands stay disabled by default. After checking the measured pose,
joint directions and clear workspace, enable them explicitly:

```bash
ros2 service call /alohamini_lerobot_bridge/command_enable \
  std_srvs/srv/SetBool '{data: true}'
```

## Controls

The left Joy-Con controls the left arm and the right one controls the right
arm. The arm-base axes are `+X = robot left`, `-Y = robot forward`, `+Z = up`.

- Hold either rail button (`SL` or `SR`) to engage that arm.
- While engaged, stick up/down moves along the controller's pointing direction
  and stick left/right moves along its rotated lateral direction. This is the
  faucet/nozzle mode: it commands TCP XYZ displacement in the arm-base frame,
  not a translation of the TCP coordinate frame.
- While engaged, rotating the Joy-Con controls TCP orientation relative to the
  clutch attitude. Releasing the rail button freezes the current TCP target.
- While engaged, shoulder is `+Z`; stick-click is `-Z`.
- With no rail clutch held, X/B and Y/A (d-pad equivalents on the left) provide
  fixed forward/back and left/right Cartesian nudging for axis checks.
- Shoulder + X/B provides fixed up/down nudging. Shoulder + Y/A is unassigned;
  it never also emits the plain left/right command. Fixed Cartesian nudges may
  be blended with faucet control while a rail clutch is held. A newly engaged
  arm gesture re-latches the measured TCP pose.
- Fixed face-button XYZ gestures also accept simultaneous relative Joy-Con
  orientation. In SL/SR faucet mode L/stick-click already provide +/-Z, so L
  does not remap a simultaneously held face button; the axes compose instead.
- Differential DLS applies `orientation_axis_signs` to the relative quaternion;
  the Joy-Con roll and yaw axes are mirrored so physical rotations command the
  gripper in the same visual direction instead of the opposite direction.
- ZL/ZR toggles the corresponding gripper. A press re-arms only after a
  continuously confirmed release, so isolated false HID reports cannot issue
  an unintended second toggle.
- Capture/Home re-latches measured FK and the Joy-Con attitude.
- Without an arm rail clutch, sticks control the mobile base. Right shoulder +
  stick horizontal controls base yaw. Left shoulder + stick vertical controls
  lift after the stick leaves its deadzone, unless an arm rail clutch is held.

Joy-Con has no magnetometer. Roll/pitch are gravity-corrected; yaw is gyro-only
and can drift, so yaw is intentionally used as a relative clutch motion and can
be re-centered by releasing/re-engaging or Capture/Home.

Command buttons use immediate press recognition and a short release grace so an
isolated false HID report cannot alternate a held left stick between lift and
base control. The same filter is applied symmetrically to both Joy-Cons. Once an
L+vertical-stick lift gesture starts, it remains assigned to lift until the
stick returns to its deadzone; it cannot leak into base translation mid-gesture.

Hardware mode keeps the input and IK loop at 50 Hz while using a bounded 120 ms
joint-position look-ahead. This overcomes servo friction/backlash without
removing the Bridge tracking gate or Host trajectory shaping. Preview mode uses
the real timer step because simulated joints have no lag. Velocity and position
bounds scale the complete DLS joint vector, preserving its direction instead of
independently clipping joints and introducing Cartesian kinks.

Tune the face-button/faucet Cartesian speed with `tcp_speed_m_s` and the
relative orientation speed with
`orientation_speed_rad_s` independently without changing the bounded DLS/Host
safety layers.

## Raw input record and deterministic replay

Each schema-v2 sample contains the Nintendo report counter, monotonic timestamp,
all three accelerometer and gyro samples, sticks, buttons, solved RPY, and a
normalized XYZW quaternion. Recording runs in a separate process so disk I/O
cannot stall the HID reader:

```bash
ros2 run alohamini_joycon_teleop joycon_input_log record ~/joycon_raw.ndjson
ros2 run alohamini_joycon_teleop joycon_input_log replay ~/joycon_raw.ndjson
```

The reader uses one HID handle per physical Joy-Con. A frozen report is detected
from the hardware report counter, not from unchanged stick/pose values; a
stationary controller is therefore never mistaken for a disconnect.

## Kinematics and MuJoCo

The standard-DH table in
`alohamini_description/config/kinematics/right_arm_kinematics.yaml` is an exact
representation of the six-axis URDF chain. Both arms share the chain in their
own `{side}_Base`; side-specific URDF limits are applied. Validation compares
DH TCP FK with independent URDF golden transforms.

MuJoCo is optional and installed in the `lerobot_alohamini` conda environment:

```bash
source /opt/ros/humble/setup.bash
source ~/alohamini_ros2/install/setup.bash
conda run -n lerobot_alohamini python -m \
  alohamini_joycon_teleop.mujoco_viewer --side right --sweep
```

The viewer shows DH link segments, the TCP frame (RGB axes), and target marker.
Use `--joints PAN LIFT ELBOW FLEX YAW ROLL` for a measured configuration.

The realtime solver is damped least squares with adaptive damping near
singularities, XYZ priority (`dls_orientation_weight: 0.35`), side-specific
joint limits, joint velocity limiting, and a per-cycle step limit. Lowering the
orientation weight lets wrist orientation yield when a full 6D pose is locally
infeasible; all six joints already participate in every Cartesian-axis solve,
so no hard-coded one-axis-to-one-joint mapping is required.

Set `arm_control_mode: moveit` only to compare the former KDL -> LMA ->
position-only point-IK path.

## Diagnosing a remaining pause

Every five seconds the teleop reports:

```text
Joy-Con timing: right dropped=0 max_age=31.2ms max_dls=1.4ms, max_loop_dt=34.0ms
```

- increasing `dropped` or large `max_age`: HID/Bluetooth or reader scheduling;
- large `max_dls`: local CPU/solver load;
- normal input/DLS but large `max_loop_dt`: ROS executor scheduling;
- all three normal but physical motion pauses: inspect bridge diagnostics and
  run the Pi Host with `--profile_timing true` to inspect bus/Host cycle jitter.

The bridge's arm `JointJog` is latest-only and expires after 150 ms. It still
uses fresh measured state, path-error gates, calibration mapping, the Host
watchdog, Host trajectory limits and Host tracking supervision. Stale input or
feedback cannot keep advancing a target.
