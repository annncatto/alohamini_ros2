# Shared physical runtime boundary

## Authority and bus ownership

`~/lerobot_alohamini` contains the currently verified AlohaMini Runtime/Host.
It owns physical motor serial ports, executes commands, reads state, and
enforces the watchdog.

`~/alohamini_lidar_imu` contains a ROS bridge and a separate experimental C++
`ros2_control` base backend. The C++ backend is an alternative hardware
backend, not part of the current shared Runtime. It must never run while the
LeRobot Host owns the same motor bus.

## Current ZMQ compatibility

| Port | Host | Required client | Old lidar bridge | Status |
|---|---|---|---|---|
| 5555 command | `PULL`, conflated JSON actions | `PUSH` | `PUSH` | compatible |
| 5556 observation | request-driven `ROUTER` | `DEALER`, sends request token | passive `PULL` | replaced here |

The latest full 5556 response is multipart:

1. request token;
2. JSON state frame;
3. zero or more camera-name/JPEG frame pairs.

The Host-side `ROUTER` envelope carries the DEALER identity in addition to
those client-visible frames. State-only requests use a token ending in
`:state`; their client-visible response has exactly two frames (token and
JSON), sets `_images=[]`, and carries no JPEG data. Full observation requests
use `:full` and may append camera-name/JPEG pairs.

The old lidar bridge neither requests observations nor decodes multipart
responses, so it cannot receive fresh observations from the latest Host. With
`require_observation_for_motion=true`, its safety gate therefore selects a
zero body-velocity command continuously even though the 5555 command socket is
otherwise compatible. Disabling that gate would remove the symptom but would
not repair observation or odometry, so it is not a protocol fix.

`alohamini_lerobot_bridge` implements the current DEALER request/reply path. It
uses `:state` requests, publishes real Host arm/lift feedback after converting
Host normalization through the supplied motor metadata and authoritative ROS
calibration, and derives wheel joint velocity from measured base velocity for
joint visualization. It does not integrate a base pose.

On `/joint_states`, measured arm/lift joints and derived wheels are never mixed
in one message. The measured partial message is mirrored at
`~/measured_joint_states`; the wheel-only partial message is mirrored at
`~/derived_wheel_states`. Wheel velocity is reconstructed from measured body
velocity and wheel position is integrated only for visualization, so neither
is raw wheel-encoder position or odometry. The same derived partial message
holds the three MoveIt virtual-root joints at zero so robot_state_publisher can
form `root -> base_link`; these constants are not a pose estimate.

The ROS command socket is disabled by default. Port 5555 has no ownership lease,
so enabling ROS commands is an explicit operation and requires every other
LeRobot command client to be stopped. Read-only observation does not compete
for the motor bus or send zero commands. Enabling clears all commands received
while disabled and sends no 5555 frame until a new `/cmd_vel` arrives.

This bridge version deliberately has no lidar, external odometer, or IMU input.
It does not launch SLAM, Nav2, Servo, or a state estimator. Host-measured body
velocity is published directly; `/odom` and `odom -> base_link` are not
published.
