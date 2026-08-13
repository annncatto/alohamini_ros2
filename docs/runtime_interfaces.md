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
| 5556 observation | request-driven `ROUTER` | `DEALER`, sends request token | passive `PULL` | incompatible |

The latest 5556 response is multipart:

1. request token;
2. JSON state frame;
3. zero or more camera-name/JPEG frame pairs.

The Host-side `ROUTER` envelope carries the DEALER identity in addition to
those client-visible frames. State-only requests use a token ending in
`:state`; full observation requests use `:full`.

The old lidar bridge neither requests observations nor decodes multipart
responses, so it cannot receive fresh observations from the latest Host. With
`require_observation_for_motion=true`, its safety gate therefore selects a
zero body-velocity command continuously even though the 5555 command socket is
otherwise compatible. Disabling that gate would remove the symptom but would
not repair observation or odometry, so it is not a protocol fix.

This document records the current interface boundary only. The bridge is not
being migrated in the initial description/MoveIt consolidation.
