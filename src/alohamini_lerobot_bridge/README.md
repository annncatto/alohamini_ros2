# alohamini_lerobot_bridge

ROS 2 state and command integration for the physical runtime hosted by
`lerobot_alohamini`. This package never opens a serial device. The LeRobot Host
remains the only owner of both motor buses and its watchdog remains the final
motion stop.

The bridge uses the current protocol:

- port 5555: PUSH command client to the Host PULL socket;
- port 5556: DEALER state requests to the Host ROUTER socket;
- observation response: request token, JSON state, then optional image pairs;
- this bridge requests `state` only and does not duplicate camera transport.

It publishes measured arm/lift state on `/joint_states`, measured Host body
velocity on `/alohamini/base_velocity`, wheel state derived from that measured
velocity, the received state on `~/state_json`, and health on `/diagnostics`.

This version does not connect lidar, an external odometer, or an IMU, and does
not launch SLAM, Nav2, or state estimation. It publishes neither `/odom` nor
`odom -> base_link` TF.

Commands are disabled by default. This matters because the Host's port 5555 has
no lease protocol: two PUSH clients would race in the Host PULL queue. After
stopping every other LeRobot command client and confirming fresh diagnostics:

```bash
ros2 service call /alohamini_lerobot_bridge/command_enable std_srvs/srv/SetBool '{data: true}'
```

Disable before returning ownership to LeRobot teleoperation:

```bash
ros2 service call /alohamini_lerobot_bridge/command_enable std_srvs/srv/SetBool '{data: false}'
```
