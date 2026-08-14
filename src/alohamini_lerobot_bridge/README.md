# alohamini_lerobot_bridge

ROS 2 state and command integration for the physical runtime hosted by
`lerobot_alohamini`. This package never opens a serial device. The LeRobot Host
remains the only owner of both motor buses and its watchdog remains the final
motion stop.

The bridge uses the current protocol:

- port 5555: PUSH command client to the Host PULL socket;
- port 5556: DEALER state requests to the Host ROUTER socket;
- a `:state` response is exactly request token plus JSON state;
- `:state` JSON declares `_images=[]` and has no camera-name/JPEG frames;
- camera pairs exist only on the separate `:full` request mode.

`/joint_states` carries two deliberately separate partial messages. Arm and
lift positions are measured by the Host and are also mirrored on
`~/measured_joint_states`. Wheel velocity is reconstructed from the measured
Host body velocity, and wheel position is only time-integrated for RViz; that
wheel-only message is mirrored on `~/derived_wheel_states`. Derived wheel state
is not encoder position, odometry, or control feedback. That derived message
also holds the MoveIt virtual root joints at zero, anchoring `root -> base_link`
without claiming to provide odometry.

This version does not connect lidar, an external odometer, or an IMU, and does
not launch SLAM, Nav2, Servo, or state estimation. It publishes neither
`/odom` nor `odom -> base_link` TF.

Commands are disabled by default. This matters because the Host's port 5555 has
no lease protocol: two PUSH clients would race in the Host PULL queue. After
stopping every other LeRobot command client and confirming fresh diagnostics:

```bash
ros2 service call /alohamini_lerobot_bridge/command_enable std_srvs/srv/SetBool '{data: true}'
```

Enabling creates a new command epoch and sends nothing by itself. A `/cmd_vel`
received before enable is discarded; only a new, finite `/cmd_vel` received
after enable can start the 5555 stream.

Disable before returning ownership to LeRobot teleoperation:

```bash
ros2 service call /alohamini_lerobot_bridge/command_enable std_srvs/srv/SetBool '{data: false}'
```
