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

The encoder-derived Host lift range is calibrated as physical `[0, 600] mm`
and mapped to URDF `vertical_move [-0.3, +0.3] m`; values beyond the confirmed
mechanical endpoints are clamped for visualization.

Arm tick-to-URDF mappings may be selected per physical robot with the
`arm_mapping_dir` launch argument. The directory must contain reviewed
`hardware_joint_map_left.yaml` and `hardware_joint_map_right.yaml` files. This
changes only encoder mapping; it does not change CAD joint frames or URDF
geometry.

This version does not connect lidar, an external odometer, or an IMU, and does
not launch SLAM, Nav2, Servo, or state estimation. It publishes neither
`/odom` nor `odom -> base_link` TF.

Commands are disabled by default. This matters because the Host's port 5555 has
no lease protocol: two PUSH clients would race in the Host PULL queue. After
stopping every other LeRobot command client and confirming fresh diagnostics:

```bash
ros2 service call /alohamini_lerobot_bridge/command_enable std_srvs/srv/SetBool '{data: true}'
```

Enabling creates a new command epoch and sends nothing by itself. Each resource
is armed independently by a post-enable command. The standard interfaces are:

- base: `/cmd_vel`;
- left arm: `/left_arm_controller/follow_joint_trajectory`;
- right arm: `/right_arm_controller/follow_joint_trajectory`;
- lift: `/lift_controller/follow_joint_trajectory`;
- lift jog: `/lift_controller/joint_jog` (`control_msgs/msg/JointJog`);
- left gripper: `/left_gripper_controller/gripper_cmd`;
- right gripper: `/right_gripper_controller/gripper_cmd`.

The gripper actions use the corresponding URDF revolute-joint coordinate, so
their `position` value is in radians. MoveIt exposes each gripper as a planning
group with `open` and `closed` named states. `max_effort` must remain `0.0`:
current limiting belongs to the already validated LeRobot Host and cannot be
overridden through ROS.

A partial arm trajectory latches every omitted joint in that arm from the
latest measured state when the resource is first activated. Inactive resources
do not contribute target fields. There are no implicit zero joint targets.

If observation freshness is lost, the bridge emits one base-zero frame, emits
no new arm, lift, or gripper targets, invalidates the old command epoch, and
then stops sending so the Host watchdog remains authoritative. Cancel or
preemption only latches a short measured-position hold while state is fresh.
Excess tracking error aborts the trajectory instead of continuing to chase its
target.

Path and endpoint checks are separate. The default lift path tolerance is
`0.03 m`; at nominal trajectory completion, measured error must be within
`0.003 m`. Otherwise the controller holds the final requested setpoint for at
most `1.0 s` before returning `GOAL_TOLERANCE_VIOLATED`. Lift goals additionally
require the Host to report that resource as homed with torque enabled; other
resources remain independently available.

Lift jog uses the same guarded command composer as trajectories. While a fresh
`vertical_move` jog is refreshed, the bridge commands the latest measured
height plus or minus `lift_jog_lookahead_m` (50 mm by default), reproducing the
smooth verified U/J control without modifying the Host. A zero jog or timeout
first sends direct zero velocity for `lift_jog_stop_settle_sec`, then locks a
newer fresh measurement. This prevents reversal toward a height cached while
the lift was still moving. Stale observation clears the jog and emits no new
lift target.

The Host paired with this control layer must accept either all three base
velocity fields or none. This permits arm-only and lift-only frames without an
implicit base target; partial base triples are rejected. The matching Host also
maintains per-resource watchdog timestamps. Its total watchdog stops the arms
by reading and latching positions locally, so ROS never has to manufacture a
hold target from stale network feedback.

Disable before returning ownership to LeRobot teleoperation:

```bash
ros2 service call /alohamini_lerobot_bridge/command_enable std_srvs/srv/SetBool '{data: false}'
```
