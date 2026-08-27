# AlohaMini Bringup

`alohamini_bringup` is the only recommended physical-hardware composition
entry point. It starts exactly one LeRobot Bridge and one
`robot_state_publisher`; cameras, MoveIt, Joy-Con and RViz are optional
consumers around those owners.

Commands remain disabled after every launch. After checking the measured pose
and stopping every non-ROS LeRobot command client, enable them explicitly:

```bash
ros2 service call /alohamini_lerobot_bridge/command_enable \
  std_srvs/srv/SetBool '{data: true}'
```

Common launch modes:

```bash
# State, TF and cameras (read-only by default).
ros2 launch alohamini_bringup hardware.launch.py host:=ROBOT_IP

# MoveIt execution and its RViz configuration.
ros2 launch alohamini_bringup hardware.launch.py host:=ROBOT_IP \
  enable_moveit:=true use_rviz:=true

# Joy-Con differential IK. MoveIt remains available for comparison/services.
ros2 launch alohamini_bringup hardware.launch.py host:=ROBOT_IP \
  enable_moveit:=true enable_joycon:=true use_rviz:=true
```

The component launches in `alohamini_moveit_config` and
`alohamini_joycon_teleop` intentionally do not start a Bridge or
`robot_state_publisher`. Do not start two Bringup graphs, and do not run another
LeRobot client against Host port 5555 while ROS commands are enabled.
