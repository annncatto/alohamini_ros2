# alohamini_moveit_config

Initial MoveIt 2 configuration consolidated from `~/lerobot_alohamini`.

`plan_only.launch.py` starts KDL/OMPL collision-aware planning with
`allow_trajectory_execution=false`. It is the supported entry point in this
first consolidation. The LeRobot bridge now provides standard left-arm,
right-arm, and lift `FollowJointTrajectory` servers, and their names are
declared in `config/moveit_controllers.yaml`; this plan-only launch deliberately
does not enable execution yet. Servo and Nav2 are not part of this runtime.
Its measured folded Home is published on the private
`/alohamini_plan_only/joint_states` topic, so an already-running hardware
bridge cannot race the offline state on `/joint_states`.

```bash
ros2 launch alohamini_moveit_config plan_only.launch.py
```

The limits in `config/joint_limits.yaml` are conservative planning limits, not
independent physical safety limits.

## Hardware execution through the verified LeRobot Host

`hardware_execution.launch.py` keeps `/joint_states` bound to measured Host
feedback and connects MoveIt's existing `FollowJointTrajectory` controller
definitions to `alohamini_lerobot_bridge`.  It does not start a fake joint-state
publisher, and the bridge still starts with commands disabled.

```bash
ros2 launch alohamini_moveit_config hardware_execution.launch.py host:=ROBOT_IP
```

After confirming that the robot model follows fresh measured state and that all
three action servers are available, explicitly enable command publication:

```bash
ros2 service call /alohamini_lerobot_bridge/command_enable \
  std_srvs/srv/SetBool '{data: true}'
```

Use RViz MotionPlanning `Plan & Execute` to send a collision-checked arm
trajectory.  Disable commands immediately after the test:

```bash
ros2 service call /alohamini_lerobot_bridge/command_enable \
  std_srvs/srv/SetBool '{data: false}'
```
