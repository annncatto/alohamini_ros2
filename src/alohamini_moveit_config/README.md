# alohamini_moveit_config

Initial MoveIt 2 configuration consolidated from `~/lerobot_alohamini`.

`plan_only.launch.py` starts KDL/OMPL collision-aware planning with
`allow_trajectory_execution=false`. It is the supported entry point in this
first consolidation. The LeRobot bridge provides standard left-arm, right-arm,
and lift `FollowJointTrajectory` servers plus left/right `GripperCommand`
servers, and their names are declared in `config/moveit_controllers.yaml`; this
plan-only launch deliberately does not enable execution. Servo and Nav2 are not
part of this runtime.
Its measured folded Home is published on the private
`/alohamini_plan_only/joint_states` topic, so an already-running hardware
bridge cannot race the offline state on `/joint_states`.

```bash
ros2 launch alohamini_moveit_config plan_only.launch.py
```

The limits in `config/joint_limits.yaml` are conservative planning limits, not
independent physical safety limits.

## Hardware execution through the verified LeRobot Host

`move_group.launch.py` is a component: it consumes measured `/joint_states` and
the standard `FollowJointTrajectory` and `GripperCommand` actions, but never
starts a Bridge or `robot_state_publisher`. `alohamini_bringup` owns the
complete hardware composition and still starts with commands disabled.

```bash
ros2 launch alohamini_bringup hardware.launch.py host:=ROBOT_IP \
  enable_moveit:=true use_rviz:=true
```

After confirming that the robot model follows fresh measured state and that all
five action servers are available, explicitly enable command publication:

```bash
ros2 service call /alohamini_lerobot_bridge/command_enable \
  std_srvs/srv/SetBool '{data: true}'
```

Use RViz MotionPlanning `Plan & Execute` to send a collision-checked arm,
gripper, or lift trajectory. The `lift` planning group provides `bottom`,
`middle`, and `top` named states; its current measured position remains the
trajectory start state.

The mobile base is velocity-controlled separately through `/cmd_vel`. It is
not exposed as a MoveIt position-trajectory controller because this runtime has
no odometry or pose estimator: the three `root_*` joints are visualization
anchors fixed at zero, not measured base pose. A MoveIt base trajectory could
therefore neither track pose error nor determine successful arrival.

Disable commands immediately after the test:

```bash
ros2 service call /alohamini_lerobot_bridge/command_enable \
  std_srvs/srv/SetBool '{data: false}'
```
