# alohamini_moveit_config

Initial MoveIt 2 configuration consolidated from `~/lerobot_alohamini`.

`plan_only.launch.py` starts KDL/OMPL collision-aware planning with
`allow_trajectory_execution=false`. It is the supported entry point in this
first consolidation. The LeRobot bridge now provides standard left-arm,
right-arm, and lift `FollowJointTrajectory` servers, and their names are
declared in `config/moveit_controllers.yaml`; this plan-only launch deliberately
does not enable execution yet. Servo and Nav2 are not part of this runtime.

```bash
ros2 launch alohamini_moveit_config plan_only.launch.py
```

The limits in `config/joint_limits.yaml` are conservative planning limits, not
independent physical safety limits.
