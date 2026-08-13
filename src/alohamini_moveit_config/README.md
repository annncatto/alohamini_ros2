# alohamini_moveit_config

Initial MoveIt 2 configuration consolidated from `~/lerobot_alohamini`.

`plan_only.launch.py` starts KDL/OMPL collision-aware planning with
`allow_trajectory_execution=false`. It is the supported entry point in this
first consolidation. Hardware execution, the ZMQ trajectory runtime, Servo,
and mock execution have not yet moved into this workspace.

```bash
ros2 launch alohamini_moveit_config plan_only.launch.py
```

The limits in `config/joint_limits.yaml` are conservative planning limits, not
independent physical safety limits.
