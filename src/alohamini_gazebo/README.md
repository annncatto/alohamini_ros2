# AlohaMini Gazebo

Gazebo Fortress integration for offline AlohaMini simulation. This package
derives its robot model from `alohamini_description`; it does not own or fork
the robot geometry and never launches the LeRobot Host or serial hardware.

The first pick-and-place scene is an **assisted visual demo**: the fingers close
around the object, then Gazebo's `DetachableJoint` holds it during transport.
It validates the scene layout, command sequence, trajectories, placement gate,
and visual result; it is not evidence of a stable friction-only grasp. The demo
requires explicit attach/detach acknowledgements, exits automatically, and the
launch command returns a non-zero status when an acceptance phase fails.

Build and launch the scene:

```bash
cd ~/alohamini_ros2
source /opt/ros/humble/setup.bash
colcon build --packages-select alohamini_gazebo
source install/setup.bash
ros2 launch alohamini_gazebo lift_pick_place_demo.launch.py
```

For CI or a machine without a display:

```bash
ros2 launch alohamini_gazebo lift_pick_place_demo.launch.py headless:=true
```

The initial base adapter consumes `/cmd_vel`. It drives simulation-only planar
joints and animates the three wheels using the same three-wheel Jacobian as the
ROS bridge. It is intentionally isolated from real-hardware control.
