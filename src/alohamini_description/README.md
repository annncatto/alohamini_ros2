# alohamini_description

This package is the ROS-facing authority for AlohaMini geometry, frame
semantics, physical description inputs, and generated planning descriptions.
Robot-specific calibration is owned by `alohamini_calibration`.

## Layout

- `urdf/source/`: CAD-export source baseline; only mesh paths were relocated to
  ROS package URIs.
- `urdf/alohamini2pro_moveit.urdf`: geometry-bearing planning model with TCP
  frames, corrected physical joint limits, and simplified collision geometry.
- `urdf/*_kinematic.urdf`: mesh-free TF/FK/IK models.
- `urdf/components/lidar_three_wheel.xacro`: legacy three-wheel source component
  retained from the lidar repository for comparison.
- `srdf/`: MoveIt planning groups and reviewed collision exclusions.
- `meshes/`: visual CAD and simplified collision meshes.
- `config/hardware/`: actuator specification data.
- `config/references/`: verified manufacturer parameter references.
- `config/kinematics/`: FK/TCP, joint-limit, physical, and collision
  metadata.
- `provenance/`: import sources and hashes.

The authoritative kinematic and MoveIt models use the current AlohaMini2Pro
126.9 mm wheel CAD (`Link2_dp`/`Link3_dp`/`Link4_dp`) with unambiguous
`wheel1`/`wheel2`/`wheel3` link names and continuous `wheelN_joint` joints.
The current Runtime parameters are wheel radius 0.063 m and base radius 0.195 m.

The lidar repository's older approximately 99 mm wheel component is retained
only as a provenance/visual comparison and can be previewed with:

```bash
ros2 launch alohamini_description view_lidar_three_wheel.launch.py
```

The experimental C++ `ros2_control` backend is not part of this integration;
the LeRobot Host remains the physical motor-bus owner. The ROS orientation of
`base_link` still requires a separate REP-103 audit.
