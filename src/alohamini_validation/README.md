# AlohaMini offline validation

No tool in this package opens a serial port or talks to the physical Host.

```bash
ros2 run alohamini_validation validate_assets
ros2 launch alohamini_description view_model.launch.py
ros2 launch alohamini_description view_model.launch.py use_gui:=false use_rviz:=false
ros2 run alohamini_validation validate_tf
ros2 launch alohamini_moveit_config plan_only.launch.py use_rviz:=false
ros2 run alohamini_validation validate_moveit
```

`validate_assets` checks the URDF tree, package mesh URIs, fixed FK golden
samples, an independent DH/URDF FK comparison, tick/radian round trips,
positive-direction mapping, and the structural collision baseline.

`view_model.launch.py` displays the geometry and all TF axes in RViz. Move one
Joint State Publisher GUI slider at a time from zero toward positive and compare
the observed rotation with `config/joint_direction_reference.yaml`: positive
rotation follows the listed local axis by the right-hand rule. This is a visual
coordinate audit; it does not replace the recorded physical tick direction.

`validate_moveit` calls MoveIt's `/compute_fk`, `/compute_ik`, and
`/check_state_validity` services against the same golden samples.

On the installed Humble MoveIt 2.5.9 build, the process may emit an expected
"No 3D sensor plugin" message because this launch has no Octomap sensor input.
An upstream class-loader fault has also been observed when `move_group` is
stopped with SIGINT after successful validation. Neither changes the FK/IK or
state-validity results, but both remain visible rather than being suppressed.
