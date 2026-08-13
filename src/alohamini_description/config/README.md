# Description parameters

- `hardware/`: actuator specification data.
- `kinematics/`: joint limits, FK representations, TCP convention, physical
  parameter, and collision-generation metadata.

Validation flags in these files are authoritative. A provisional or false flag
must not be interpreted as a verified physical safety limit. Robot-specific
encoder, gripper, and camera calibration is in `alohamini_calibration`.
