# alohamini_calibration

Authoritative robot-specific calibration assets for AlohaMini.

- `config/hardware/`: encoder tick-to-radian reference, sign, and measured safe
  range. The left logical arm currently inherits the one physically measured
  arm mapping.
- `config/gripper/`: normalized command to URDF joint mapping.
- `config/cameras/camera_models.yaml`: capture-derived camera model metadata.
- `config/cameras/intrinsics/`: ROS camera-info candidates.
- `config/cameras/extrinsics/`: explicitly labelled manual candidates.

Candidate and `calibrated: false` values are not promoted by directory
placement. Their status remains part of the authoritative record. Downstream
projects consume exports from this package and do not reverse-sync their local
copies over these files.
