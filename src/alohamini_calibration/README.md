# alohamini_calibration

Authoritative robot-specific calibration assets for AlohaMini.

## Lift-axis encoder calibration

The lift calibration tool is observation-only: it connects only to the Host
5556 DEALER/ROUTER state path and never opens the 5555 command port or a serial
device. The updated Host must expose raw, extended, zero-reference, and homed
lift fields.

After the Host has completed its normal homing, run:

```bash
ros2 run alohamini_calibration calibrate_lift_axis \
  --host 192.168.3.48 \
  --output src/alohamini_calibration/config/hardware/lift_axis_candidate.yaml
```

Keep the lift at home for the first capture. Then use an existing controller in
another terminal to move it, physically measure height above home, and enter at
least two additional points spread across the travel. The output remains a
candidate until its direction, measured lead, residual, limits, and
`vertical_move` mapping have been reviewed.

- `config/hardware/`: encoder tick-to-radian reference, sign, and measured safe
  range. The left logical arm currently inherits the one physically measured
  arm mapping.
- `config/gripper/`: normalized command to URDF joint mapping.
- `config/geometry/`: candidate geometric fit adjustments applied to the
  authoritative description, retaining their uncalibrated status and CAD
  baselines.
- `config/cameras/camera_models.yaml`: capture-derived camera model metadata.
- `config/cameras/intrinsics/`: ROS camera-info candidates.
- `config/cameras/extrinsics/`: explicitly labelled manual candidates.

Candidate and `calibrated: false` values are not promoted by directory
placement. Their status remains part of the authoritative record. Downstream
projects consume exports from this package and do not reverse-sync their local
copies over these files.
