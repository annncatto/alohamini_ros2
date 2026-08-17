# alohamini_calibration

Authoritative robot-specific calibration assets for AlohaMini.

`config/lerobot/AlohaMiniRobot.json` is the corresponding deployable LeRobot
motor-calibration export for the installed left and right arms. Keep this JSON
separate from the tick-to-URDF YAML maps: it contains measured firmware homing
offsets and LeRobot normalization ranges used by teleoperation.

## Dual-arm LeRobot calibration

Run the standalone tool on the computer physically connected to both arm buses,
inside the `lerobot_alohamini` Python environment. Stop Host and teleoperation
first:

```bash
source /opt/ros/humble/setup.bash
source ~/alohamini_ros2/install/setup.bash
ros2 run alohamini_calibration calibrate_arms \
  --robot-model alohamini2pro \
  --left-port /dev/serial/by-id/<left-adapter> \
  --right-port /dev/serial/by-id/<right-adapter> \
  --output ~/AlohaMiniRobot.candidate.json
```

Use `--list-ports` first if needed. Prefer stable `/dev/serial/by-id/...` names
when available. The tool never guesses which `ttyACM` device is left or right,
because enumeration order can change and both arm buses expose motor IDs 1–7.

This is an interactive real-hardware operation. By default it disables arm
torque, preserves the existing homing offsets, records manually swept min/max
ranges, writes only arm motors 1–7, and generates one combined left/right
LeRobot JSON. Pass `--rehome` only when the encoder half-turn reference must also
be recalibrated. It never calls AlohaMini `connect`, `configure`, lift homing, or
any motion command; base and lift JSON entries come unchanged from the packaged
template.

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
