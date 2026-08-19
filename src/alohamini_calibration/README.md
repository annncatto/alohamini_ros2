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

## Per-robot folded-Home mapping

`AlohaMiniRobot.json` contains EEPROM homing offsets and LeRobot normalization
ranges. It does not contain the live `Present_Position` at the folded CAD Home,
so JSON alone cannot establish an exact URDF zero/reference mapping.

With the Host running and both follower arms stationary in the verified folded
Home (both grippers closed), pull the machine JSON and capture current arm ticks
over the read-only 5556 state endpoint:

```bash
ros2 run alohamini_calibration sync_arm_mapping \
  --ssh-target pi5@192.168.3.73 \
  --host 192.168.3.73 \
  --output-dir ~/.config/alohamini/192.168.3.73
```

The tool uses `rsync` for the JSON and ZMQ `:state` for the live capture. It
never opens command port 5555, serial devices, changes torque, homes an axis, or
writes EEPROM. It cross-checks the running Host ranges against the pulled JSON,
rejects moving captures, and writes:

- `AlohaMiniRobot.json`;
- `hardware_joint_map_left.yaml`;
- `hardware_joint_map_right.yaml`.

The output remains a candidate until RViz and collision validation pass. Load a
reviewed profile without modifying the CAD/URDF definition:

```bash
ros2 launch alohamini_moveit_config hardware_execution.launch.py \
  host:=192.168.3.73 \
  arm_mapping_dir:=$HOME/.config/alohamini/192.168.3.73
```

The bridge remains read-only until `command_enable` is explicitly called.
