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

## Camera intrinsic and hand-eye calibration

The canonical target is the packaged OpenCV-standard 9x7 ChArUco board: 31
markers (IDs 300-330), 26.00 mm squares, and 18.70 mm markers on A4 landscape.
Print `alohamini_charuco_a4_9x7_31.pdf` at actual size with fit-to-page disabled,
then verify several squares measure 26.00 mm. The older 9x6/25 mm checkerboard
file is an optional template and is not the AlohaMini default.

Regenerate the printable PNG and PDF after intentionally changing board
geometry:

```bash
conda activate lerobot_alohamini
ros2 run alohamini_calibration generate_charuco_board \
  --board $(ros2 pkg prefix alohamini_calibration)/share/alohamini_calibration/config/cameras/boards/charuco_9x7_26mm_18p7_ids300_330.yaml \
  --output-dir ~/camera_calibration/print
```

The Host camera-only stream uses port 5557 and does not compete with the 50 Hz
Bridge state channel. Capture unique JPEG frames without commanding hardware:

```bash
ros2 run alohamini_calibration capture_camera_calibration \
  --host PI5_IP --camera wrist_right --count 40 \
  --output ~/camera_calibration/wrist_right
```

The intrinsic capture opens a live OpenCV preview by default. It continues to
save unique frames automatically at `--min-interval-sec`; press `Q` or `Esc` to
finish early while keeping the images already saved. Pass `--no-preview` only
for a headless capture host.

Run the OpenCV solvers in the `lerobot_alohamini` Conda environment, which
contains fisheye and hand-eye support:

```bash
conda activate lerobot_alohamini
ros2 run alohamini_calibration calibrate_camera_intrinsics \
  --capture-dir ~/camera_calibration/wrist_right \
  --board $(ros2 pkg prefix alohamini_calibration)/share/alohamini_calibration/config/cameras/boards/charuco_9x7_26mm_18p7_ids300_330.yaml \
  --camera wrist_right --frame-id right_camera_optical --model plumb_bob \
  --output ~/camera_calibration/wrist_right_candidate.yaml
```

For eye-in-hand capture, synchronize the Pi and ROS computer clocks and start
Bringup with both camera and robot state on Host wall time:

```bash
ros2 launch alohamini_bringup hardware.launch.py host:=PI5_IP \
  enable_cameras:=true camera_timestamp_mode:=host_wall \
  state_timestamp_mode:=host_wall
```

Fix the ChArUco board in the world and move the arm slowly through diverse
poses. The capture tool is read-only:

```bash
ros2 run alohamini_calibration capture_hand_eye_samples \
  --camera wrist_right \
  --image-topic /alohamini/cameras/wrist_right/image_raw/compressed \
  --gripper-frame right_Fixed_Jaw --mount-link right_camera \
  --output ~/camera_calibration/hand_eye_right
```

Hand-eye capture also opens a live OpenCV preview by default. It overlays the
saved count and the current stationary/diversity gate state. Press `Q` or
`Esc` to finish early and retain completed samples; use `--no-preview` on a
headless ROS computer. The ROS Python environment therefore needs the declared
`python3-opencv` runtime dependency. Its two-worker executor allows TF data to
enter the Buffer while an image callback briefly waits for the transform at
the capture timestamp; changing this path back to a single-threaded spin causes
an accumulating future-extrapolation delay.

The capture waits for the measured gripper pose to remain within 1.5 mm and
0.75 degrees for 0.4 seconds before accepting a sample. These defaults can be
adjusted with `--stationary-dwell-sec`, `--stationary-translation-mm`, and
`--stationary-rotation-deg`. For eye-to-hand capture it also rejects samples if
the fixed camera mount moves by more than 2 mm or 1 degree, protecting chest
calibration from accidental lift motion.

Then solve and compare Tsai, Park, and Horaud:

```bash
conda activate lerobot_alohamini
ros2 run alohamini_calibration calibrate_hand_eye \
  --capture-dir ~/camera_calibration/hand_eye_right \
  --intrinsics ~/camera_calibration/wrist_right_candidate.yaml \
  --board $(ros2 pkg prefix alohamini_calibration)/share/alohamini_calibration/config/cameras/boards/charuco_9x7_26mm_18p7_ids300_330.yaml \
  --optical-frame right_camera_optical \
  --output ~/camera_calibration/wrist_right_hand_eye_candidate.yaml
```

For the fixed `forward` camera, rigidly attach the ChArUco board to a gripper and
pass `--calibration-type eye_to_hand` during capture. The same solver then uses
the inverted robot poses and outputs `base_link -> forward_camera_optical` plus
the derived `front_camera -> forward_camera_optical` transform. Never mix a
fixed-board wrist capture with an attached-board forward capture.

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
ros2 launch alohamini_bringup hardware.launch.py \
  host:=192.168.3.73 \
  arm_mapping_dir:=$HOME/.config/alohamini/192.168.3.73 \
  enable_moveit:=true use_rviz:=true
```

The bridge remains read-only until `command_enable` is explicitly called.
