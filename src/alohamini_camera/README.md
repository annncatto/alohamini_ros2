# alohamini_camera

ROS 2 camera bridge for the dedicated AlohaMini Host camera stream on port
5557. Camera traffic is separate from the Bridge's 50 Hz state channel on
5556.

```bash
ros2 launch alohamini_camera camera.launch.py host:=PI5_IP
```

The default configuration subscribes to all five Host cameras: `forward`,
`backward`, `chest`, `wrist_left`, and `wrist_right`. Published topics for each
configured camera:

- `/alohamini/cameras/<name>/image_raw`
- `/alohamini/cameras/<name>/image_raw/compressed`
- `/alohamini/cameras/<name>/camera_info`

JPEG and CameraInfo remain at camera rate. Raw RGB decoding is demand-driven:
the node does not allocate or serialize uncompressed images until an
`image_raw` subscriber exists.

An uncalibrated camera still publishes raw and compressed images using its
configured optical frame, but intentionally does not publish `camera_info`.
Diagnostics report `camera_info=unavailable_calibration_mode` until that
camera's reviewed intrinsic YAML is installed. An explicitly configured but
invalid `camera_info_url` remains a startup error.

`timestamp_mode:=host_wall` preserves the Host capture time and requires the Pi
and ROS computer clocks to be synchronized. Use `receipt` only for preview and
diagnostics; it is not accepted for hand-eye calibration.

Accepted hand-eye files can publish optical TFs explicitly:

```bash
ros2 launch alohamini_camera camera.launch.py host:=PI5_IP \
  enable_extrinsics:=true \
  extrinsics:=/path/forward.yaml,/path/wrist_right.yaml
```

Candidate files are rejected unless `allow_candidate_extrinsics:=true` is
provided for a controlled validation run.
