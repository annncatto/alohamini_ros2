# Camera calibration assets

Files under `intrinsics/` are ROS `camera_info` candidates derived from real
fisheye captures. Files under `extrinsics/` are manual simulation-tuning
candidates, not hand-eye calibration results. Consumers must inspect each
file's `status` or source metadata before use.

No left-wrist intrinsic or hand-eye extrinsic calibration was available at
the time of consolidation, so no value is fabricated here.

New captures and solver outputs must be written as candidates. The runtime
extrinsics node rejects candidate transforms unless `allow_candidate:=true` is
explicitly selected for validation.

The default printable target is the OpenCV-standard 9x7 ChArUco board in
`boards/charuco_9x7_26mm_18p7_ids300_330.yaml`. It has 31 markers with IDs
300-330, 26.00 mm squares, and 18.70 mm markers. Print its PDF at actual size
with fit-to-page disabled and verify the square size before calibration.

`checkerboard_9x6_25mm.yaml` remains only as an optional generic checkerboard
template. Intrinsic and hand-eye solvers dispatch on `target_type` and use the
same board geometry and detector.
