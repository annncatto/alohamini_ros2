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

The packaged checkerboard file is a template: physically measure the printed
square and update `square_size_m` before collecting hand-eye data.
