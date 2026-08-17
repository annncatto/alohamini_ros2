# Asset provenance

For the initial consolidation only, the whole-robot CAD baseline, visual
meshes, simplified jaw collision meshes, SRDF, and parameter files were
imported from:

`/home/anncatto/RoboTwin/assets/embodiments/alohamini2pro`

The MoveIt URDF, kinematic URDFs, TCP frames, initial conservative AABB/VHACD
collision model, and plan-only configuration were imported from:

`/home/anncatto/lerobot_alohamini`

That source is newer than the ManiSkill asset mirror: it includes the
2026-08-11 M4 hardware-test correction of the `shoulder_pan` encoder direction.
The stale ManiSkill hardware mapping was not imported.

These paths and hashes are bootstrap provenance, not an ongoing upstream
relationship. After consolidation, this ROS repository owns the public robot
description. ManiSkill, RoboTwin, and LeRobot consume its exports and must not
overwrite it through reverse synchronization.

The ROS camera-info candidates were imported from:

`/home/anncatto/ManiSkill/outputs/alohamini2pro_fisheye_candidate`

The initial arm AABBs were replaced in this ROS authority by same-frame VHACD
collision meshes after the verified compact right-arm pose produced multiple
false-positive non-adjacent contacts. Left/right CAD sources are byte-identical
and share generated collision assets. The moving-jaw VHACD pieces remain in
use. Parameters and generated-mesh hashes are recorded with the description.

The wrist extrinsics candidate was imported from:

`/home/anncatto/ManiSkill/outputs/alohamini2pro_right_wrist_tuning_offset_smoke`

`source.sha256` records hashes of imported source files at the time of the
initial consolidation. The MoveIt URDF in this package differs from the
LeRobot-generated copy only in mesh URI form, adjusted for this ROS package.

The three wheel meshes, source poses, axes, masses, and inertias were migrated
from:

`/home/anncatto/alohamini_lidar_imu/ros2_ws/src/alohamini_description`

They are retained as `urdf/components/lidar_three_wheel.xacro` for source
comparison, but are not the active whole-robot wheel geometry. The authoritative
kinematic and MoveIt models use the current AlohaMini2Pro
`Link2_dp`/`Link3_dp`/`Link4_dp` meshes and CAD poses, exposed as continuous
`wheel1`/`wheel2`/`wheel3` links.
The source's ambiguous wheel joint names were changed from `wheelN` to
`wheelN_joint`; link names remain `wheelN`. The archival CAD source itself is
kept unmodified for traceability. The ROS bridge and experimental C++
`ros2_control` backend were not migrated. The verified
serial-owning physical Runtime/Host is `/home/anncatto/lerobot_alohamini`, and
it must never contend with that experimental backend for the motor bus.
