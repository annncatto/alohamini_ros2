# Robot description ownership

## Authority

`alohamini_description` is the authoritative public source for:

- link and joint names, origins, axes, and frame semantics;
- planning reference and TCP frames;
- visual and collision geometry;
- physical joint limits and frame conventions;
- ROS URDF/SRDF, mesh-free kinematic, and planning-model exports.

The files under `urdf/source/` and `config/` are maintained inputs. The
top-level files under `urdf/` plus `srdf/` and `meshes/` are public consumable
exports. An exported file may be copied or packaged for a downstream runtime,
but that copy is not authoritative.

## Downstream ownership

ManiSkill and RoboTwin retain their own environment definitions, tasks,
controllers, rendering setup, and simulation-specific loaders/adapters.
LeRobot retains embodied-intelligence workflows and the verified physical
Runtime/Host. Those projects may adapt a public export locally when their
runtime requires it.

Description corrections flow into this repository first and are then exported
outward. A change discovered in a downstream project can be proposed here with
its measurement or test evidence, but downstream generated files must never be
reverse-synced over the authoritative inputs.

The original RoboTwin, LeRobot, and ManiSkill paths recorded in `provenance/`
exist only to make the initial consolidation auditable.

`alohamini_calibration` is the authority for robot-specific encoder-to-radian
mappings, gripper mapping, camera intrinsics/extrinsics, and their validation
status. Description exports may consume those values, but they do not own the
calibration source.
