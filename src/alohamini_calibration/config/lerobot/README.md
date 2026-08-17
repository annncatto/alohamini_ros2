# LeRobot calibration export

`AlohaMiniRobot.json` is the deployable LeRobot calibration for the currently
measured robot. It contains the two installed arms' independent motor homing
offsets and normalized ranges, plus the base and lift entries expected by the
existing AlohaMini teleoperation runtime.

It is deliberately separate from `config/hardware/hardware_joint_map_*.yaml`:

- this JSON configures LeRobot motor normalization and firmware homing offsets;
- the hardware joint maps convert corrected encoder ticks to authoritative URDF
  radians.

Do not derive `homing_offset` from a URDF reference angle. Regenerate this file
only from a reviewed `calibrate_arms` capture.

LeRobot normally consumes it from:

```text
~/.cache/huggingface/lerobot/calibration/robots/alohamini/AlohaMiniRobot.json
```
