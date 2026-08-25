from __future__ import annotations

import argparse
import math
import time

import numpy as np

from .kinematics import AlohaMiniArmKinematics


def quaternion_from_z(direction: np.ndarray) -> np.ndarray:
    """Return a MuJoCo WXYZ quaternion rotating +Z onto direction."""
    direction = np.asarray(direction, dtype=float)
    direction /= np.linalg.norm(direction)
    z_axis = np.array([0.0, 0.0, 1.0])
    dot = float(np.clip(np.dot(z_axis, direction), -1.0, 1.0))
    if dot > 1.0 - 1.0e-10:
        return np.array([1.0, 0.0, 0.0, 0.0])
    if dot < -1.0 + 1.0e-10:
        return np.array([0.0, 1.0, 0.0, 0.0])
    axis = np.cross(z_axis, direction)
    axis /= np.linalg.norm(axis)
    half = 0.5 * math.acos(dot)
    return np.concatenate(([math.cos(half)], axis * math.sin(half)))


def viewer_xml(lengths: list[float]) -> str:
    segments = []
    for index, length in enumerate(lengths):
        segments.append(
            f"""
            <body name="segment_{index}">
              <freejoint name="segment_joint_{index}"/>
              <geom type="capsule" size="0.009 {length / 2.0:.12f}"
                    rgba="0.25 0.55 0.85 1"/>
            </body>"""
        )
    return f"""
<mujoco model="alohamini_dh">
  <option gravity="0 0 0" timestep="0.01"/>
  <worldbody>
    <light pos="0 -1 1" dir="0 1 -1"/>
    <geom type="plane" size="1 1 0.01" rgba="0.15 0.15 0.18 1"/>
    {"".join(segments)}
    <body name="tcp_frame">
      <freejoint name="tcp_joint"/>
      <geom type="sphere" size="0.018" rgba="1 0.8 0.1 1"/>
      <geom type="capsule" fromto="0 0 0 0.07 0 0" size="0.004" rgba="1 0 0 1"/>
      <geom type="capsule" fromto="0 0 0 0 0.07 0" size="0.004" rgba="0 1 0 1"/>
      <geom type="capsule" fromto="0 0 0 0 0 0.07" size="0.004" rgba="0 0.4 1 1"/>
    </body>
    <body name="target" mocap="true"><geom type="sphere" size="0.014" rgba="1 0.1 0.2 0.6"/></body>
  </worldbody>
</mujoco>
"""


def set_free_body(data, model, joint_name: str, position, quaternion_wxyz) -> None:
    address = model.jnt_qposadr[model.joint(joint_name).id]
    data.qpos[address : address + 3] = position
    data.qpos[address + 3 : address + 7] = quaternion_wxyz


def update_scene(data, model, kinematics, joints, segment_pairs) -> np.ndarray:
    transforms = kinematics.link_transforms(joints)
    for segment_index, (start_index, end_index) in enumerate(segment_pairs):
        start = transforms[start_index][:3, 3]
        end = transforms[end_index][:3, 3]
        set_free_body(
            data,
            model,
            f"segment_joint_{segment_index}",
            (start + end) * 0.5,
            quaternion_from_z(end - start),
        )
    tcp = transforms[-1]
    # matrix_to_quaternion returns XYZW; MuJoCo free joints use WXYZ.
    from .kinematics import matrix_to_quaternion

    x, y, z, w = matrix_to_quaternion(tcp[:3, :3])
    set_free_body(data, model, "tcp_joint", tcp[:3, 3], [w, x, y, z])
    return tcp


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Visualize the exact AlohaMini DH chain and TCP in MuJoCo.")
    parser.add_argument("--side", choices=("left", "right"), default="right")
    parser.add_argument(
        "--joints",
        nargs=6,
        type=float,
        default=[0.0, -1.571, 1.571, 0.0, 0.0, 0.0],
        metavar=("PAN", "LIFT", "ELBOW", "FLEX", "YAW", "ROLL"),
    )
    parser.add_argument("--target-xyz", nargs=3, type=float)
    parser.add_argument("--sweep", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    import mujoco
    import mujoco.viewer

    kinematics = AlohaMiniArmKinematics.from_description(args.side)
    initial = np.asarray(args.joints, dtype=float)
    transforms = kinematics.link_transforms(initial)
    segment_pairs = [
        (index, index + 1)
        for index in range(len(transforms) - 1)
        if np.linalg.norm(transforms[index + 1][:3, 3] - transforms[index][:3, 3]) > 1.0e-7
    ]
    lengths = [
        float(np.linalg.norm(transforms[end][:3, 3] - transforms[start][:3, 3]))
        for start, end in segment_pairs
    ]
    model = mujoco.MjModel.from_xml_string(viewer_xml(lengths))
    data = mujoco.MjData(model)
    target = (
        np.asarray(args.target_xyz, dtype=float) if args.target_xyz is not None else transforms[-1][:3, 3]
    )
    target_id = model.body("target").mocapid[0]
    data.mocap_pos[target_id] = target
    with mujoco.viewer.launch_passive(model, data) as viewer:
        started = time.monotonic()
        while viewer.is_running():
            joints = initial.copy()
            if args.sweep:
                phase = time.monotonic() - started
                joints[0] += 0.35 * math.sin(phase * 0.7)
                joints[2] += 0.25 * math.sin(phase * 0.9)
                joints[4] += 0.4 * math.sin(phase * 1.1)
            update_scene(data, model, kinematics, joints, segment_pairs)
            mujoco.mj_forward(model, data)
            viewer.sync()
            time.sleep(1.0 / 60.0)


if __name__ == "__main__":
    main()
