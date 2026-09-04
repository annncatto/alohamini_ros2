from __future__ import annotations

import math
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory


ARM_JOINTS = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_yaw_joint",
    "wrist_roll",
)


def load_yaml(path: Path) -> dict:
    with path.open(encoding="utf-8") as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, dict):
        raise AssertionError(f"expected YAML mapping: {path}")
    return value


def rpy_matrix(rpy: str) -> np.ndarray:
    roll, pitch, yaw = np.fromstring(rpy, sep=" ")
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c, s, one_minus_c = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    return np.array(
        [
            [c + x * x * one_minus_c, x * y * one_minus_c - z * s, x * z * one_minus_c + y * s],
            [y * x * one_minus_c + z * s, c + y * y * one_minus_c, y * z * one_minus_c - x * s],
            [z * x * one_minus_c - y * s, z * y * one_minus_c + x * s, c + z * z * one_minus_c],
        ]
    )


def origin_transform(joint: ET.Element) -> np.ndarray:
    origin = joint.find("origin")
    transform = np.eye(4)
    if origin is not None:
        transform[:3, :3] = rpy_matrix(origin.get("rpy", "0 0 0"))
        transform[:3, 3] = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
    return transform


class UrdfKinematics:
    def __init__(self, path: Path) -> None:
        self.root = ET.parse(path).getroot()
        self.joint_by_child = {
            joint.find("child").get("link"): joint for joint in self.root.findall("joint")
        }

    def chain(self, base: str, tip: str) -> list[ET.Element]:
        result = []
        current = tip
        while current != base:
            if current not in self.joint_by_child:
                raise AssertionError(f"{tip} is not downstream of {base}")
            joint = self.joint_by_child[current]
            result.append(joint)
            current = joint.find("parent").get("link")
        return list(reversed(result))

    def fk(self, base: str, tip: str, positions: dict[str, float]) -> np.ndarray:
        transform = np.eye(4)
        for joint in self.chain(base, tip):
            transform = transform @ origin_transform(joint)
            value = float(positions.get(joint.get("name"), 0.0))
            motion = np.eye(4)
            if joint.get("type") in ("revolute", "continuous"):
                axis = np.fromstring(joint.find("axis").get("xyz"), sep=" ")
                motion[:3, :3] = axis_angle(axis, value)
            elif joint.get("type") == "prismatic":
                axis = np.fromstring(joint.find("axis").get("xyz"), sep=" ")
                motion[:3, 3] = axis * value
            transform = transform @ motion
        return transform


def dh_transform(a: float, alpha: float, d: float, theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array(
        [[c, -s * ca, s * sa, a * c], [s, c * ca, -c * sa, a * s], [0, sa, ca, d], [0, 0, 0, 1]]
    )


def dh_fk(config: dict, q_rad: list[float]) -> np.ndarray:
    standard = config["standard_dh"]
    transform = np.asarray(standard["base_transform"], dtype=float)
    for row, value in zip(standard["rows"], q_rad, strict=True):
        transform = transform @ dh_transform(
            float(row["a"]),
            float(row["alpha"]),
            float(row["d"]),
            float(value) + float(row["theta_offset"]),
        )
    return transform @ np.asarray(standard["tool_transform"], dtype=float)


def signed_tick_delta(tick: int, reference: int, period: int) -> int:
    return int((tick - reference + period // 2) % period - period // 2)


def tick_to_rad(entry: dict, tick: int, period: int, near: float | None = None) -> float:
    delta = signed_tick_delta(tick, int(entry["reference_tick"]), period)
    ratio = float(entry.get("joint_per_encoder_ratio", 1.0))
    value = float(entry["reference_q_rad"]) + int(entry["sign"]) * delta * 2 * math.pi / period * ratio
    if near is not None:
        value += round((near - value) / (2 * math.pi * ratio)) * 2 * math.pi * ratio
    return value


def rad_to_tick(entry: dict, value: float, period: int) -> int:
    ratio = float(entry.get("joint_per_encoder_ratio", 1.0))
    encoder_delta = (value - float(entry["reference_q_rad"])) / int(entry["sign"]) / ratio
    return int((int(entry["reference_tick"]) + round(encoder_delta * period / (2 * math.pi))) % period)


def joint_limits(entry: dict, name: str) -> tuple[float, float]:
    if name == "gripper":
        return float(entry["urdf_open_rad"]), float(entry["urdf_closed_rad"])
    if name == "wrist_roll":
        return -math.pi, math.pi
    return float(entry["safe_q_min_rad"]), float(entry["safe_q_max_rad"])


def validate_tree(model: UrdfKinematics) -> None:
    links = {link.get("name") for link in model.root.findall("link")}
    joints = model.root.findall("joint")
    children = [joint.find("child").get("link") for joint in joints]
    parents = [joint.find("parent").get("link") for joint in joints]
    assert len(links) == 32 and len(joints) == 31
    assert len(children) == len(set(children))
    assert links - set(children) == {"root"}
    assert set(children) <= links and set(parents) <= links
    assert {"wheel1", "wheel2", "wheel3"} <= links
    assert not {"Link2_dp", "Link3_dp", "Link4_dp"} & links
    joints_by_name = {joint.get("name"): joint for joint in joints}
    cad_joint = joints_by_name["base_cad_joint"]
    assert cad_joint.get("type") == "fixed"
    assert cad_joint.find("parent").get("link") == "base_link"
    assert cad_joint.find("child").get("link") == "base_cad_link"
    assert np.allclose(
        np.fromstring(cad_joint.find("origin").get("xyz"), sep=" "),
        [0.0, 0.0, 0.0],
    )
    assert np.allclose(
        np.fromstring(cad_joint.find("origin").get("rpy"), sep=" "),
        [0.0, 0.0, math.pi / 2.0],
    )
    cad_rotation = origin_transform(cad_joint)[:3, :3]
    assert np.allclose(cad_rotation @ [0.0, -1.0, 0.0], [1.0, 0.0, 0.0])
    assert np.allclose(cad_rotation @ [1.0, 0.0, 0.0], [0.0, 1.0, 0.0])
    for wheel in ("wheel1", "wheel2", "wheel3"):
        joint = joints_by_name[f"{wheel}_joint"]
        assert joint.get("type") == "continuous"
        assert joint.find("parent").get("link") == "base_cad_link"
        assert joint.find("child").get("link") == wheel


def validate_fk(model: UrdfKinematics, description: Path, validation: Path) -> None:
    golden = load_yaml(validation / "config/fk_golden.yaml")
    dh = load_yaml(description / "config/kinematics/right_arm_kinematics.yaml")
    tool = load_yaml(description / "config/kinematics/kinematics.yaml")["tool_frames"]
    fixed_to_tcp = np.eye(4)
    fixed_to_tcp[:3, :3] = np.asarray(tool["delta_matrix"], dtype=float)
    fixed_to_tcp[:3, 3] = fixed_to_tcp[:3, :3] @ np.asarray(tool["tcp_tool_m"], dtype=float)
    tolerance = float(golden["tolerance"])
    for sample_name, sample in golden["samples"].items():
        q_rad = [float(value) for value in sample["q_rad"]]
        expected = np.asarray(sample["transform"], dtype=float)
        results = []
        for side in ("left", "right"):
            positions = {
                f"{side}_{joint}": value for joint, value in zip(ARM_JOINTS, q_rad, strict=True)
            }
            actual = model.fk(f"{side}_Base", f"{side}_tcp", positions)
            assert np.allclose(actual, expected, atol=tolerance), f"{sample_name}/{side} FK drift"
            results.append(actual)
        assert np.allclose(results[0], results[1], atol=tolerance)
        assert np.allclose(dh_fk(dh, q_rad) @ fixed_to_tcp, expected, atol=tolerance), f"{sample_name} DH drift"


def validate_mapping(calibration: Path, description: Path, validation: Path) -> None:
    right = load_yaml(calibration / "config/hardware/hardware_joint_map_right.yaml")
    left = load_yaml(calibration / "config/hardware/hardware_joint_map_left.yaml")
    home = load_yaml(calibration / "config/hardware/arm_home.yaml")
    lift = load_yaml(calibration / "config/hardware/lift_axis.yaml")
    assert left["side"] == "left"
    assert right["side"] == "right"
    assert set(left["joints"]) == set(right["joints"])
    period = int(right["ticks_per_revolution"])
    assert int(left["ticks_per_revolution"]) == period
    one_tick = 2 * math.pi / period
    direction_reference = load_yaml(validation / "config/joint_direction_reference.yaml")["joint_state_gui"]
    urdf = ET.parse(description / "urdf/alohamini2pro_kinematic.urdf").getroot()
    urdf_joints = {joint.get("name"): joint for joint in urdf.findall("joint")}
    exported_urdfs = {
        path.name: {
            joint.get("name"): joint
            for joint in ET.parse(path).getroot().findall("joint")
        }
        for path in (
            description / "urdf/alohamini2pro_left_kinematic.urdf",
            description / "urdf/alohamini2pro_right_kinematic.urdf",
            description / "urdf/alohamini2pro_kinematic.urdf",
            description / "urdf/alohamini2pro_moveit.urdf",
        )
    }
    for side, mapping in (("left", left), ("right", right)):
        expected_reference_ticks = mapping["reference_capture"]["ticks"]
        stowed_ticks = home["installed_arm_present_position_ticks"][side]["values"]
        for index, (name, entry) in enumerate(mapping["joints"].items()):
            reference_q = float(entry["reference_q_rad"])
            assert math.isclose(
                tick_to_rad(entry, int(entry["reference_tick"]), period), reference_q
            )
            if name != "gripper":
                assert int(entry["reference_tick"]) == int(
                    expected_reference_ticks[index]
                )
                assert math.isclose(
                    reference_q,
                    float(home["mapping_reference_cad_home_q_rad_by_side"][side][name]),
                )
                suffix = "wrist_yaw_joint" if name == "wrist_yaw" else name
                stowed_name = f"{side}_{suffix}"
                expected_stowed = float(home["stowed_joint_positions"][stowed_name])
                actual_stowed = tick_to_rad(
                    entry, int(stowed_ticks[index]), period, near=expected_stowed
                )
                assert math.isclose(actual_stowed, expected_stowed, abs_tol=1e-11)
            lower, upper = joint_limits(entry, name)
            for value in (lower, reference_q, upper, (lower + upper) / 2):
                tick = rad_to_tick(entry, value, period)
                recovered = tick_to_rad(entry, tick, period, near=value)
                assert abs(recovered - value) <= one_tick * float(
                    entry.get("joint_per_encoder_ratio", 1.0)
                )
            positive_tick = rad_to_tick(entry, reference_q + one_tick, period)
            direction = signed_tick_delta(
                positive_tick, int(entry["reference_tick"]), period
            )
            assert direction == int(entry["sign"]), (
                f"{side}/{name}: positive-q tick direction drift"
            )
            suffix = "wrist_yaw_joint" if name == "wrist_yaw" else name
            expected = direction_reference[suffix]
            expected_tick_direction = (
                "increasing" if int(entry["sign"]) == 1 else "decreasing"
            )
            assert expected["positive_q_raw_tick_direction"] == expected_tick_direction
            axis = np.fromstring(urdf_joints[f"{side}_{suffix}"].find("axis").get("xyz"), sep=" ")
            assert np.array_equal(axis, np.asarray(expected["urdf_axis_xyz"], dtype=float))
            joint_name = f"{side}_{suffix}"
            for filename, joints in exported_urdfs.items():
                if joint_name not in joints:
                    continue
                limit = joints[joint_name].find("limit")
                assert math.isclose(float(limit.get("lower")), lower, abs_tol=1e-11), (
                    f"{filename}/{joint_name} lower limit drift"
                )
                assert math.isclose(float(limit.get("upper")), upper, abs_tol=1e-11), (
                    f"{filename}/{joint_name} upper limit drift"
                )
    assert int(right["joints"]["shoulder_pan"]["sign"]) == 1
    assert int(left["joints"]["shoulder_pan"]["sign"]) == 1
    assert float(lift["mechanism"]["physical_min_mm"]) == 0.0
    assert float(lift["mechanism"]["physical_max_mm"]) == 600.0
    assert float(lift["urdf"]["q_at_physical_min_m"]) == -0.3
    assert float(lift["urdf"]["q_at_physical_max_m"]) == 0.3
    assert float(lift["mechanism"]["fitted_lead_error_percent"]) < 1.0


def resolve_package_uri(uri: str, description: Path) -> Path:
    prefix = "package://alohamini_description/"
    assert uri.startswith(prefix), uri
    return description / uri.removeprefix(prefix)


def validate_collision(description: Path, validation: Path) -> None:
    model = ET.parse(description / "urdf/alohamini2pro_moveit.urdf").getroot()
    srdf = ET.parse(description / "srdf/alohamini2pro.srdf").getroot()
    baseline = load_yaml(validation / "config/collision_baseline.yaml")
    expected = baseline["geometry_counts"]
    actual = {
        "links": len(model.findall("link")),
        "joints": len(model.findall("joint")),
        "visuals": len(model.findall(".//visual")),
        "collisions": len(model.findall(".//collision")),
        "collision_boxes": len(model.findall(".//collision/geometry/box")),
        "collision_cylinders": len(model.findall(".//collision/geometry/cylinder")),
        "collision_meshes": len(model.findall(".//collision/geometry/mesh")),
        "srdf_disabled_pairs": len(srdf.findall("disable_collisions")),
    }
    assert actual == expected, f"collision baseline drift: {actual} != {expected}"
    for mesh in model.findall(".//mesh"):
        assert resolve_package_uri(mesh.get("filename"), description).is_file()
    for material in model.findall(".//visual/material"):
        assert material.get("name"), "visual material names must not be empty"
    links = {link.get("name"): link for link in model.findall("link")}
    joints_by_name = {joint.get("name"): joint for joint in model.findall("joint")}
    arm_collision_meshes = {
        "Base": "arm_base_vhacd.stl",
        "Rotation_Pitch": "rotation_pitch_vhacd.stl",
        "Upper_Arm": "upper_arm_vhacd.stl",
        "Lower_Arm": "lower_arm_vhacd.stl",
        "Wrist_Pitch_Roll": "wrist_pitch_roll_vhacd.stl",
        "wrist_yaw": "wrist_yaw_vhacd.stl",
        "camera": "wrist_camera_vhacd.stl",
    }
    for side in ("left", "right"):
        for suffix, collision_name in arm_collision_meshes.items():
            link = links[f"{side}_{suffix}"]
            visual = link.find("visual/geometry/mesh")
            collision = link.find("collision/geometry/mesh")
            assert visual is not None and f"/visual/{side}_{suffix}.STL" in visual.get("filename")
            assert collision is not None and collision.get("filename").endswith(
                f"/collision/{collision_name}"
            )
    for side in ("left", "right"):
        fixed = links[f"{side}_Fixed_Jaw"]
        fixed_collisions = fixed.findall("collision")
        fixed_boxes = [
            collision for collision in fixed_collisions
            if collision.find("geometry/box") is not None
        ]
        fixed_meshes = [
            collision for collision in fixed_collisions
            if collision.find("geometry/mesh") is not None
        ]
        assert len(fixed_boxes) == int(baseline["fixed_jaw_boxes_per_side"])
        assert len(fixed_meshes) == int(baseline["fixed_jaw_vhacd_pieces_per_side"])
        expected_indices = {0, 1, 2, 3, 4, 5, 7}
        actual_indices = {
            int(collision.get("name").rsplit("_", 1)[1])
            for collision in fixed_meshes
        }
        assert actual_indices == expected_indices
        assert all(
            not collision.find("geometry/mesh").get("filename").endswith(
                "/collision/fixed_jaw_vhacd.stl"
            )
            for collision in fixed_meshes
        )
        pad = fixed_boxes[0]
        assert pad.get("name") == f"{side}_fixed_finger_pad"
        assert np.allclose(
            np.fromstring(pad.find("origin").get("xyz"), sep=" "),
            [-0.015, 0.0, 0.072],
        )
        assert np.allclose(
            np.fromstring(pad.find("geometry/box").get("size"), sep=" "),
            [0.010, 0.014, 0.064],
        )
    for side in ("left", "right"):
        moving = links[f"{side}_Moving_Jaw"]
        assert len(moving.findall("collision/geometry/mesh")) == int(
            baseline["moving_jaw_vhacd_pieces_per_side"]
        )
        moving_boxes = [
            collision for collision in moving.findall("collision")
            if collision.find("geometry/box") is not None
        ]
        assert len(moving_boxes) == int(baseline["moving_jaw_boxes_per_side"])
        pad = moving_boxes[0]
        assert pad.get("name") == f"{side}_moving_finger_pad"
        pad_xyz = np.fromstring(pad.find("origin").get("xyz"), sep=" ")
        pad_rpy = np.fromstring(pad.find("origin").get("rpy"), sep=" ")
        assert np.allclose(
            pad_xyz,
            [-0.0012969999878772, -0.050369580583936, 0.0033063194616967],
        )
        assert np.allclose(pad_rpy, [-math.pi / 2.0, 0.0, math.pi - 0.03])
        assert np.allclose(
            np.fromstring(pad.find("geometry/box").get("size"), sep=" "),
            [0.010, 0.014, 0.064],
        )

        gripper_joint = joints_by_name[f"{side}_gripper"]
        joint_at_calibration = origin_transform(gripper_joint)
        axis = np.fromstring(gripper_joint.find("axis").get("xyz"), sep=" ")
        joint_at_calibration[:3, :3] = (
            joint_at_calibration[:3, :3]
            @ axis_angle(axis, 0.03)
        )
        moving_pad = origin_transform(pad)
        pad_in_fixed = joint_at_calibration @ moving_pad
        assert np.allclose(pad_in_fixed[:3, :3], np.eye(3), atol=1e-12)
        assert np.allclose(pad_in_fixed[:3, 3], [0.020, 0.0, 0.072], atol=1e-12)
        fixed_inner_x = -0.015 + 0.010 / 2.0
        moving_inner_x = pad_in_fixed[0, 3] - 0.010 / 2.0
        assert math.isclose(moving_inner_x - fixed_inner_x, 0.025, abs_tol=1e-12)
        assert math.isclose((moving_inner_x + fixed_inner_x) / 2.0, 0.0025, abs_tol=1e-12)
    assert not links["base_link"].findall("visual")
    assert not links["base_link"].findall("collision")
    assert links["base_link"].find("inertial") is None
    base_collision = links["base_cad_link"].findall("collision/geometry/mesh")
    assert len(base_collision) == 1 and base_collision[0].get("filename").endswith("/base_link.STL")
    passive = {joint.get("name") for joint in srdf.findall("passive_joint")}
    assert passive == {"wheel1_joint", "wheel2_joint", "wheel3_joint"}
    assert baseline["state_validity"]["installed_stowed"]["contact_pairs"] == []
    for index, wheel in enumerate(("wheel1", "wheel2", "wheel3"), start=2):
        visual_meshes = links[wheel].findall("visual/geometry/mesh")
        assert len(visual_meshes) == 1
        assert visual_meshes[0].get("filename").endswith(f"/Link{index}_dp.STL")
        cylinders = links[wheel].findall("collision/geometry/cylinder")
        assert len(cylinders) == 1
        assert float(cylinders[0].get("radius")) > 0
        assert float(cylinders[0].get("length")) > 0
    for box in model.findall(".//collision/geometry/box"):
        assert np.all(np.fromstring(box.get("size"), sep=" ") > 0)


def validate_lidar_three_wheel(description: Path) -> None:
    preview = description / "urdf/lidar_three_wheel_preview.urdf.xacro"
    result = subprocess.run(
        ["xacro", str(preview)],
        check=True,
        capture_output=True,
        text=True,
    )
    root = ET.fromstring(result.stdout)
    links = {link.get("name"): link for link in root.findall("link")}
    joints = {joint.get("name"): joint for joint in root.findall("joint")}
    assert set(links) == {"base_link", "wheel1", "wheel2", "wheel3"}
    assert set(joints) == {"wheel1_joint", "wheel2_joint", "wheel3_joint"}

    expected = load_yaml(description / "config/three_wheel_base.yaml")
    assert expected["status"] == "integrated_current_cad_wheels"
    assert expected["integration_constraints"]["do_not_attach_both_wheel_variants"] is True
    kinematics = expected["kinematics"]
    assert kinematics["command_frame"] == "base_link"
    assert kinematics["convention"] == "rep103_x_forward_y_left_z_up_yaw_ccw"
    assert kinematics["body_twist_order"] == ["vx_m_s", "vy_m_s", "yaw_rate_rad_s"]
    assert kinematics["wheel_order"] == [
        "wheel1_joint",
        "wheel2_joint",
        "wheel3_joint",
    ]
    angles = np.deg2rad(kinematics["rolling_direction_angles_base_link_deg"])
    base_radius = float(expected["geometry_authority"]["base_radius_m"])
    expected_matrix = np.column_stack(
        (-np.cos(angles), -np.sin(angles), np.full(3, base_radius))
    )
    matrix = np.asarray(
        kinematics["body_twist_to_wheel_linear_matrix_m"], dtype=float
    )
    assert np.allclose(matrix, expected_matrix, atol=1e-9)
    for wheel_name, values in expected["legacy_source_wheels"].items():
        joint = joints[f"{wheel_name}_joint"]
        assert joint.get("type") == "continuous"
        assert joint.find("parent").get("link") == "base_link"
        assert joint.find("child").get("link") == wheel_name
        origin = joint.find("origin")
        assert np.allclose(
            np.fromstring(origin.get("xyz"), sep=" "), values["position_xyz_m"], atol=1e-9
        )
        assert np.allclose(
            np.fromstring(origin.get("rpy"), sep=" "), values["orientation_rpy_rad"], atol=1e-9
        )
        assert np.allclose(
            np.fromstring(joint.find("axis").get("xyz"), sep=" "), values["axis_xyz"], atol=1e-9
        )
        mesh = links[wheel_name].find("visual/geometry/mesh")
        assert resolve_package_uri(mesh.get("filename"), description).is_file()

    integrated = ET.parse(description / "urdf/alohamini2pro_moveit.urdf").getroot()
    integrated_joints = {joint.get("name"): joint for joint in integrated.findall("joint")}
    base_to_cad = rpy_matrix(
        integrated_joints["base_cad_joint"].find("origin").get("rpy")
    )
    for index, (wheel_name, values) in enumerate(
        expected["authoritative_wheels"].items()
    ):
        joint = integrated_joints[f"{wheel_name}_joint"]
        assert joint.find("parent").get("link") == "base_cad_link"
        origin = joint.find("origin")
        assert np.allclose(np.fromstring(origin.get("xyz"), sep=" "), values["position_xyz_m"])
        assert np.allclose(np.fromstring(origin.get("rpy"), sep=" "), values["orientation_rpy_rad"])
        assert np.allclose(np.fromstring(joint.find("axis").get("xyz"), sep=" "), values["axis_xyz"])
        joint_to_cad = rpy_matrix(origin.get("rpy"))
        local_axis = np.fromstring(joint.find("axis").get("xyz"), sep=" ")
        axis_in_cad = joint_to_cad @ local_axis
        rolling_in_cad = np.cross([0.0, 0.0, 1.0], axis_in_cad)
        rolling_in_base = base_to_cad @ (
            rolling_in_cad / np.linalg.norm(rolling_in_cad)
        )
        configured_rolling = np.asarray(
            [math.cos(angles[index]), math.sin(angles[index]), 0.0]
        )
        assert np.allclose(rolling_in_base, configured_rolling, atol=2e-4)
        assert np.allclose(matrix[index, :2], -rolling_in_base[:2], atol=2e-4)


def main() -> None:
    description = Path(get_package_share_directory("alohamini_description"))
    calibration = Path(get_package_share_directory("alohamini_calibration"))
    validation = Path(get_package_share_directory("alohamini_validation"))
    model = UrdfKinematics(description / "urdf/alohamini2pro_kinematic.urdf")
    checks = (
        ("URDF tree", lambda: validate_tree(model)),
        ("FK golden + DH", lambda: validate_fk(model, description, validation)),
        ("joint direction + tick/rad mapping", lambda: validate_mapping(calibration, description, validation)),
        ("collision structure", lambda: validate_collision(description, validation)),
        ("lidar three-wheel component", lambda: validate_lidar_three_wheel(description)),
    )
    for name, check in checks:
        check()
        print(f"[PASS] {name}")


if __name__ == "__main__":
    main()
