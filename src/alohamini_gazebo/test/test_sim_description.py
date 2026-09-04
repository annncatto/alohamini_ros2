from pathlib import Path
import xml.etree.ElementTree as ET

from alohamini_gazebo.sim_description import (
    ARM_JOINTS,
    SIM_BLACK_VISUAL_LINKS,
    make_sim_description,
)


ROOT = Path(__file__).resolve().parents[2]
AUTHORITATIVE = ROOT / "alohamini_description/urdf/alohamini2pro_moveit.urdf"


def test_overlay_preserves_authoritative_joint_kinematics(tmp_path):
    source = ET.parse(AUTHORITATIVE).getroot()
    derived = ET.fromstring(make_sim_description(AUTHORITATIVE, tmp_path / "controllers.yaml"))

    def semantic_xml(element):
        return (
            element.tag,
            tuple(sorted(element.attrib.items())),
            (element.text or "").strip(),
            tuple(semantic_xml(child) for child in element),
        )

    source_joints = {
        joint.get("name"): semantic_xml(joint)
        for joint in source.findall("joint")
    }
    derived_joints = {
        joint.get("name"): semantic_xml(joint)
        for joint in derived.findall("joint")
        if joint.get("name") != "world_to_root"
    }
    assert derived_joints == source_joints
    assert derived.find("./joint[@name='world_to_root']") is not None
    assert derived.find("./ros2_control") is not None


def test_overlay_exports_only_expected_controlled_joints(tmp_path):
    derived = ET.fromstring(make_sim_description(AUTHORITATIVE, tmp_path / "controllers.yaml"))
    actual = {joint.get("name") for joint in derived.findall("./ros2_control/joint")}
    expected = {
        "root_x_axis_joint", "root_y_axis_joint", "root_z_rotation_joint",
        "vertical_move", "wheel1_joint", "wheel2_joint", "wheel3_joint",
        *ARM_JOINTS,
    }
    assert actual == expected


def test_overlay_makes_only_arms_and_cameras_matte_black(tmp_path):
    derived = ET.fromstring(make_sim_description(AUTHORITATIVE, tmp_path / "controllers.yaml"))
    for name in SIM_BLACK_VISUAL_LINKS:
        materials = derived.findall(f"./link[@name='{name}']/visual/material")
        assert materials
        assert all(material.get("name") == "sim_matte_black" for material in materials)
        assert all(
            material.find("color").get("rgba") == "0.012 0.015 0.020 1"
            for material in materials
        )

    body_material = derived.find("./link[@name='vertical_link']/visual/material")
    assert body_material is not None
    assert body_material.get("name") != "sim_matte_black"
