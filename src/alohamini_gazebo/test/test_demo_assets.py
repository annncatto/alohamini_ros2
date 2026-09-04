import importlib.util
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import pytest

from alohamini_gazebo.lift_pick_place_demo import (
    HOME,
    PICK,
    PICK_PRE,
    PLACE,
    PLACE_PRE,
    RIGHT_JOINTS,
    parse_attachment_state,
)


PACKAGE = Path(__file__).resolve().parents[1]
DESCRIPTION = PACKAGE.parent / "alohamini_description/urdf/alohamini2pro_moveit.urdf"


def test_all_demo_sdf_assets_parse():
    assets = [PACKAGE / "worlds/lift_pick_place.sdf", *PACKAGE.glob("models/*/model.sdf")]
    assert len(assets) == 4
    for asset in assets:
        assert ET.parse(asset).getroot().tag == "sdf"


def test_object_is_gripper_sized_and_target_marker_is_complete():
    object_root = ET.parse(PACKAGE / "models/grasp_object/model.sdf").getroot()
    size_text = object_root.findtext(".//collision/geometry/box/size")
    size = [float(value) for value in size_text.split()]
    assert size == [0.05, 0.02, 0.065]

    target_root = ET.parse(PACKAGE / "models/aruco_drop_zone/model.sdf").getroot()
    marker_cells = {
        visual.get("name") for visual in target_root.findall(".//visual")
        if visual.get("name", "").startswith("marker_r")
    }
    assert marker_cells == {
        "marker_r1c3", "marker_r2c3", "marker_r3c2", "marker_r3c3",
        "marker_r3c4", "marker_r4c1", "marker_r4c2", "marker_r4c4",
    }


def test_pick_platform_is_one_600_mm_high_stage():
    root = ET.parse(PACKAGE / "models/high_plinth/model.sdf").getroot()
    collisions = root.findall(".//collision")
    assert len(collisions) == 1
    assert collisions[0].get("name") == "platform_collision"
    size = [float(value) for value in collisions[0].findtext("geometry/box/size").split()]
    pose = [float(value) for value in collisions[0].findtext("pose").split()]
    assert size == [0.48, 0.38, 0.60]
    assert pose[2] + size[2] / 2 == 0.60


def test_demo_arm_waypoints_respect_authoritative_limits():
    root = ET.parse(DESCRIPTION).getroot()
    limits = {}
    for joint in root.findall("joint"):
        limit = joint.find("limit")
        if limit is not None and limit.get("lower") is not None:
            limits[joint.get("name")] = (float(limit.get("lower")), float(limit.get("upper")))
    for waypoint in (HOME, PICK_PRE, PICK, PLACE_PRE, PLACE):
        assert len(waypoint) == len(RIGHT_JOINTS)
        for name, value in zip(RIGHT_JOINTS, waypoint, strict=True):
            lower, upper = limits[name]
            assert lower <= value <= upper


def test_detachable_joint_string_state_is_strict():
    assert parse_attachment_state("attached") is True
    assert parse_attachment_state(" detached ") is False
    assert parse_attachment_state("true") is None
    assert parse_attachment_state("") is None


def test_bridge_uses_detachable_joint_string_message():
    bridge = (PACKAGE / "config/bridge.yaml").read_text()
    block = bridge.split("ros_topic_name: /demo_object/attached", 1)[1]
    assert "ros_type_name: std_msgs/msg/String" in block
    assert "gz_type_name: gz.msgs.StringMsg" in block


def test_demo_launch_propagates_failed_exit_code():
    launch_path = PACKAGE / "launch/lift_pick_place_demo.launch.py"
    spec = importlib.util.spec_from_file_location("lift_pick_place_launch", launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    assert len(module._shutdown_after_demo(SimpleNamespace(returncode=0), None)) == 1
    with pytest.raises(RuntimeError, match="exit code 1"):
        module._shutdown_after_demo(SimpleNamespace(returncode=1), None)
