import importlib.util
import math
from importlib.machinery import SourceFileLoader
from pathlib import Path

import pytest


SCRIPT = Path(__file__).parents[1] / "scripts/sync_arm_mapping"
SPEC = importlib.util.spec_from_loader(
    "sync_arm_mapping", SourceFileLoader("sync_arm_mapping", str(SCRIPT))
)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def calibration():
    return {
        f"arm_{side}_{joint}": {
            "id": motor_id,
            "drive_mode": 0,
            "homing_offset": -100 + motor_id,
            "range_min": 500,
            "range_max": 3500,
        }
        for side in ("left", "right")
        for motor_id, joint in enumerate(MODULE.ARM_JOINTS, start=1)
    }


def template(side):
    joints = {}
    for motor_id, joint in enumerate(MODULE.ARM_JOINTS, start=1):
        joints[joint] = {
            "id": motor_id,
            "model": "sts3250",
            "reference_tick": 2048,
            "reference_q_rad": 0.0,
            "sign": 1,
        }
    joints["gripper"].update(
        {
            "sign": -1,
            "reference_q_rad": 0.32,
            "joint_per_encoder_ratio": 1.0,
            "urdf_closed_rad": 0.32,
            "urdf_open_rad": -1.8030294104,
            "closed_tick": 2048,
            "open_tick": 3432,
        }
    )
    return {
        "schema_version": 1,
        "robot_model": "alohamini2pro",
        "side": side,
        "ticks_per_revolution": 4096,
        "joints": joints,
    }


def host_observation(document, ticks):
    motors = {}
    observation = {"_images": []}
    for name, entry in document.items():
        metadata = {
            "range_min": entry["range_min"],
            "range_max": entry["range_max"],
            "drive_mode": entry["drive_mode"],
            "normalization": "degrees",
        }
        motors[name] = metadata
        midpoint = (entry["range_min"] + entry["range_max"]) / 2.0
        observation[f"{name}.pos"] = (ticks[name] - midpoint) * 360.0 / 4095.0
    observation["_robot_metadata"] = {
        "schema_version": 1,
        "robot_model": "alohamini2pro",
        "motors": motors,
    }
    return observation


def test_json_and_host_capture_build_machine_specific_mapping():
    document = calibration()
    MODULE.validate_calibration(document)
    expected = {
        name: 2000 + entry["id"] for name, entry in document.items()
    }
    observation = host_observation(document, expected)

    captured = MODULE.median_stable_ticks(
        [observation, observation, observation], document, max_spread=2
    )
    mapping = MODULE.build_side_mapping(
        template("right"),
        document,
        captured,
        "right",
        {
            "captured_at": "2026-08-19T00:00:00+00:00",
            "host": "192.168.3.73",
            "observation_port": 5556,
            "command_port_used": False,
            "calibration_sha256": "abc",
        },
    )

    pan = mapping["joints"]["shoulder_pan"]
    assert pan["reference_tick"] == pytest.approx(expected["arm_right_shoulder_pan"], abs=1)
    assert pan["lerobot_calibration"]["homing_offset"] == -99
    assert pan["safe_q_min_rad"] == pytest.approx(
        (500 - pan["reference_tick"]) * 2.0 * math.pi / 4096
    )
    assert mapping["machine_profile"]["command_port_used"] is False
    assert mapping["reference_capture"]["q_rad"] == [0.0] * 6


def test_capture_rejects_motion_and_json_host_mismatch():
    document = calibration()
    ticks = {name: 2048 for name in document}
    first = host_observation(document, ticks)
    moved = host_observation(document, {**ticks, "arm_left_shoulder_pan": 2060})

    with pytest.raises(ValueError, match="moved by"):
        MODULE.median_stable_ticks([first, moved], document, max_spread=4)

    first["_robot_metadata"]["motors"]["arm_left_shoulder_pan"]["range_min"] = 501
    with pytest.raises(ValueError, match="disagree"):
        MODULE.ticks_from_observation(first, document)


def test_calibration_requires_all_arm_entries():
    document = calibration()
    document.pop("arm_right_gripper")
    with pytest.raises(ValueError, match="lacks arm_right_gripper"):
        MODULE.validate_calibration(document)
