import json
import math
import time
from collections import deque

import pytest
import zmq

from alohamini_lerobot_bridge.protocol import (
    BodyVelocity,
    CommandGate,
    JointMapper,
    ZmqHostTransport,
    lift_command_ready,
    validate_state_observation,
    wheel_velocity_from_body,
)


def test_wheel_velocity_round_trip_order():
    velocity = BodyVelocity(x=0.12, y=-0.03, yaw=0.4)
    wheels = wheel_velocity_from_body(velocity)
    assert len(wheels) == 3
    assert all(math.isfinite(value) for value in wheels)
    assert wheels[0] != wheels[1] != wheels[2]


def test_host_normalization_to_reference_urdf():
    calibration = {
        "ticks_per_revolution": 4096,
        "joints": {
            "shoulder_pan": {
                "reference_tick": 2027,
                "reference_q_rad": 0.0,
                "sign": -1,
            }
        },
    }
    mapper = JointMapper(calibration)
    metadata = {
        "range_min": 0,
        "range_max": 4095,
        "drive_mode": 0,
        "normalization": "degrees",
    }
    midpoint = (metadata["range_min"] + metadata["range_max"]) / 2.0
    degrees = (2027 - midpoint) * 360.0 / 4095.0
    tick = mapper.lerobot_to_tick(degrees, metadata)
    assert abs(tick - 2027) <= 1
    assert abs(mapper.tick_to_urdf("shoulder_pan", tick, "left_shoulder_pan")) < 0.002

    key, normalized = mapper.urdf_to_lerobot("left_shoulder_pan", 0.0, {
        "motors": {"arm_left_shoulder_pan": metadata}
    })
    assert key == "arm_left_shoulder_pan.pos"
    assert mapper.lerobot_to_tick(normalized, metadata) == pytest.approx(2027, abs=1)


def test_independent_side_references_route_observation_and_command():
    joint = {
        "reference_q_rad": 0.0,
        "sign": 1,
        "safe_q_min_rad": -1.0,
        "safe_q_max_rad": 1.0,
    }
    mapper = JointMapper(
        {
            "left": {
                "ticks_per_revolution": 4096,
                "joints": {"shoulder_pan": {**joint, "reference_tick": 2014}},
            },
            "right": {
                "ticks_per_revolution": 4096,
                "joints": {"shoulder_pan": {**joint, "reference_tick": 2041}},
            },
        }
    )
    motors = {
        f"arm_{side}_shoulder_pan": {
            "range_min": 500,
            "range_max": 3500,
            "drive_mode": 0,
            "normalization": "degrees",
        }
        for side in ("left", "right")
    }
    metadata = {"motors": motors}
    observation = {
        f"arm_{side}_shoulder_pan.pos": mapper.tick_to_lerobot(
            tick, motors[f"arm_{side}_shoulder_pan"]
        )
        for side, tick in (("left", 2014), ("right", 2041))
    }

    positions = mapper.observation_to_joint_positions(observation, metadata)

    assert positions["left_shoulder_pan"] == pytest.approx(0.0, abs=0.002)
    assert positions["right_shoulder_pan"] == pytest.approx(0.0, abs=0.002)
    for side, expected_tick in (("left", 2014), ("right", 2041)):
        key, value = mapper.urdf_to_lerobot(
            f"{side}_shoulder_pan", 0.0, metadata
        )
        assert key == f"arm_{side}_shoulder_pan.pos"
        assert mapper.lerobot_to_tick(value, motors[key.removesuffix(".pos")]) == pytest.approx(
            expected_tick, abs=1
        )


def test_command_mapping_rejects_tick_outside_side_host_range():
    calibration = {
        "ticks_per_revolution": 4096,
        "joints": {
            "shoulder_pan": {
                "reference_tick": 2041,
                "reference_q_rad": 0.0,
                "sign": 1,
                "safe_q_min_rad": -2.0,
                "safe_q_max_rad": 2.0,
            }
        },
    }
    mapper = JointMapper(calibration)
    metadata = {
        "motors": {
            "arm_right_shoulder_pan": {
                "range_min": 2000,
                "range_max": 2100,
                "drive_mode": 0,
                "normalization": "degrees",
            }
        }
    }

    with pytest.raises(ValueError, match="outside Host range"):
        mapper.urdf_to_lerobot("right_shoulder_pan", 0.5, metadata)


def test_first_observation_selects_calibrated_periodic_branch():
    mapper = JointMapper(
        {
            "ticks_per_revolution": 4096,
            "joints": {
                "shoulder_lift": {
                    "reference_tick": 969,
                    "reference_q_rad": -1.571,
                    "sign": -1,
                    "safe_q_min_rad": -5.0532,
                    "safe_q_max_rad": -1.5617,
                }
            },
        }
    )

    # The far range endpoint crosses the 0/4095 branch.  A fresh bridge must
    # report the equivalent angle inside the calibrated URDF interval rather
    # than jumping to the +1.2 rad periodic representation.
    value = mapper.tick_to_urdf(
        "shoulder_lift", 3239, "right_shoulder_lift"
    )

    assert -5.0532 <= value <= -1.5617
    assert value == pytest.approx(-5.0531363885, abs=2e-4)


def test_lift_physical_endpoints_map_to_urdf_limits():
    mapper = JointMapper(
        {"ticks_per_revolution": 4096, "joints": {}},
        {
            "mechanism": {"physical_min_mm": 0.0, "physical_max_mm": 600.0},
            "urdf": {
                "q_at_physical_min_m": -0.3,
                "q_at_physical_max_m": 0.3,
                "clamp_to_physical_range": True,
            },
        },
    )

    assert mapper.lift_height_to_urdf(0.0) == pytest.approx(-0.3)
    assert mapper.lift_height_to_urdf(300.0) == pytest.approx(0.0)
    assert mapper.lift_height_to_urdf(600.0) == pytest.approx(0.3)
    assert mapper.lift_height_to_urdf(605.0) == pytest.approx(0.3)


def test_command_gate_requires_a_new_command_after_every_enable():
    gate = CommandGate()
    old_command = BodyVelocity(x=0.2)

    assert not gate.accept(old_command, now=1.0)
    gate.enable()
    assert gate.resolve(permitted=True, timeout_sec=0.5, now=1.1) is None

    assert gate.accept(BodyVelocity(x=0.1), now=1.2)
    assert gate.resolve(permitted=True, timeout_sec=0.5, now=1.3) == BodyVelocity(x=0.1)

    gate.enable()
    assert gate.resolve(permitted=True, timeout_sec=0.5, now=1.4) is None


def test_command_gate_stops_only_after_its_stream_started():
    gate = CommandGate()
    gate.enable()
    assert not gate.disable()
    gate.enable()
    gate.accept(BodyVelocity(y=0.1), now=1.0)
    assert gate.resolve(permitted=True, timeout_sec=0.5, now=1.6) == BodyVelocity()
    assert gate.disable()


def test_lift_readiness_accepts_legacy_height_and_prefers_explicit_flags():
    metadata = {"lift_axis": {"soft_min_mm": 0.0, "soft_max_mm": 600.0}}

    assert lift_command_ready(metadata, {"lift_axis.height_mm": 300.0})
    assert not lift_command_ready(metadata, {"lift_axis.height_mm": math.nan})
    assert lift_command_ready(
        metadata,
        {
            "lift_axis.height_mm": 300.0,
            "lift_axis.homed": True,
            "lift_axis.torque_enabled": True,
        },
    )
    assert not lift_command_ready(
        metadata,
        {
            "lift_axis.height_mm": 300.0,
            "lift_axis.homed": False,
            "lift_axis.torque_enabled": True,
        },
    )


def _validator_fixture():
    calibration = {
        "ticks_per_revolution": 4096,
        "joints": {
            "shoulder_pan": {
                "reference_tick": 2048,
                "reference_q_rad": 0.0,
                "sign": 1,
            }
        },
    }
    metadata = {
        "schema_version": 1,
        "robot_model": "alohamini2pro",
        "motors": {
            "arm_left_shoulder_pan": {
                "normalization": "degrees",
                "drive_mode": 0,
                "range_min": 0,
                "range_max": 4095,
            }
        },
    }
    observation = {
        "x.vel": 0.1,
        "y.vel": -0.02,
        "theta.vel": 5.0,
        "arm_left_shoulder_pan.pos": 0.0,
        "_robot_metadata": metadata,
        "_images": [],
    }
    return JointMapper(calibration), observation


def test_state_validation_requires_complete_finite_fields():
    mapper, observation = _validator_fixture()
    metadata, positions, velocity = validate_state_observation(observation, mapper)
    assert metadata["robot_model"] == "alohamini2pro"
    assert set(positions) == {"left_shoulder_pan"}
    assert velocity == BodyVelocity(x=0.1, y=-0.02, yaw=5.0)

    for missing in ("x.vel", "arm_left_shoulder_pan.pos", "_robot_metadata"):
        invalid = dict(observation)
        invalid.pop(missing)
        with pytest.raises(ValueError):
            validate_state_observation(invalid, mapper)

    non_finite = dict(observation, **{"theta.vel": math.nan})
    with pytest.raises(ValueError):
        validate_state_observation(non_finite, mapper)


class _FakeObservationSocket:
    def __init__(self, frames):
        self.frames = deque(frames)

    def recv_multipart(self, flags):
        if self.frames:
            return self.frames.popleft()
        raise zmq.Again()


def test_state_transport_rejects_camera_frames_and_unmatched_tokens():
    transport = ZmqHostTransport.__new__(ZmqHostTransport)
    valid_token = b"ros-1:state"
    state = json.dumps({"_images": [], "x.vel": 0.0}).encode()
    transport.pending = deque([(valid_token, time.monotonic())])
    transport.observation = _FakeObservationSocket(
        [
            [b"unknown:state", state],
            [valid_token, state, b"forward", b"jpeg"],
            [valid_token, state],
        ]
    )

    latest, malformed = transport.receive_latest()

    assert latest == {"_images": [], "x.vel": 0.0}
    assert malformed == 2
