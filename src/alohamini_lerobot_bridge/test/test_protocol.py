import math

from alohamini_lerobot_bridge.protocol import (
    BodyVelocity,
    JointMapper,
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
