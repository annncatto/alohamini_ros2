import importlib.util
from pathlib import Path

import pytest

SCRIPT = Path(__file__).parents[1] / "scripts" / "joycon_native_reader.py"
SPEC = importlib.util.spec_from_file_location("joycon_native_reader", SCRIPT)
reader = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(reader)


def test_stationary_gravity_initializes_level_attitude():
    estimator = reader.ComplementaryAttitudeEstimator()
    orientation = estimator.update(
        [[0.0, 0.0, 0.0]] * 3,
        [[0.0, 0.0, -1.0]] * 3,
        1.0,
    )
    assert orientation == pytest.approx([0.0, 0.0, 0.0])


def test_yaw_uses_measured_elapsed_time_and_remains_relative():
    estimator = reader.ComplementaryAttitudeEstimator()
    estimator.update(
        [[0.0, 0.0, 1.0]] * 3,
        [[0.0, 0.0, -1.0]] * 3,
        1.0,
    )
    first_yaw = estimator.rpy[2]
    estimator.update(
        [[0.0, 0.0, 1.0]] * 3,
        [[0.0, 0.0, -1.0]] * 3,
        1.02,
    )
    assert estimator.rpy[2] - first_yaw == pytest.approx(-0.02)


def test_orientation_quaternion_is_normalized():
    quaternion = reader.euler_xyz_quaternion(0.3, -0.2, 0.7)
    assert sum(value * value for value in quaternion) == pytest.approx(1.0)
