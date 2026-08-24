import math

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory

from alohamini_joycon_teleop.mapping import (
    LEVEL_TCP_QUATERNION,
    base_command,
    integrate_base_preview,
    normalize_stick,
    step_orientation_toward,
    tcp_button_delta,
)


def test_stick_deadzone_and_range():
    assert normalize_stick(2000, 0.25) == 0.0
    assert normalize_stick(2200, 0.25) == 0.0
    assert normalize_stick(4000, 0.25) == pytest.approx(1.0)
    assert normalize_stick(0, 0.25) == pytest.approx(-1.0)


def _buttons(**overrides) -> dict[str, bool]:
    buttons = {
        "stick": False,
        "shoulder": False,
        "trigger": False,
        "sl": False,
        "sr": False,
        "menu": False,
        "relatch": False,
        "up": False,
        "down": False,
        "left": False,
        "right": False,
    }
    buttons.update(overrides)
    return buttons


def test_tcp_buttons_forward_back_left_right():
    speed, dt = 0.1, 0.5
    step = speed * dt
    assert tcp_button_delta(_buttons(up=True), speed, dt) == pytest.approx(
        [0.0, -step, 0.0]
    )
    assert tcp_button_delta(_buttons(down=True), speed, dt) == pytest.approx(
        [0.0, step, 0.0]
    )
    assert tcp_button_delta(_buttons(left=True), speed, dt) == pytest.approx(
        [step, 0.0, 0.0]
    )
    assert tcp_button_delta(_buttons(right=True), speed, dt) == pytest.approx(
        [-step, 0.0, 0.0]
    )


def test_tcp_shoulder_combo_is_z():
    speed, dt = 0.1, 0.5
    step = speed * dt
    assert tcp_button_delta(
        _buttons(shoulder=True, up=True), speed, dt
    ) == pytest.approx([0.0, 0.0, step])
    assert tcp_button_delta(
        _buttons(shoulder=True, down=True), speed, dt
    ) == pytest.approx([0.0, 0.0, -step])


def test_base_command_translation():
    # Single stick: up = forward, right = right strafe; yaw is handled by the
    # caller (shoulder-modified turn stick).
    assert base_command([(0.0, 1.0)], 0.2, 0.5) == pytest.approx(
        (0.2, 0.0, 0.0)
    )
    assert base_command([(1.0, 0.0)], 0.2, 0.5) == pytest.approx(
        (0.0, -0.2, 0.0)
    )
    # Two sticks sum and clamp.
    assert base_command([(0.5, 0.0), (0.5, 0.0)], 0.2, 0.5) == pytest.approx(
        (0.0, -0.2, 0.0)
    )
    assert base_command([(0.0, 1.0), (0.7, 0.0)], 0.2, 0.5) == pytest.approx(
        (0.2, -0.14, 0.0)
    )


def test_base_preview_uses_rep103_axes():
    assert integrate_base_preview(
        [1.0, 2.0, 0.25], 0.4, -0.2, 0.5, 0.5
    ) == pytest.approx([1.2, 1.9, 0.5])


def test_orientation_step_is_rate_limited():
    identity = [0.0, 0.0, 0.0, 1.0]
    half_turn = [0.0, 0.0, 1.0, 0.0]  # 180 degrees about z
    # A big step caps the rotation at max_step_rad.
    small = step_orientation_toward(identity, half_turn, math.pi / 4.0)
    assert small == pytest.approx(
        [0.0, 0.0, math.sin(math.pi / 8.0), math.cos(math.pi / 8.0)]
    )
    # Within the cap the goal is reached exactly.
    tiny = step_orientation_toward(identity, small, math.pi / 4.0)
    assert tiny == pytest.approx(small)
    # Double-cover: approaching -goal yields the same rotation (up to sign).
    negative = step_orientation_toward(
        identity, [-value for value in half_turn], math.pi / 4.0
    )
    for expected, actual in zip(small, negative, strict=True):
        assert abs(expected - actual) < 1.0e-9 or abs(expected + actual) < 1.0e-9


def test_level_tcp_quaternion_matches_fk_golden_reference():
    golden = get_package_share_directory("alohamini_validation")
    with open(f"{golden}/config/fk_golden.yaml", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    matrix = data["samples"]["reference"]["transform"]
    # Rotation part of the validated "level grasp" reference FK; convert the
    # rotation matrix to an XYZW quaternion and compare with the constant.
    r00, r01, r02 = matrix[0][0], matrix[0][1], matrix[0][2]
    r10, r11, r12 = matrix[1][0], matrix[1][1], matrix[1][2]
    r20, r21, r22 = matrix[2][0], matrix[2][1], matrix[2][2]
    trace = r00 + r11 + r22
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        w = 0.25 * s
        x = (r21 - r12) / s
        y = (r02 - r20) / s
        z = (r10 - r01) / s
    else:
        if r00 >= r11 and r00 >= r22:
            s = math.sqrt(1.0 + r00 - r11 - r22) * 2
            x = 0.25 * s
            y = (r01 + r10) / s
            z = (r02 + r20) / s
            w = (r21 - r12) / s
        elif r11 >= r00 and r11 >= r22:
            s = math.sqrt(1.0 + r11 - r00 - r22) * 2
            x = (r01 + r10) / s
            y = 0.25 * s
            z = (r12 + r21) / s
            w = (r02 - r20) / s
        else:
            s = math.sqrt(1.0 + r22 - r00 - r11) * 2
            x = (r02 + r20) / s
            y = (r12 + r21) / s
            z = 0.25 * s
            w = (r10 - r01) / s
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    quaternion = [value / norm for value in (x, y, z, w)]
    # Quaternions are double-cover; compare up to sign. The golden matrix
    # carries ~1e-12 rounding noise, so allow a small tolerance.
    for expected, actual in zip(LEVEL_TCP_QUATERNION, quaternion, strict=True):
        assert abs(expected - actual) < 1.0e-4 or abs(expected + actual) < 1.0e-4
    assert math.sqrt(sum(value * value for value in LEVEL_TCP_QUATERNION)) == pytest.approx(1.0)
