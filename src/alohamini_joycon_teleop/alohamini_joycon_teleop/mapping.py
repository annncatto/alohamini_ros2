from __future__ import annotations

import math


def normalize_stick(raw: float, deadzone: float) -> float:
    """Normalize a Joy-Con 0..4095 stick channel with a radial-style deadzone."""
    if not 0.0 <= deadzone < 1.0:
        raise ValueError("deadzone must be in [0, 1)")
    value = max(-1.0, min(1.0, (float(raw) - 2000.0) / 2000.0))
    if abs(value) <= deadzone:
        return 0.0
    return math.copysign((abs(value) - deadzone) / (1.0 - deadzone), value)


def quaternion_multiply(left: list[float], right: list[float]) -> list[float]:
    """Multiply XYZW quaternions."""
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    result = [
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    ]
    norm = math.sqrt(sum(value * value for value in result))
    if norm <= 1.0e-12:
        raise ValueError("zero quaternion")
    return [value / norm for value in result]


def quaternion_slerp(start: list[float], goal: list[float], t: float) -> list[float]:
    """Spherical linear interpolation between XYZW quaternions."""
    dot = sum(a * b for a, b in zip(start, goal, strict=True))
    dot = max(-1.0, min(1.0, dot))
    if dot < 0.0:
        goal = [-value for value in goal]
        dot = -dot
    if dot > 1.0 - 1.0e-9:
        return [a + t * (b - a) for a, b in zip(start, goal, strict=True)]
    theta = math.acos(dot)
    start_weight = math.sin((1.0 - t) * theta) / math.sin(theta)
    goal_weight = math.sin(t * theta) / math.sin(theta)
    return [
        start_weight * a + goal_weight * b
        for a, b in zip(start, goal, strict=True)
    ]


def step_orientation_toward(
    current: list[float], commanded: list[float], max_step_rad: float
) -> list[float]:
    """Rotate ``current`` toward ``commanded`` by at most ``max_step_rad``.

    A fixed commanded attitude computed from the latch delta would otherwise be
    rejected forever by the per-request joint-step filter; stepping the target
    keeps each IK request small and reachable.
    """
    dot = sum(a * b for a, b in zip(current, commanded, strict=True))
    dot = max(-1.0, min(1.0, dot))
    angle = 2.0 * math.acos(abs(dot))
    if angle <= max_step_rad or angle <= 1.0e-9:
        return list(commanded)
    return quaternion_slerp(current, commanded, max_step_rad / angle)


def euler_delta_quaternion(roll: float, pitch: float, yaw: float) -> list[float]:
    """Return an XYZW quaternion for intrinsic XYZ Euler increments."""
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return [
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ]


# TCP orientation in the {side}_Base frame of the classic "level grasp" rest
# pose (arm extended forward, gripper opening straight down, jaws horizontal).
# Derived from the validated MoveIt FK golden reference transform
# (alohamini_validation/config/fk_golden.yaml, sample "reference").
LEVEL_TCP_QUATERNION = [math.sqrt(0.5), -math.sqrt(0.5), 0.0, 0.0]


def tcp_button_delta(
    buttons: dict[str, bool], speed: float, dt: float
) -> list[float]:
    """Map face buttons to a TCP translation step in the {side}_Base frame.

    Both arm base frames are root-aligned (+x = robot left, -y = robot
    forward, +z = up), so the same mapping reads naturally for either hand:

    - X (left Joy-Con: d-pad up): forward  (-y)
    - B (left Joy-Con: d-pad down): backward (+y)
    - Y (left Joy-Con: d-pad left): left (+x)
    - A (left Joy-Con: d-pad right): right (-x)
    - shoulder + X: up (+z)
    - shoulder + B: down (-z)
    """
    shoulder = bool(buttons.get("shoulder"))
    forward = bool(buttons.get("up")) and not shoulder
    backward = bool(buttons.get("down")) and not shoulder
    left = bool(buttons.get("left"))
    right = bool(buttons.get("right"))
    up = shoulder and bool(buttons.get("up"))
    down = shoulder and bool(buttons.get("down"))
    step = speed * dt
    return [
        step * (float(left) - float(right)),
        step * (float(backward) - float(forward)),
        step * (float(up) - float(down)),
    ]


def base_command(
    sticks: list[tuple[float, float]],
    linear_speed: float,
    angular_speed: float,
) -> tuple[float, float, float]:
    """Map one or two sticks to a LeRobot body velocity command.

    ``sticks`` holds (horizontal, vertical) normalized inputs summed with
    clamping: +x forward, +y left. Yaw is always zero here; the caller adds
    the shoulder-modified turn stick separately.
    """
    del angular_speed
    horizontal = 0.0
    vertical = 0.0
    for stick_h, stick_v in sticks:
        horizontal += stick_h
        vertical += stick_v
    horizontal = max(-1.0, min(1.0, horizontal))
    vertical = max(-1.0, min(1.0, vertical))
    return (
        linear_speed * vertical,
        -linear_speed * horizontal,
        0.0,
    )


def integrate_base_preview(
    pose: list[float], vx: float, vy: float, yaw_rate: float, dt: float
) -> list[float]:
    """Integrate a preview pose in REP-103 ``base_link`` coordinates."""
    return [
        float(pose[0]) + float(vx) * float(dt),
        float(pose[1]) + float(vy) * float(dt),
        float(pose[2]) + float(yaw_rate) * float(dt),
    ]
