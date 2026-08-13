from __future__ import annotations

import json
import math
import time
from collections import deque
from dataclasses import dataclass
from typing import Any

import zmq


ARM_JOINTS = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_yaw",
    "wrist_roll",
    "gripper",
)


@dataclass(frozen=True)
class BodyVelocity:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


def signed_tick_delta(tick: int, reference_tick: int, period: int) -> int:
    return int((int(tick) - int(reference_tick) + period // 2) % period - period // 2)


class JointMapper:
    """Convert Host-normalized motor observations to the authoritative URDF space."""

    def __init__(self, calibration: dict[str, Any]) -> None:
        self.period = int(calibration["ticks_per_revolution"])
        self.joints = calibration["joints"]
        self.previous: dict[str, float] = {}

    def lerobot_to_tick(self, value: float, metadata: dict[str, Any]) -> int:
        lower = int(metadata["range_min"])
        upper = int(metadata["range_max"])
        if upper <= lower:
            raise ValueError("Host motor range_max must exceed range_min")
        mode = metadata["normalization"]
        drive_mode = int(metadata["drive_mode"])
        if mode == "range_m100_100":
            normalized = -value if drive_mode else value
            normalized = min(100.0, max(-100.0, normalized))
            return int(((normalized + 100.0) / 200.0) * (upper - lower) + lower)
        if mode == "range_0_100":
            normalized = 100.0 - value if drive_mode else value
            normalized = min(100.0, max(0.0, normalized))
            return int((normalized / 100.0) * (upper - lower) + lower)
        if mode == "degrees":
            midpoint = (lower + upper) / 2.0
            return int(value * (self.period - 1) / 360.0 + midpoint)
        raise ValueError(f"Unsupported Host normalization: {mode!r}")

    def tick_to_urdf(self, joint: str, tick: int, state_key: str) -> float:
        entry = self.joints[joint]
        delta = signed_tick_delta(tick, int(entry["reference_tick"]), self.period)
        ratio = float(entry.get("joint_per_encoder_ratio", 1.0))
        value = (
            float(entry["reference_q_rad"])
            + int(entry["sign"]) * delta * 2.0 * math.pi / self.period * ratio
        )
        if joint == "wrist_roll" and state_key in self.previous:
            period_rad = 2.0 * math.pi * ratio
            value += round((self.previous[state_key] - value) / period_rad) * period_rad
        self.previous[state_key] = value
        return value

    def observation_to_joint_positions(
        self, observation: dict[str, Any], metadata: dict[str, Any]
    ) -> dict[str, float]:
        motors = metadata.get("motors", {})
        positions: dict[str, float] = {}
        for side in ("left", "right"):
            for joint in ARM_JOINTS:
                motor_name = f"arm_{side}_{joint}"
                observation_key = f"{motor_name}.pos"
                motor_metadata = motors.get(motor_name)
                if observation_key not in observation or not isinstance(motor_metadata, dict):
                    continue
                tick = self.lerobot_to_tick(float(observation[observation_key]), motor_metadata)
                suffix = "wrist_yaw_joint" if joint == "wrist_yaw" else joint
                urdf_name = f"{side}_{suffix}"
                positions[urdf_name] = self.tick_to_urdf(joint, tick, urdf_name)
        if "lift_axis.height_mm" in observation:
            positions["vertical_move"] = float(observation["lift_axis.height_mm"]) / 1000.0
        return positions


def wheel_velocity_from_body(
    velocity: BodyVelocity, wheel_radius: float = 0.063, base_radius: float = 0.195
) -> tuple[float, float, float]:
    """Return wheel1(left), wheel2(back), wheel3(right) angular velocity in rad/s."""
    values = []
    for angle_deg in (150.0, -90.0, 30.0):
        angle = math.radians(angle_deg)
        linear = (
            math.cos(angle) * -velocity.x
            + math.sin(angle) * -velocity.y
            + base_radius * velocity.yaw
        )
        values.append(linear / wheel_radius)
    return tuple(values)


class ZmqHostTransport:
    """5555 PUSH command and 5556 DEALER request/reply transport."""

    def __init__(
        self,
        host: str,
        command_port: int = 5555,
        observation_port: int = 5556,
        request_window: int = 3,
        context: zmq.Context | None = None,
    ) -> None:
        if request_window < 1:
            raise ValueError("request_window must be at least one")
        self.context = context or zmq.Context()
        self.owns_context = context is None
        self.request_window = request_window
        self.command = self.context.socket(zmq.PUSH)
        self.command.setsockopt(zmq.CONFLATE, 1)
        self.command.setsockopt(zmq.LINGER, 0)
        self.command.connect(f"tcp://{host}:{command_port}")
        self.observation = self.context.socket(zmq.DEALER)
        self.observation.setsockopt(zmq.SNDHWM, request_window)
        self.observation.setsockopt(zmq.RCVHWM, request_window)
        self.observation.setsockopt(zmq.LINGER, 0)
        self.observation.connect(f"tcp://{host}:{observation_port}")
        self.sequence = 0
        self.pending: deque[tuple[bytes, float]] = deque()

    def fill_state_requests(self) -> None:
        while len(self.pending) < self.request_window:
            self.sequence += 1
            token = f"ros-{self.sequence}:state".encode("ascii")
            try:
                self.observation.send(token, flags=zmq.NOBLOCK)
            except zmq.Again:
                return
            self.pending.append((token, time.monotonic()))

    def expire_requests(self, timeout_sec: float) -> int:
        now = time.monotonic()
        expired = 0
        while self.pending and now - self.pending[0][1] > timeout_sec:
            self.pending.popleft()
            expired += 1
        return expired

    def receive_latest(self) -> tuple[dict[str, Any] | None, int]:
        latest = None
        malformed = 0
        while True:
            try:
                parts = self.observation.recv_multipart(flags=zmq.NOBLOCK)
            except zmq.Again:
                break
            if len(parts) < 2:
                malformed += 1
                continue
            token = parts[0]
            matching_index = next(
                (index for index, (candidate, _) in enumerate(self.pending) if candidate == token),
                None,
            )
            if matching_index is not None:
                for _ in range(matching_index + 1):
                    self.pending.popleft()
            try:
                decoded = json.loads(parts[1].decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                malformed += 1
                continue
            if isinstance(decoded, dict):
                latest = decoded
            else:
                malformed += 1
        return latest, malformed

    def send_action(self, action: dict[str, float]) -> bool:
        try:
            self.command.send_string(json.dumps(action, separators=(",", ":")), flags=zmq.NOBLOCK)
            return True
        except zmq.Again:
            return False

    def close(self) -> None:
        self.command.close(linger=0)
        self.observation.close(linger=0)
        if self.owns_context:
            self.context.term()
