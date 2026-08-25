from __future__ import annotations

import math
import time
from collections.abc import Iterable
from dataclasses import dataclass
from enum import Enum
from threading import RLock

from .protocol import BodyVelocity, CommandGate, JointMapper

LEFT_ARM_JOINTS = (
    "left_shoulder_pan",
    "left_shoulder_lift",
    "left_elbow_flex",
    "left_wrist_flex",
    "left_wrist_yaw_joint",
    "left_wrist_roll",
)
RIGHT_ARM_JOINTS = tuple(name.replace("left_", "right_", 1) for name in LEFT_ARM_JOINTS)
LEFT_GRIPPER_JOINTS = ("left_gripper",)
RIGHT_GRIPPER_JOINTS = ("right_gripper",)
LIFT_JOINTS = ("vertical_move",)


class TerminalState(Enum):
    SUCCEEDED = "succeeded"
    CANCELED = "canceled"
    PREEMPTED = "preempted"
    ABORTED = "aborted"
    GOAL_TOLERANCE = "goal_tolerance"
    STALE = "stale"


@dataclass(frozen=True)
class TrajectorySample:
    time_from_start: float
    positions: dict[str, float]


@dataclass(frozen=True)
class TerminalEvent:
    goal_id: int
    state: TerminalState
    message: str


class TrajectoryResource:
    """One independently activated joint resource with no implicit zero targets."""

    def __init__(
        self,
        name: str,
        joints: Iterable[str],
        tracking_error: float,
        goal_tolerance: float,
        goal_time_tolerance: float,
        hold_duration: float,
    ) -> None:
        self.name = name
        self.joints = tuple(joints)
        self.tracking_error = float(tracking_error)
        self.goal_tolerance = float(goal_tolerance)
        self.goal_time_tolerance = float(goal_time_tolerance)
        self.hold_duration = float(hold_duration)
        if not math.isfinite(self.tracking_error) or self.tracking_error <= 0.0:
            raise ValueError(f"{name} tracking_error must be finite and positive")
        if not math.isfinite(self.goal_tolerance) or self.goal_tolerance <= 0.0:
            raise ValueError(f"{name} goal_tolerance must be finite and positive")
        if not math.isfinite(self.goal_time_tolerance) or self.goal_time_tolerance < 0.0:
            raise ValueError(f"{name} goal_time_tolerance must be finite and non-negative")
        if self.hold_duration != math.inf and (
            not math.isfinite(self.hold_duration) or self.hold_duration < 0.0
        ):
            raise ValueError(f"{name} hold_duration must be finite and non-negative")
        self.goal_id = 0
        self.active_goal_id: int | None = None
        self.start_time = 0.0
        self.start_positions: dict[str, float] = {}
        self.samples: tuple[TrajectorySample, ...] = ()
        self.hold_positions: dict[str, float] | None = None
        self.hold_until = 0.0
        self.stream_positions: dict[str, float] | None = None
        self.stream_until = 0.0
        self.desired: dict[str, float] | None = None
        self.terminals: dict[int, TerminalEvent] = {}

    @property
    def active(self) -> bool:
        return (
            self.active_goal_id is not None
            or self.hold_positions is not None
            or self.stream_positions is not None
        )

    def accept_stream_target(
        self,
        positions: dict[str, float],
        measured: dict[str, float],
        now: float,
        timeout: float,
    ) -> None:
        """Replace the latest realtime setpoint without creating an Action goal."""
        if set(positions) != set(self.joints):
            raise ValueError(f"{self.name} stream must contain exactly {self.joints}")
        if any(not math.isfinite(value) for value in positions.values()):
            raise ValueError(f"{self.name} stream positions must be finite")
        if any(joint not in measured for joint in self.joints):
            raise ValueError(f"fresh measured state lacks {self.joints}")
        if self.active_goal_id is not None:
            self._finish(
                TerminalState.PREEMPTED,
                "preempted by realtime JointJog",
                measured,
                now,
                hold=False,
            )
        self.hold_positions = None
        self.stream_positions = dict(positions)
        self.stream_until = now + timeout

    def validate(self, joint_names: Iterable[str], samples: Iterable[TrajectorySample]) -> None:
        names = tuple(joint_names)
        points = tuple(samples)
        if not names or len(names) != len(set(names)):
            raise ValueError("trajectory joint_names must be non-empty and unique")
        unsupported = set(names) - set(self.joints)
        if unsupported:
            raise ValueError(f"{self.name} does not own joints {sorted(unsupported)}")
        if not points:
            raise ValueError("trajectory must contain at least one point")
        previous_time = -1.0
        for point in points:
            if not math.isfinite(point.time_from_start) or point.time_from_start < 0.0:
                raise ValueError("trajectory time_from_start must be finite and non-negative")
            if point.time_from_start <= previous_time:
                raise ValueError("trajectory times must be strictly increasing")
            if set(point.positions) != set(names):
                raise ValueError("every trajectory point must contain exactly joint_names")
            if any(not math.isfinite(value) for value in point.positions.values()):
                raise ValueError("trajectory positions must be finite")
            previous_time = point.time_from_start

    def activate(
        self,
        joint_names: Iterable[str],
        samples: Iterable[TrajectorySample],
        measured: dict[str, float],
        now: float,
    ) -> int:
        samples = tuple(samples)
        self.validate(joint_names, samples)
        missing = set(self.joints) - set(measured)
        if missing:
            raise ValueError(f"fresh measured state lacks {sorted(missing)}")
        if self.active_goal_id is not None:
            self._finish(
                TerminalState.PREEMPTED,
                "preempted by a newer trajectory",
                measured,
                now,
            )
        self.goal_id += 1
        self.active_goal_id = self.goal_id
        self.start_time = now
        self.start_positions = {joint: float(measured[joint]) for joint in self.joints}
        self.samples = samples
        self.hold_positions = None
        self.stream_positions = None
        self.desired = dict(self.start_positions)
        return self.goal_id

    def _finish(
        self,
        state: TerminalState,
        message: str,
        measured: dict[str, float] | None,
        now: float,
        hold: bool = True,
    ) -> None:
        if self.active_goal_id is not None:
            self.terminals[self.active_goal_id] = TerminalEvent(self.active_goal_id, state, message)
        self.active_goal_id = None
        self.samples = ()
        self.desired = None
        if hold and measured is not None and all(joint in measured for joint in self.joints):
            self.hold_positions = {joint: float(measured[joint]) for joint in self.joints}
            self.hold_until = now + self.hold_duration
        else:
            self.hold_positions = None
        self.stream_positions = None

    def cancel(self, goal_id: int, measured: dict[str, float], fresh: bool, now: float) -> bool:
        if goal_id != self.active_goal_id:
            return False
        self._finish(
            TerminalState.CANCELED,
            "trajectory canceled",
            measured if fresh else None,
            now,
            hold=fresh,
        )
        return True

    def invalidate_stale(self, now: float) -> None:
        self._finish(
            TerminalState.STALE,
            "Host observation became stale",
            None,
            now,
            hold=False,
        )

    def terminal(self, goal_id: int) -> TerminalEvent | None:
        # Each action execute callback consumes its terminal event exactly once. This
        # bounds memory when teleoperation continuously preempts short trajectories.
        return self.terminals.pop(goal_id, None)

    def _interpolate(self, elapsed: float) -> dict[str, float]:
        previous_time = 0.0
        previous = self.start_positions
        for point in self.samples:
            if elapsed <= point.time_from_start:
                duration = point.time_from_start - previous_time
                ratio = 1.0 if duration <= 0.0 else (elapsed - previous_time) / duration
                ratio = min(1.0, max(0.0, ratio))
                target = dict(previous)
                for joint, value in point.positions.items():
                    target[joint] = previous[joint] + ratio * (value - previous[joint])
                return target
            previous_time = point.time_from_start
            previous = {**previous, **point.positions}
        return dict(previous)

    def update(self, measured: dict[str, float], fresh: bool, now: float) -> dict[str, float] | None:
        if not fresh:
            if self.active:
                self.invalidate_stale(now)
            return None
        if self.stream_positions is not None:
            if now > self.stream_until:
                self.stream_positions = None
                return None
            error = max(abs(self.stream_positions[joint] - measured[joint]) for joint in self.joints)
            if error > self.tracking_error:
                self.stream_positions = None
                return None
            return dict(self.stream_positions)
        if self.active_goal_id is not None:
            desired = self._interpolate(now - self.start_time)
            error = max(abs(desired[joint] - measured[joint]) for joint in self.joints)
            if error > self.tracking_error:
                self._finish(
                    TerminalState.ABORTED,
                    f"tracking error {error:.6f} exceeds {self.tracking_error:.6f}",
                    measured,
                    now,
                )
                return dict(self.hold_positions) if self.hold_positions else None
            self.desired = desired
            elapsed = now - self.start_time
            end_time = self.samples[-1].time_from_start
            if elapsed >= end_time:
                if error <= self.goal_tolerance:
                    goal_id = self.active_goal_id
                    self.terminals[goal_id] = TerminalEvent(
                        goal_id,
                        TerminalState.SUCCEEDED,
                        f"goal reached with error {error:.6f}",
                    )
                    self.active_goal_id = None
                    self.samples = ()
                    self.hold_positions = desired
                    self.hold_until = now + self.hold_duration
                elif elapsed > end_time + self.goal_time_tolerance:
                    self._finish(
                        TerminalState.GOAL_TOLERANCE,
                        f"goal error {error:.6f} exceeds {self.goal_tolerance:.6f} "
                        f"after {self.goal_time_tolerance:.3f}s grace period",
                        measured,
                        now,
                    )
                    return dict(self.hold_positions) if self.hold_positions else None
            return desired
        if self.hold_positions is not None:
            if now <= self.hold_until:
                return dict(self.hold_positions)
            self.hold_positions = None
        return None


class CommandComposer:
    """Compose independently active resources into the bridge's sole Host action."""

    def __init__(
        self,
        mapper: JointMapper,
        command_timeout: float,
        arm_tracking_error: float,
        lift_tracking_error: float,
        hold_duration: float,
        arm_goal_tolerance: float = 0.03,
        lift_goal_tolerance: float = 0.003,
        goal_time_tolerance: float = 1.0,
        gripper_tracking_error: float = 0.5,
        gripper_goal_tolerance: float = 0.05,
        lift_jog_lookahead: float = 0.05,
        lift_jog_stop_settle: float = 0.1,
    ) -> None:
        self.mapper = mapper
        self.command_timeout = float(command_timeout)
        if not math.isfinite(self.command_timeout) or self.command_timeout <= 0.0:
            raise ValueError("command_timeout must be finite and positive")
        self.base = CommandGate()
        self.resources = {
            "left_arm": TrajectoryResource(
                "left_arm",
                LEFT_ARM_JOINTS,
                arm_tracking_error,
                arm_goal_tolerance,
                goal_time_tolerance,
                hold_duration,
            ),
            "right_arm": TrajectoryResource(
                "right_arm",
                RIGHT_ARM_JOINTS,
                arm_tracking_error,
                arm_goal_tolerance,
                goal_time_tolerance,
                hold_duration,
            ),
            "left_gripper": TrajectoryResource(
                "left_gripper",
                LEFT_GRIPPER_JOINTS,
                gripper_tracking_error,
                gripper_goal_tolerance,
                goal_time_tolerance,
                hold_duration,
            ),
            "right_gripper": TrajectoryResource(
                "right_gripper",
                RIGHT_GRIPPER_JOINTS,
                gripper_tracking_error,
                gripper_goal_tolerance,
                goal_time_tolerance,
                hold_duration,
            ),
            "lift": TrajectoryResource(
                "lift",
                LIFT_JOINTS,
                lift_tracking_error,
                lift_goal_tolerance,
                goal_time_tolerance,
                # The lift servo runs in velocity mode (the Host applies a
                # P-controller on the height error). Velocity-mode servos do
                # not self-lock: once the command stream stops, nothing
                # resists the leadscrew back-driving and the axis sinks.
                # Hold the last commanded height indefinitely so the Host
                # keeps applying corrective velocities; new goals preempt
                # the hold seamlessly.
                math.inf,
            ),
        }
        self.enabled = False
        self.lift_jog_lookahead = float(lift_jog_lookahead)
        if not math.isfinite(self.lift_jog_lookahead) or self.lift_jog_lookahead <= 0.0:
            raise ValueError("lift_jog_lookahead must be finite and positive")
        self.lift_jog_active = False
        self.lift_jog_velocity = 0.0
        self.lift_jog_time = 0.0
        self.lift_jog_stop_settle = float(lift_jog_stop_settle)
        if not math.isfinite(self.lift_jog_stop_settle) or self.lift_jog_stop_settle < 0.0:
            raise ValueError("lift_jog_stop_settle must be finite and non-negative")
        self.lift_jog_stop_pending = False
        self.lift_jog_stop_time = 0.0
        self.stale_stop_pending = False
        self.ever_commanded = False
        self.lock = RLock()

    def enable(self) -> None:
        with self.lock:
            self.enabled = True
            self.base.enable()
            self.stale_stop_pending = False
            self.ever_commanded = False
            self.lift_jog_active = False
            self.lift_jog_velocity = 0.0
            self.lift_jog_stop_pending = False
            for resource in self.resources.values():
                resource.active_goal_id = None
                resource.samples = ()
                resource.hold_positions = None
                resource.stream_positions = None
                resource.desired = None

    def disable(self) -> bool:
        with self.lock:
            should_stop = self.enabled and self.ever_commanded
            self.enabled = False
            self.base.disable()
            self.lift_jog_active = False
            self.lift_jog_velocity = 0.0
            self.lift_jog_stop_pending = False
            now = time.monotonic()
            for resource in self.resources.values():
                resource._finish(
                    TerminalState.ABORTED,
                    "ROS command channel disabled",
                    None,
                    now,
                    hold=False,
                )
                resource.stream_positions = None
            return should_stop

    def disable_action(
        self,
        measured: dict[str, float],
        metadata: dict,
        fresh: bool,
    ) -> dict[str, float] | None:
        """Build one final stop frame without inventing targets from stale state."""
        with self.lock:
            if not self.enabled or not self.ever_commanded:
                return None
            action = {"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}
            if not fresh:
                return action
            try:
                if self.lift_jog_active or self.lift_jog_stop_pending:
                    action["lift_axis.vel"] = 0.0
                for name, resource in self.resources.items():
                    if not resource.active:
                        continue
                    if name == "lift":
                        action["lift_axis.vel"] = 0.0
                        continue
                    for joint in resource.joints:
                        key, value = self.mapper.urdf_to_lerobot(joint, measured[joint], metadata)
                        action[key] = value
            except (KeyError, TypeError, ValueError, OverflowError):
                return {"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}
            return action

    def accept_base(self, velocity: BodyVelocity, now: float | None = None) -> bool:
        with self.lock:
            return self.base.accept(velocity, now)

    def accept_lift_jog(
        self,
        velocity: float,
        measured: dict[str, float],
        fresh: bool,
        now: float | None = None,
    ) -> None:
        """Accept a standard JointJog lift velocity without bypassing safety."""
        with self.lock:
            if not self.enabled:
                raise ValueError("ROS command channel is disabled")
            if not fresh or "vertical_move" not in measured:
                raise ValueError("fresh measured lift state is required")
            velocity = float(velocity)
            if not math.isfinite(velocity):
                raise ValueError("lift jog velocity must be finite")
            now = time.monotonic() if now is None else now
            lift = self.resources["lift"]
            if abs(velocity) <= 1.0e-9:
                if self.lift_jog_active:
                    self.lift_jog_active = False
                    self.lift_jog_velocity = 0.0
                    # Stop velocity first. The cached observation can trail a
                    # moving lift; immediately holding it would reverse the
                    # mechanism toward an older height. Latch position only
                    # after fresh observations have caught up.
                    lift.hold_positions = None
                    self.lift_jog_stop_pending = True
                    self.lift_jog_stop_time = now
                return
            if lift.active_goal_id is not None:
                lift._finish(
                    TerminalState.PREEMPTED,
                    "preempted by lift jog",
                    measured,
                    now,
                )
            lift.hold_positions = None
            self.lift_jog_active = True
            self.lift_jog_velocity = velocity
            self.lift_jog_time = now
            self.lift_jog_stop_pending = False

    def accept_arm_jog(
        self,
        resource: str,
        joint_names: Iterable[str],
        displacements: Iterable[float],
        measured: dict[str, float],
        fresh: bool,
        timeout: float,
        now: float | None = None,
    ) -> None:
        """Accept a latest-only arm setpoint carried by standard JointJog."""
        with self.lock:
            if not self.enabled:
                raise ValueError("ROS command channel is disabled")
            if resource not in ("left_arm", "right_arm"):
                raise ValueError(f"unsupported arm jog resource {resource}")
            if not fresh:
                raise ValueError("fresh Host observation is required")
            names = tuple(joint_names)
            values = tuple(float(value) for value in displacements)
            controller = self.resources[resource]
            if names != controller.joints or len(values) != len(names):
                raise ValueError(f"{resource} JointJog must contain joints in canonical order")
            target = {
                joint: float(measured[joint]) + displacement
                for joint, displacement in zip(names, values, strict=True)
            }
            controller.accept_stream_target(
                target,
                measured,
                time.monotonic() if now is None else now,
                timeout,
            )

    def start_trajectory(
        self,
        resource: str,
        joint_names: Iterable[str],
        samples: Iterable[TrajectorySample],
        measured: dict[str, float],
        fresh: bool,
        now: float | None = None,
    ) -> int:
        with self.lock:
            if not self.enabled:
                raise ValueError("ROS command channel is disabled")
            if not fresh:
                raise ValueError("fresh Host observation is required")
            if resource == "lift":
                self.lift_jog_active = False
                self.lift_jog_velocity = 0.0
                self.lift_jog_stop_pending = False
            return self.resources[resource].activate(
                joint_names,
                samples,
                measured,
                time.monotonic() if now is None else now,
            )

    def cancel_trajectory(
        self,
        resource: str,
        goal_id: int,
        measured: dict[str, float],
        fresh: bool,
        now: float | None = None,
    ) -> bool:
        with self.lock:
            return self.resources[resource].cancel(
                goal_id,
                measured,
                fresh,
                time.monotonic() if now is None else now,
            )

    def compose(
        self,
        measured: dict[str, float],
        metadata: dict,
        permitted: bool,
        now: float | None = None,
    ) -> dict[str, float] | None:
        with self.lock:
            if not self.enabled:
                return None
            now = time.monotonic() if now is None else now
            if permitted and any(
                resource.active and any(joint not in measured for joint in resource.joints)
                for resource in self.resources.values()
            ):
                permitted = False
            if not permitted:
                self.lift_jog_active = False
                self.lift_jog_velocity = 0.0
                self.lift_jog_stop_pending = False
                for resource in self.resources.values():
                    resource.update(measured, False, now)
                self.base.enable()  # invalidate every pre-stale base command
                if self.ever_commanded and not self.stale_stop_pending:
                    self.stale_stop_pending = True
                    self.ever_commanded = False
                    return {"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}
                return None
            self.stale_stop_pending = False
            action: dict[str, float] = {}
            if self.lift_jog_active and now - self.lift_jog_time > self.command_timeout:
                self.lift_jog_active = False
                self.lift_jog_velocity = 0.0
                self.resources["lift"].hold_positions = None
                self.lift_jog_stop_pending = True
                self.lift_jog_stop_time = now
            if self.lift_jog_stop_pending:
                action["lift_axis.vel"] = 0.0
                if now - self.lift_jog_stop_time >= self.lift_jog_stop_settle:
                    self.lift_jog_stop_pending = False
                    self.resources["lift"]._finish(
                        TerminalState.CANCELED,
                        "lift jog stopped at fresh settled measurement",
                        measured,
                        now,
                    )
            base = self.base.resolve(True, self.command_timeout, now)
            if base is not None:
                action.update(
                    {
                        "x.vel": base.x,
                        "y.vel": base.y,
                        "theta.vel": math.degrees(base.yaw),
                    }
                )
            try:
                for name, resource in self.resources.items():
                    if name == "lift" and self.lift_jog_active:
                        target = measured["vertical_move"] + math.copysign(
                            self.lift_jog_lookahead,
                            self.lift_jog_velocity,
                        )
                        target = max(-0.3, min(0.3, target))
                        action["lift_axis.height_mm"] = self.mapper.lift_urdf_to_height(target)
                        continue
                    targets = resource.update(measured, True, now)
                    if targets is None:
                        continue
                    if name == "lift":
                        action["lift_axis.height_mm"] = self.mapper.lift_urdf_to_height(
                            targets["vertical_move"]
                        )
                        continue
                    for joint, position in targets.items():
                        key, value = self.mapper.urdf_to_lerobot(joint, position, metadata)
                        action[key] = value
            except (KeyError, TypeError, ValueError, OverflowError) as error:
                for resource in self.resources.values():
                    resource._finish(
                        TerminalState.ABORTED,
                        f"Host action conversion failed: {error}",
                        None,
                        now,
                        hold=False,
                    )
                self.base.enable()
                if self.ever_commanded and not self.stale_stop_pending:
                    self.stale_stop_pending = True
                    self.ever_commanded = False
                    return {"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}
                return None
            if not action:
                return None
            # The verified LeRobot Host currently parses x/y/theta as a required
            # command envelope even for arm-only and lift-only actions.  Preserve
            # independent ROS resources while supplying an explicit stopped base
            # whenever no live base command exists.
            action.setdefault("x.vel", 0.0)
            action.setdefault("y.vel", 0.0)
            action.setdefault("theta.vel", 0.0)
            self.ever_commanded = True
            return action
