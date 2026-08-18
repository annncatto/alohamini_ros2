from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field
from functools import partial

import rclpy
import yaml
import zmq
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import Point, Twist
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray

from .mapping import (
    LEVEL_TCP_QUATERNION,
    base_command,
    euler_delta_quaternion,
    normalize_stick,
    quaternion_multiply,
    step_orientation_toward,
    tcp_button_delta,
)


ARM_SUFFIXES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_yaw_joint",
    "wrist_roll",
)
ROOT_JOINTS = (
    "root_x_axis_joint",
    "root_y_axis_joint",
    "root_z_rotation_joint",
)


def arm_names(side: str) -> list[str]:
    return [f"{side}_{suffix}" for suffix in ARM_SUFFIXES]


def duration(seconds: float) -> Duration:
    seconds = max(0.0, float(seconds))
    whole = int(seconds)
    return Duration(sec=whole, nanosec=int((seconds - whole) * 1_000_000_000))


def pose_to_dict(pose) -> dict[str, list[float]]:
    return {
        "position": [pose.position.x, pose.position.y, pose.position.z],
        "orientation": [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ],
    }


def rotate_vector(orientation: list[float], vector: list[float]) -> list[float]:
    x, y, z, w = orientation
    vx, vy, vz = vector
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return [
        vx + w * tx + y * tz - z * ty,
        vy + w * ty + z * tx - x * tz,
        vz + w * tz + x * ty - y * tx,
    ]


def relative_pose(parent: dict, child: dict) -> dict:
    inverse = [
        -parent["orientation"][0],
        -parent["orientation"][1],
        -parent["orientation"][2],
        parent["orientation"][3],
    ]
    offset = [
        child["position"][index] - parent["position"][index]
        for index in range(3)
    ]
    return {
        "position": rotate_vector(inverse, offset),
        "orientation": quaternion_multiply(inverse, child["orientation"]),
    }


def compose_pose(parent: dict, child: dict) -> dict:
    rotated = rotate_vector(parent["orientation"], child["position"])
    return {
        "position": [
            parent["position"][index] + rotated[index] for index in range(3)
        ],
        "orientation": quaternion_multiply(
            parent["orientation"], child["orientation"]
        ),
    }


@dataclass
class ArmControl:
    active: bool = False
    pending_fk: bool = False
    pending_ik: bool = False
    target_pose: dict | None = None
    base_pose: dict | None = None
    orientation_reference: list[float] | None = None
    joy_anchor_rpy: list[float] | None = None
    joy_anchor_orientation: list[float] | None = None
    last_ik_request: float = 0.0
    goal_busy: bool = False
    goal_handle: object | None = None
    relatch_was_pressed: bool = False
    trigger_was_pressed: bool = False
    rejection_streak: int = 0


@dataclass
class InputSample:
    received_at: float
    payload: dict = field(default_factory=dict)


class JoyConTeleop(Node):
    def __init__(self) -> None:
        super().__init__("alohamini_joycon_teleop")
        defaults = {
            "hardware_mode": False,
            "input_endpoint": "tcp://127.0.0.1:5567",
            "input_timeout_sec": 0.25,
            "measured_state_timeout_sec": 0.25,
            "control_rate_hz": 20.0,
            "ik_rate_hz": 8.0,
            "ik_timeout_sec": 0.3,
            "deadzone": 0.25,
            "tcp_speed_m_s": 0.03,
            "orientation_scale": 1.0,
            "orientation_deadband_rad": 0.02,
            "max_orientation_delta_rad": 1.2,
            "orientation_speed_rad_s": 0.8,
            "imu_latch_reference": "level",
            "preview_home": "reference",
            "max_joint_step_rad": 0.10,
            "max_ik_jump_rad": 0.5,
            "arm_goal_max_velocity_rad_s": 0.4,
            "avoid_collisions": True,
            "arm_trajectory_duration_sec": 0.35,
            "base_linear_speed_m_s": 0.10,
            "base_angular_speed_rad_s": 0.5,
            "lift_speed_m_s": 0.02,
            "lift_command_period_sec": 0.4,
            "preview_joint_states_topic": "/alohamini_plan_only/joint_states",
            "measured_joint_states_topic": "/joint_states",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.hardware_mode = bool(self.get_parameter("hardware_mode").value)
        self.input_timeout = float(self.get_parameter("input_timeout_sec").value)
        self.measured_state_timeout = float(
            self.get_parameter("measured_state_timeout_sec").value
        )
        self.deadzone = float(self.get_parameter("deadzone").value)
        self.ik_period = 1.0 / float(self.get_parameter("ik_rate_hz").value)
        self.ik_timeout = float(self.get_parameter("ik_timeout_sec").value)
        self.orientation_deadband = float(
            self.get_parameter("orientation_deadband_rad").value
        )
        self.imu_latch_reference = str(
            self.get_parameter("imu_latch_reference").value
        )
        if self.imu_latch_reference not in ("level", "current"):
            raise ValueError("imu_latch_reference must be 'level' or 'current'")
        self.callback_group = ReentrantCallbackGroup()

        calibration = get_package_share_directory("alohamini_calibration")
        with open(
            f"{calibration}/config/hardware/arm_home.yaml", encoding="utf-8"
        ) as stream:
            installed_home = yaml.safe_load(stream)["offline_initial_joint_positions"]
        self.positions = {name: float(value) for name, value in installed_home.items()}
        if str(self.get_parameter("preview_home").value) == "reference":
            # Teleop rest pose: arm extended forward with the gripper level
            # (approach axis straight down), matching the MoveIt FK golden
            # "reference" sample. Hardware mode replaces this with measured
            # /joint_states anyway; this only shapes the offline preview.
            reference = {
                "shoulder_pan": 0.0,
                "shoulder_lift": -1.571,
                "elbow_flex": 1.571,
                "wrist_flex": 0.0,
                "wrist_yaw_joint": 0.0,
                "wrist_roll": 0.0,
            }
            for side in ("left", "right"):
                for joint, value in reference.items():
                    self.positions[f"{side}_{joint}"] = value
        self.positions.update(
            {
                "root_x_axis_joint": 0.0,
                "root_y_axis_joint": 0.0,
                "root_z_rotation_joint": 0.0,
                "vertical_move": 0.0,
                "wheel1_joint": 0.0,
                "wheel2_joint": 0.0,
                "wheel3_joint": 0.0,
            }
        )
        self.samples: dict[str, InputSample] = {}
        self.arms = {side: ArmControl() for side in ("left", "right")}
        self.preview_base = [0.0, 0.0, 0.0]
        self.lift_target = self.positions["vertical_move"]
        self.measured_lift = self.lift_target
        self.lift_active = False
        self.lift_goal_busy = False
        self.lift_goal_handle = None
        self.last_lift_goal = 0.0
        self.base_was_active = False
        self.last_loop = time.monotonic()
        self.last_stale_warning = 0.0
        self.last_measured_state = None
        self.last_ik_warning: dict[str, float] = {}

        self.zmq_context = zmq.Context()
        self.input_socket = self.zmq_context.socket(zmq.SUB)
        self.input_socket.setsockopt(zmq.LINGER, 0)
        self.input_socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self.input_socket.connect(str(self.get_parameter("input_endpoint").value))

        measured_topic = str(
            self.get_parameter("measured_joint_states_topic").value
        )
        self.create_subscription(JointState, measured_topic, self.on_joint_state, 20)
        self.preview_pub = self.create_publisher(
            JointState,
            str(self.get_parameter("preview_joint_states_topic").value),
            10,
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, "/alohamini/joycon_tcp_markers", 10
        )
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.fk_client = self.create_client(
            GetPositionFK, "/compute_fk", callback_group=self.callback_group
        )
        self.ik_client = self.create_client(
            GetPositionIK, "/compute_ik", callback_group=self.callback_group
        )
        self.arm_clients = {
            side: ActionClient(
                self,
                FollowJointTrajectory,
                f"/{side}_arm_controller/follow_joint_trajectory",
                callback_group=self.callback_group,
            )
            for side in ("left", "right")
        }
        self.gripper_clients = {
            side: ActionClient(
                self,
                GripperCommand,
                f"/{side}_gripper_controller/gripper_cmd",
                callback_group=self.callback_group,
            )
            for side in ("left", "right")
        }
        self.lift_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/lift_controller/follow_joint_trajectory",
            callback_group=self.callback_group,
        )
        rate = float(self.get_parameter("control_rate_hz").value)
        self.timer = self.create_timer(
            1.0 / rate, self.on_timer, callback_group=self.callback_group
        )
        mode = "HARDWARE" if self.hardware_mode else "PREVIEW"
        self.get_logger().info(
            f"Joy-Con teleop started in {mode} mode; it never enables bridge commands"
        )

    def destroy_node(self):
        self.input_socket.close()
        self.zmq_context.term()
        return super().destroy_node()

    def on_joint_state(self, message: JointState) -> None:
        if not self.hardware_mode:
            return
        for name, value in zip(message.name, message.position, strict=False):
            if math.isfinite(value):
                self.positions[name] = float(value)
        if "vertical_move" in message.name:
            self.measured_lift = self.positions["vertical_move"]
            if not self.lift_active:
                self.lift_target = self.measured_lift
        if any(name in message.name for name in arm_names("left")) or any(
            name in message.name for name in arm_names("right")
        ):
            self.last_measured_state = time.monotonic()

    def receive_inputs(self, now: float) -> None:
        while True:
            try:
                payload = json.loads(self.input_socket.recv_string(flags=zmq.NOBLOCK))
            except zmq.Again:
                return
            except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                self.get_logger().warning(f"Rejected malformed Joy-Con input: {exc}")
                continue
            side = payload.get("side")
            if payload.get("schema_version") != 1 or side not in self.arms:
                self.get_logger().warning("Rejected incompatible Joy-Con input")
                continue
            self.samples[side] = InputSample(now, payload)

    def sample(self, side: str, now: float) -> dict | None:
        item = self.samples.get(side)
        if item is None or now - item.received_at > self.input_timeout:
            return None
        return item.payload

    def fill_robot_state(self, state) -> None:
        state.joint_state.name = list(self.positions)
        state.joint_state.position = list(self.positions.values())

    def arm_seed(self, side: str) -> list[float] | None:
        names = arm_names(side)
        if any(name not in self.positions for name in names):
            return None
        return [self.positions[name] for name in names]

    def request_fk(self, side: str, sample: dict | None = None) -> None:
        arm = self.arms[side]
        seed = self.arm_seed(side)
        if arm.pending_fk or seed is None:
            return
        request = GetPositionFK.Request()
        request.header.frame_id = "root"
        request.fk_link_names = [f"{side}_Base", f"{side}_tcp"]
        self.fill_robot_state(request.robot_state)
        arm.pending_fk = True
        future = self.fk_client.call_async(request)
        future.add_done_callback(partial(self.on_fk, side))

    def on_fk(self, side: str, future) -> None:
        arm = self.arms[side]
        arm.pending_fk = False
        try:
            response = future.result()
            if (
                response.error_code.val != response.error_code.SUCCESS
                or len(response.pose_stamped) != 2
            ):
                raise RuntimeError(f"FK error {response.error_code.val}")
            base = pose_to_dict(response.pose_stamped[0].pose)
            tcp = pose_to_dict(response.pose_stamped[1].pose)
            arm.base_pose = base
            arm.target_pose = relative_pose(base, tcp)
            if self.imu_latch_reference == "level":
                # Rotate toward the level-grasp orientation gradually instead
                # of jumping the target, so the first IK requests stay within
                # the joint-step filter (the preview already starts level).
                arm.orientation_reference = list(LEVEL_TCP_QUATERNION)
            else:
                arm.orientation_reference = None
            arm.joy_anchor_rpy = None
            arm.joy_anchor_orientation = None
            self.publish_markers(side, arm.target_pose)
        except Exception as exc:
            self.get_logger().warning(f"{side} arm FK latch failed: {exc}")

    def request_ik(
        self,
        side: str,
        proposed_pose: dict,
        now: float,
        allow_position_fallback: bool = False,
        mode: str = "kdl",
    ) -> None:
        arm = self.arms[side]
        seed = self.arm_seed(side)
        if arm.pending_ik or arm.base_pose is None or seed is None:
            return
        group_suffix = {"kdl": "", "lma": "_lma", "position": "_pos"}[mode]
        root_pose = compose_pose(arm.base_pose, proposed_pose)
        request = GetPositionIK.Request()
        ik = request.ik_request
        ik.group_name = f"{side}_arm{group_suffix}"
        ik.ik_link_name = f"{side}_tcp"
        ik.pose_stamped.header.frame_id = "root"
        (
            ik.pose_stamped.pose.position.x,
            ik.pose_stamped.pose.position.y,
            ik.pose_stamped.pose.position.z,
        ) = root_pose["position"]
        (
            ik.pose_stamped.pose.orientation.x,
            ik.pose_stamped.pose.orientation.y,
            ik.pose_stamped.pose.orientation.z,
            ik.pose_stamped.pose.orientation.w,
        ) = root_pose["orientation"]
        self.fill_robot_state(ik.robot_state)
        ik.avoid_collisions = bool(self.get_parameter("avoid_collisions").value)
        ik.timeout = duration(self.ik_timeout)
        arm.pending_ik = True
        arm.last_ik_request = now
        future = self.ik_client.call_async(request)
        future.add_done_callback(
            partial(
                self.on_ik,
                side,
                proposed_pose,
                seed,
                mode,
                allow_position_fallback,
            )
        )

    def warn_ik(self, side: str, message: str) -> None:
        """Rate-limited per-side warning so IK failures are visible without
        flooding the terminal at control rate."""
        now = time.monotonic()
        if now - self.last_ik_warning.get(side, 0.0) < 1.0:
            return
        self.last_ik_warning[side] = now
        self.get_logger().warning(f"[{side}] {message}")

    def retry_ik_mode(
        self, side: str, proposed_pose: dict, reason: str
    ) -> None:
        self.get_logger().info(
            f"[{side}] {reason}; retrying with LMA IK "
            f"pos={[round(v, 4) for v in proposed_pose['position']]} "
            f"orient={[round(v, 4) for v in proposed_pose['orientation']]} "
            f"seed={[round(v, 4) for v in self.arm_seed(side) or []]}"
        )
        self.request_ik(
            side,
            proposed_pose,
            time.monotonic(),
            allow_position_fallback=False,
            mode="lma",
        )

    def retry_position_only(
        self, side: str, proposed_pose: dict, reason: str
    ) -> None:
        self.get_logger().info(
            f"[{side}] {reason}; retrying with position-only IK "
            f"({side}_arm_pos group)"
        )
        self.request_ik(
            side,
            proposed_pose,
            time.monotonic(),
            allow_position_fallback=False,
            mode="position",
        )

    def on_ik(
        self,
        side: str,
        proposed_pose: dict,
        seed: list[float],
        mode: str,
        allow_position_fallback: bool,
        future,
    ) -> None:
        arm = self.arms[side]
        arm.pending_ik = False
        try:
            response = future.result()
            if response.error_code.val != response.error_code.SUCCESS:
                code = response.error_code.val
                hint = {
                    -31: "NO_IK_SOLUTION (unreachable pose)",
                    -6: "TIMED_OUT (solver timeout)",
                    -21: "FRAME_TRANSFORM_FAILURE",
                }.get(code, f"error {code}")
                self.warn_ik(side, f"IK failed: {hint} [{mode}]")
                if mode == "kdl":
                    self.retry_ik_mode(side, proposed_pose, hint)
                elif mode == "lma" and allow_position_fallback:
                    self.retry_position_only(side, proposed_pose, hint)
                return
            solution = dict(
                zip(
                    response.solution.joint_state.name,
                    response.solution.joint_state.position,
                    strict=True,
                )
            )
            names = arm_names(side)
            candidate = [float(solution[name]) for name in names]
            max_step = float(self.get_parameter("max_joint_step_rad").value)
            worst = max(
                range(len(names)),
                key=lambda index: abs(candidate[index] - seed[index]),
            )
            worst_step = abs(candidate[worst] - seed[worst])
            if worst_step > max_step:
                arm.rejection_streak += 1
                max_jump = float(self.get_parameter("max_ik_jump_rad").value)
                if arm.rejection_streak >= 2 and worst_step <= max_jump:
                    # The solver occasionally needs a branch change (elbow
                    # flip) near the reference pose; after repeated identical
                    # rejections, allow one bounded larger step so the arm
                    # does not stick. The goal duration is scaled by the step.
                    self.get_logger().info(
                        f"[{side}] accepting larger IK step after repeated "
                        f"rejections ({names[worst]} {worst_step:.3f} rad)"
                    )
                    arm.rejection_streak = 0
                elif mode == "kdl":
                    self.retry_ik_mode(
                        side,
                        proposed_pose,
                        f"step {worst_step:.3f} rad > {max_step} rad",
                    )
                    return
                elif mode == "lma" and allow_position_fallback:
                    self.retry_position_only(
                        side,
                        proposed_pose,
                        f"step {worst_step:.3f} rad > {max_step} rad",
                    )
                    return
                else:
                    self.warn_ik(
                        side,
                        f"IK solution rejected by max_joint_step_rad "
                        f"({names[worst]} step {worst_step:.3f} rad > {max_step}) "
                        f"[{mode}]",
                    )
                    return
            else:
                arm.rejection_streak = 0
            arm.target_pose = proposed_pose
            for name, value in zip(names, candidate, strict=True):
                if not self.hardware_mode:
                    self.positions[name] = value
            self.publish_markers(side, proposed_pose)
            if self.hardware_mode:
                self.send_arm_goal(side, candidate)
            if mode == "position":
                # A position-only solution ignores the requested orientation;
                # re-latch FK so the target pose and markers show the true
                # achieved pose.
                self.request_fk(side)
        except Exception as exc:
            self.get_logger().warning(f"{side} arm IK failed: {exc}")

    def send_arm_goal(self, side: str, positions: list[float]) -> None:
        arm = self.arms[side]
        client = self.arm_clients[side]
        if arm.goal_busy or not client.server_is_ready():
            return
        names = arm_names(side)
        worst_step = max(
            (
                abs(target - self.positions.get(name, target))
                for name, target in zip(names, positions, strict=True)
            ),
            default=0.0,
        )
        goal_duration = max(
            float(self.get_parameter("arm_trajectory_duration_sec").value),
            worst_step / float(self.get_parameter("arm_goal_max_velocity_rad_s").value),
        )
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = names
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = duration(goal_duration)
        goal.trajectory.points = [point]
        arm.goal_busy = True
        future = client.send_goal_async(goal)
        future.add_done_callback(partial(self.on_arm_goal_response, side))

    def on_arm_goal_response(self, side: str, future) -> None:
        arm = self.arms[side]
        try:
            handle = future.result()
            if not handle.accepted:
                arm.goal_busy = False
                return
            arm.goal_handle = handle
            if not arm.active:
                handle.cancel_goal_async()
            result = handle.get_result_async()
            result.add_done_callback(partial(self.on_arm_result, side))
        except Exception:
            arm.goal_busy = False

    def on_arm_result(self, side: str, _future) -> None:
        arm = self.arms[side]
        arm.goal_busy = False
        arm.goal_handle = None

    def deactivate_arm(self, side: str) -> None:
        """Stop new arm targets; keep the last latch so Home re-latch and the
        TCP marker persist while the arm is idle."""
        arm = self.arms[side]
        if arm.active and self.hardware_mode and arm.goal_handle is not None:
            arm.goal_handle.cancel_goal_async()
        arm.active = False

    def update_arm(
        self,
        side: str,
        sample: dict | None,
        now: float,
        dt: float,
    ) -> None:
        arm = self.arms[side]
        if sample is None:
            self.deactivate_arm(side)
            arm.joy_anchor_rpy = None
            return
        buttons = sample["buttons"]
        delta = tcp_button_delta(
            buttons,
            float(self.get_parameter("tcp_speed_m_s").value),
            dt,
        )
        position_changed = any(abs(value) > 1.0e-8 for value in delta)
        # IMU tilt only adjusts TCP orientation while an SL/SR rail button of
        # that controller is held, so normal position moves never drift.
        orientation_mode = bool(buttons.get("sl") or buttons.get("sr"))
        engaged = position_changed or orientation_mode
        relatch = bool(buttons["relatch"])
        relatch_pressed = relatch and not arm.relatch_was_pressed
        arm.relatch_was_pressed = relatch
        if relatch_pressed and not arm.pending_fk:
            self.get_logger().info(
                f"{side} arm relatch requested (Home/Capture)"
            )
            self.request_fk(side, sample)
        if not engaged:
            self.deactivate_arm(side)
            arm.joy_anchor_rpy = None
        elif not arm.active:
            arm.active = True
            # Only the first-ever engage (or an explicit Home relatch) runs
            # FK and resets the orientation reference; later engages keep the
            # adjusted TCP pose so releasing and re-pressing a button never
            # undoes the operator's orientation or triggers a large, rejected
            # level correction.
            if arm.target_pose is None and not arm.pending_fk:
                self.request_fk(side, sample)
        if not orientation_mode:
            arm.joy_anchor_rpy = None

        trigger = bool(buttons["trigger"])
        if trigger and not arm.trigger_was_pressed:
            self.toggle_gripper(side)
        arm.trigger_was_pressed = trigger
        if not arm.active or arm.target_pose is None or arm.pending_fk:
            return
        if now - arm.last_ik_request < self.ik_period:
            return
        proposed = {
            "position": [
                arm.target_pose["position"][index] + delta[index]
                for index in range(3)
            ],
            "orientation": list(arm.target_pose["orientation"]),
        }
        orientation_changed = False
        if orientation_mode:
            # Anchor on press: the commanded TCP attitude is the pose at
            # press time rotated by the Joy-Con attitude change since then,
            # stepped at orientation_speed_rad_s. The controller's absolute
            # attitude is irrelevant, so re-pressing SL/SR continues from the
            # current gripper orientation without returning to any pose.
            if arm.joy_anchor_rpy is None:
                arm.joy_anchor_rpy = [
                    float(value) for value in sample["orientation_rpy"]
                ]
                arm.joy_anchor_orientation = list(
                    arm.target_pose["orientation"]
                )
            else:
                max_delta = float(
                    self.get_parameter("max_orientation_delta_rad").value
                )
                scale = float(self.get_parameter("orientation_scale").value)
                deltas = [
                    max(-max_delta, min(max_delta, scale * (current - anchor)))
                    for current, anchor in zip(
                        sample["orientation_rpy"], arm.joy_anchor_rpy, strict=True
                    )
                ]
                if max(abs(value) for value in deltas) > self.orientation_deadband:
                    commanded = quaternion_multiply(
                        arm.joy_anchor_orientation,
                        euler_delta_quaternion(*deltas),
                    )
                    proposed["orientation"] = step_orientation_toward(
                        proposed["orientation"],
                        commanded,
                        float(
                            self.get_parameter("orientation_speed_rad_s").value
                        )
                        * dt,
                    )
                    orientation_changed = True
        if arm.orientation_reference is not None:
            # Gradual leveling after the first latch: step the target toward
            # the level-grasp reference at the configured speed, then stop.
            proposed["orientation"] = step_orientation_toward(
                proposed["orientation"],
                arm.orientation_reference,
                float(self.get_parameter("orientation_speed_rad_s").value) * dt,
            )
            dot = sum(
                a * b
                for a, b in zip(
                    proposed["orientation"], arm.orientation_reference, strict=True
                )
            )
            if 2.0 * math.acos(max(-1.0, min(1.0, abs(dot)))) < 0.005:
                arm.orientation_reference = None
            else:
                orientation_changed = True
        if position_changed or orientation_changed:
            self.request_ik(
                side, proposed, now,
                allow_position_fallback=position_changed,
            )

    def toggle_gripper(self, side: str) -> None:
        name = f"{side}_gripper"
        closed = 0.32
        opened = -1.8030294104
        target = opened if self.positions.get(name, closed) > -0.7 else closed
        if not self.hardware_mode:
            self.positions[name] = target
            return
        client = self.gripper_clients[side]
        if not client.server_is_ready():
            return
        goal = GripperCommand.Goal()
        goal.command.position = target
        goal.command.max_effort = 0.0
        client.send_goal_async(goal)

    def update_base(
        self,
        left: dict | None,
        right: dict | None,
        lift_active: bool,
        dt: float,
    ) -> None:
        sticks = []
        turn = 0.0
        if right is not None:
            if bool(right["buttons"]["shoulder"]):
                # R + right stick left/right: base turn only.
                turn += normalize_stick(right["stick"][0], self.deadzone)
            else:
                sticks.append(
                    (
                        normalize_stick(right["stick"][0], self.deadzone),
                        normalize_stick(right["stick"][1], self.deadzone),
                    )
                )
        if left is not None:
            if lift_active or bool(left["buttons"]["shoulder"]):
                # L held: the left stick is reserved for the lift (vertical)
                # and contributes no base translation.
                pass
            else:
                sticks.append(
                    (
                        normalize_stick(left["stick"][0], self.deadzone),
                        normalize_stick(left["stick"][1], self.deadzone),
                    )
                )
        vx, vy, yaw = base_command(
            sticks,
            float(self.get_parameter("base_linear_speed_m_s").value),
            float(self.get_parameter("base_angular_speed_rad_s").value),
        )
        yaw = -max(-1.0, min(1.0, turn)) * float(
            self.get_parameter("base_angular_speed_rad_s").value
        )
        active = any(
            abs(value) > 1.0e-9 for value in (vx, vy, yaw)
        )
        if active:
            if self.hardware_mode:
                message = Twist()
                message.linear.x, message.linear.y, message.angular.z = vx, vy, yaw
                self.cmd_vel_pub.publish(message)
            else:
                self.preview_base[0] += vy * dt
                self.preview_base[1] -= vx * dt
                self.preview_base[2] += yaw * dt
                for name, value in zip(ROOT_JOINTS, self.preview_base, strict=True):
                    self.positions[name] = value
            self.base_was_active = True
        elif self.base_was_active:
            if self.hardware_mode:
                self.cmd_vel_pub.publish(Twist())
            self.base_was_active = False

    def update_lift(
        self, sample: dict | None, active: bool, now: float, dt: float
    ) -> None:
        if not active:
            if self.lift_active and self.hardware_mode:
                if self.lift_goal_handle is not None:
                    self.lift_goal_handle.cancel_goal_async()
                self.lift_target = self.measured_lift
            self.lift_active = False
            return
        if not self.lift_active:
            self.lift_active = True
            if self.hardware_mode:
                self.lift_target = self.measured_lift
        vertical = normalize_stick(sample["stick"][1], self.deadzone)
        if vertical == 0.0:
            return
        speed = float(self.get_parameter("lift_speed_m_s").value)
        self.lift_target = max(-0.3, min(0.3, self.lift_target + vertical * speed * dt))
        if not self.hardware_mode:
            self.positions["vertical_move"] = self.lift_target
            return
        period = float(self.get_parameter("lift_command_period_sec").value)
        if self.lift_goal_busy or now - self.last_lift_goal < period:
            return
        if not self.lift_client.server_is_ready():
            return
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ["vertical_move"]
        point = JointTrajectoryPoint()
        point.positions = [self.lift_target]
        point.time_from_start = duration(max(period * 1.5, 0.5))
        goal.trajectory.points = [point]
        self.lift_goal_busy = True
        self.last_lift_goal = now
        future = self.lift_client.send_goal_async(goal)
        future.add_done_callback(self.on_lift_goal_response)

    def on_lift_goal_response(self, future) -> None:
        try:
            handle = future.result()
            if not handle.accepted:
                self.lift_goal_busy = False
                return
            self.lift_goal_handle = handle
            if not self.lift_active:
                handle.cancel_goal_async()
            handle.get_result_async().add_done_callback(self.on_lift_result)
        except Exception:
            self.lift_goal_busy = False

    def on_lift_result(self, _future) -> None:
        self.lift_goal_busy = False
        self.lift_goal_handle = None

    def publish_preview(self) -> None:
        if self.hardware_mode:
            return
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(self.positions)
        message.position = list(self.positions.values())
        self.preview_pub.publish(message)

    def publish_markers(self, side: str, pose: dict) -> None:
        marker = Marker()
        marker.header.frame_id = f"{side}_Base"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "joycon_tcp_target"
        marker.id = 0 if side == "left" else 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = Point(
            x=float(pose["position"][0]),
            y=float(pose["position"][1]),
            z=float(pose["position"][2]),
        )
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.025
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = (
            (0.2, 1.0, 0.2, 1.0) if side == "left" else (1.0, 0.7, 0.1, 1.0)
        )
        marker.frame_locked = True
        array = MarkerArray()
        array.markers = [marker]
        self.marker_pub.publish(array)

    def on_timer(self) -> None:
        now = time.monotonic()
        dt = min(0.1, max(0.0, now - self.last_loop))
        self.last_loop = now
        self.receive_inputs(now)
        left = self.sample("left", now)
        right = self.sample("right", now)
        if left is None and right is None and now - self.last_stale_warning > 5.0:
            self.get_logger().warning("No fresh Joy-Con input; no new robot targets")
            self.last_stale_warning = now
        measured_fresh = bool(
            not self.hardware_mode
            or (
                self.last_measured_state is not None
                and now - self.last_measured_state <= self.measured_state_timeout
            )
        )
        if not measured_fresh:
            left = None
            right = None
        lift_active = bool(
            left is not None and left["buttons"]["shoulder"]
        )
        self.update_lift(left, lift_active, now, dt)
        self.update_base(left, right, lift_active, dt)
        self.update_arm("left", left, now, dt)
        self.update_arm("right", right, now, dt)
        self.publish_preview()


def main() -> None:
    rclpy.init()
    node = JoyConTeleop()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
