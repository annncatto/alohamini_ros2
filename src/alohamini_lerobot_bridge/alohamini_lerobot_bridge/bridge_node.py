from __future__ import annotations

import json
import math
import time
from functools import partial
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory, GripperCommand
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectoryPoint

from .control import CommandComposer, TerminalState, TrajectorySample

from .protocol import (
    BodyVelocity,
    JointMapper,
    ZmqHostTransport,
    lift_command_ready,
    validate_state_observation,
    wheel_velocity_from_body,
)


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value)) if limit > 0.0 else value


class AlohaMiniLeRobotBridge(Node):
    def __init__(self) -> None:
        super().__init__("alohamini_lerobot_bridge")
        defaults = {
            "host": "127.0.0.1",
            "command_port": 5555,
            "observation_port": 5556,
            "request_window": 3,
            "rate_hz": 30.0,
            "observation_timeout_sec": 0.5,
            "request_timeout_sec": 1.0,
            "command_timeout_sec": 0.5,
            "trajectory_hold_sec": 0.25,
            "arm_path_tolerance_rad": 0.35,
            "lift_path_tolerance_m": 0.03,
            "arm_goal_tolerance_rad": 0.03,
            "gripper_path_tolerance_rad": 0.5,
            "gripper_goal_tolerance_rad": 0.05,
            "gripper_command_duration_sec": 3.0,
            "lift_goal_tolerance_m": 0.003,
            "goal_time_tolerance_sec": 1.0,
            "expected_robot_model": "alohamini2pro",
            "require_model_match": True,
            "cmd_vel_topic": "/cmd_vel",
            "joint_states_topic": "/joint_states",
            "base_velocity_topic": "/alohamini/base_velocity",
            "base_frame": "base_link",
            "max_linear_speed": 0.25,
            "max_lateral_speed": 0.25,
            "max_angular_speed": 1.0,
            "linear_x_scale": 1.0,
            "linear_y_scale": 1.0,
            "angular_z_scale": 1.0,
            "swap_xy": False,
            "wheel_radius": 0.063,
            "base_radius": 0.195,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        calibration_share = Path(get_package_share_directory("alohamini_calibration"))
        joint_calibrations = {}
        for side in ("left", "right"):
            path = (
                calibration_share
                / f"config/hardware/hardware_joint_map_{side}.yaml"
            )
            with path.open(encoding="utf-8") as stream:
                joint_calibrations[side] = yaml.safe_load(stream)
        with (calibration_share / "config/hardware/lift_axis.yaml").open(
            encoding="utf-8"
        ) as stream:
            lift_calibration = yaml.safe_load(stream)
        self.mapper = JointMapper(joint_calibrations, lift_calibration)

        self.transport = ZmqHostTransport(
            str(self.get_parameter("host").value),
            int(self.get_parameter("command_port").value),
            int(self.get_parameter("observation_port").value),
            int(self.get_parameter("request_window").value),
        )
        self.expected_model = str(self.get_parameter("expected_robot_model").value)
        self.require_model_match = bool(self.get_parameter("require_model_match").value)
        self.obs_timeout = float(self.get_parameter("observation_timeout_sec").value)
        self.request_timeout = float(self.get_parameter("request_timeout_sec").value)
        self.command_timeout = float(self.get_parameter("command_timeout_sec").value)
        # Port 5555 has no lease protocol.  Never claim it at startup: a fresh
        # observation plus an explicit service call is required every run.
        self.composer = CommandComposer(
            self.mapper,
            self.command_timeout,
            float(self.get_parameter("arm_path_tolerance_rad").value),
            float(self.get_parameter("lift_path_tolerance_m").value),
            float(self.get_parameter("trajectory_hold_sec").value),
            float(self.get_parameter("arm_goal_tolerance_rad").value),
            float(self.get_parameter("lift_goal_tolerance_m").value),
            float(self.get_parameter("goal_time_tolerance_sec").value),
            float(self.get_parameter("gripper_path_tolerance_rad").value),
            float(self.get_parameter("gripper_goal_tolerance_rad").value),
        )
        self.gripper_command_duration = float(
            self.get_parameter("gripper_command_duration_sec").value
        )
        if (
            not math.isfinite(self.gripper_command_duration)
            or self.gripper_command_duration <= 0.0
        ):
            raise ValueError("gripper_command_duration_sec must be finite and positive")
        self.command_gate = self.composer.base
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.base_radius = float(self.get_parameter("base_radius").value)
        for name in ("linear_x_scale", "linear_y_scale", "angular_z_scale"):
            value = float(self.get_parameter(name).value)
            if not math.isfinite(value) or value == 0.0:
                raise ValueError(f"{name} must be finite and non-zero")

        self.latest_observation: dict | None = None
        self.latest_positions: dict[str, float] = {}
        self.robot_metadata: dict | None = None
        self.last_observation_monotonic: float | None = None
        self.request_expirations = 0
        self.malformed_responses = 0
        self.invalid_observations = 0
        self.last_observation_error = ""
        self.observation_count = 0
        self.command_count = 0
        self.measured = BodyVelocity()
        self.wheel_positions = [0.0, 0.0, 0.0]
        self.last_integrate = time.monotonic()

        self.joint_pub = self.create_publisher(
            JointState, str(self.get_parameter("joint_states_topic").value), 10
        )
        self.measured_joint_pub = self.create_publisher(
            JointState, "~/measured_joint_states", 10
        )
        self.derived_wheel_pub = self.create_publisher(
            JointState, "~/derived_wheel_states", 10
        )
        self.base_velocity_pub = self.create_publisher(
            TwistStamped, str(self.get_parameter("base_velocity_topic").value), 10
        )
        self.raw_pub = self.create_publisher(String, "~/state_json", 10)
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray, "/diagnostics", 10
        )
        self.create_subscription(
            Twist, str(self.get_parameter("cmd_vel_topic").value), self.on_cmd_vel, 10
        )
        self.create_service(SetBool, "~/command_enable", self.on_command_enable)
        self.action_callback_group = ReentrantCallbackGroup()
        self.action_servers = []
        for resource, action_name in (
            ("left_arm", "/left_arm_controller/follow_joint_trajectory"),
            ("right_arm", "/right_arm_controller/follow_joint_trajectory"),
            ("lift", "/lift_controller/follow_joint_trajectory"),
        ):
            self.action_servers.append(
                ActionServer(
                    self,
                    FollowJointTrajectory,
                    action_name,
                    execute_callback=partial(
                        self.execute_trajectory, resource=resource
                    ),
                    goal_callback=partial(self.on_trajectory_goal, resource=resource),
                    cancel_callback=self.on_trajectory_cancel,
                    callback_group=self.action_callback_group,
                )
            )
        for resource, action_name in (
            ("left_gripper", "/left_gripper_controller/gripper_cmd"),
            ("right_gripper", "/right_gripper_controller/gripper_cmd"),
        ):
            self.action_servers.append(
                ActionServer(
                    self,
                    GripperCommand,
                    action_name,
                    execute_callback=partial(
                        self.execute_gripper_command, resource=resource
                    ),
                    goal_callback=partial(self.on_gripper_goal, resource=resource),
                    cancel_callback=self.on_trajectory_cancel,
                    callback_group=self.action_callback_group,
                )
            )
        rate = max(1.0, float(self.get_parameter("rate_hz").value))
        self.timer = self.create_timer(1.0 / rate, self.on_timer)
        self.diagnostic_timer = self.create_timer(1.0, self.publish_diagnostics)
        self.get_logger().info(
            "LeRobot bridge started read-only; Host owns serial buses. "
            "Use ~/command_enable explicitly before ROS may publish to port 5555."
        )

    def observation_fresh(self) -> bool:
        return (
            self.last_observation_monotonic is not None
            and time.monotonic() - self.last_observation_monotonic <= self.obs_timeout
        )

    def model_matches(self) -> bool:
        if not isinstance(self.robot_metadata, dict):
            return False
        return self.robot_metadata.get("robot_model") == self.expected_model

    def lift_ready_for_commands(self) -> bool:
        return lift_command_ready(self.robot_metadata, self.latest_observation)

    def on_cmd_vel(self, message: Twist) -> None:
        if not self.command_gate.enabled:
            return
        raw_values = (message.linear.x, message.linear.y, message.angular.z)
        if any(not math.isfinite(float(value)) for value in raw_values):
            self.get_logger().warning("Rejected non-finite /cmd_vel")
            return
        x = clamp(
            float(message.linear.x), float(self.get_parameter("max_linear_speed").value)
        )
        y = clamp(
            float(message.linear.y),
            float(self.get_parameter("max_lateral_speed").value),
        )
        yaw = clamp(
            float(message.angular.z),
            float(self.get_parameter("max_angular_speed").value),
        )
        x *= float(self.get_parameter("linear_x_scale").value)
        y *= float(self.get_parameter("linear_y_scale").value)
        if bool(self.get_parameter("swap_xy").value):
            x, y = y, x
        self.composer.accept_base(
            BodyVelocity(
                x=x,
                y=y,
                yaw=yaw * float(self.get_parameter("angular_z_scale").value),
            )
        )

    def on_command_enable(self, request: SetBool.Request, response: SetBool.Response):
        if request.data:
            if not self.observation_fresh():
                response.success = False
                response.message = (
                    "Cannot enable commands without a fresh Host observation"
                )
                return response
            if self.require_model_match and not self.model_matches():
                response.success = False
                response.message = (
                    "Host robot_model does not match expected_robot_model"
                )
                return response
            # Re-enabling is also an epoch boundary. Stop an already-started
            # stream before discarding it, then wait for another new /cmd_vel.
            stop_action = self.composer.disable_action(
                self.latest_positions,
                self.robot_metadata or {},
                self.observation_fresh(),
            )
            self.composer.disable()
            if stop_action is not None:
                self.transport.send_action(stop_action)
            self.composer.enable()
            response.success = True
            response.message = (
                "ROS command channel enabled; every control resource remains unarmed "
                "until it receives a new command"
            )
            return response
        stop_action = self.composer.disable_action(
            self.latest_positions,
            self.robot_metadata or {},
            self.observation_fresh(),
        )
        self.composer.disable()
        if stop_action is not None:
            self.transport.send_action(stop_action)
        response.success = True
        response.message = "ROS command channel disabled"
        return response

    def on_timer(self) -> None:
        self.request_expirations += self.transport.expire_requests(self.request_timeout)
        self.transport.fill_state_requests()
        observation, malformed = self.transport.receive_latest()
        self.malformed_responses += malformed
        if observation is not None:
            self.handle_observation(observation)
        self.publish_wheel_states()
        self.send_command_if_enabled()

    def handle_observation(self, observation: dict) -> None:
        try:
            metadata, positions, host_velocity = validate_state_observation(
                observation, self.mapper
            )
        except (KeyError, TypeError, ValueError) as error:
            self.invalid_observations += 1
            error_text = str(error)
            if error_text != self.last_observation_error:
                self.get_logger().warning(f"Host observation rejected: {error_text}")
            self.last_observation_error = error_text
            return

        self.robot_metadata = metadata
        self.latest_observation = observation
        self.latest_positions = positions
        self.last_observation_monotonic = time.monotonic()
        self.observation_count += 1
        self.last_observation_error = ""
        host_x = host_velocity.x
        host_y = host_velocity.y
        if bool(self.get_parameter("swap_xy").value):
            host_x, host_y = host_y, host_x
        self.measured = BodyVelocity(
            x=host_x / float(self.get_parameter("linear_x_scale").value),
            y=host_y / float(self.get_parameter("linear_y_scale").value),
            yaw=math.radians(host_velocity.yaw)
            / float(self.get_parameter("angular_z_scale").value),
        )
        measured = TwistStamped()
        measured.header.stamp = self.get_clock().now().to_msg()
        measured.header.frame_id = self.base_frame
        measured.twist.linear.x = self.measured.x
        measured.twist.linear.y = self.measured.y
        measured.twist.angular.z = self.measured.yaw
        self.base_velocity_pub.publish(measured)
        self.raw_pub.publish(
            String(data=json.dumps(observation, separators=(",", ":")))
        )
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(positions)
        message.position = list(positions.values())
        self.joint_pub.publish(message)
        self.measured_joint_pub.publish(message)

    def publish_wheel_states(self) -> None:
        now_mono = time.monotonic()
        dt = now_mono - self.last_integrate
        self.last_integrate = now_mono
        velocity = self.measured if self.observation_fresh() else BodyVelocity()
        if 0.0 < dt < 1.0:
            wheel_velocities = wheel_velocity_from_body(
                velocity, self.wheel_radius, self.base_radius
            )
            for index, wheel_velocity in enumerate(wheel_velocities):
                self.wheel_positions[index] += wheel_velocity * dt
            wheels = JointState()
            wheels.header.stamp = self.get_clock().now().to_msg()
            # The MoveIt description has a virtual planar root made from three
            # ordinary URDF joints. This bridge has no odometry source, so keep
            # that virtual root explicitly at zero to anchor root -> base_link.
            # These values are model state, not measured hardware feedback.
            wheels.name = [
                "root_x_axis_joint",
                "root_y_axis_joint",
                "root_z_rotation_joint",
                "wheel1_joint",
                "wheel2_joint",
                "wheel3_joint",
            ]
            wheels.position = [0.0, 0.0, 0.0, *self.wheel_positions]
            wheels.velocity = [0.0, 0.0, 0.0, *wheel_velocities]
            self.joint_pub.publish(wheels)
            self.derived_wheel_pub.publish(wheels)

    def send_command_if_enabled(self) -> None:
        permitted = self.observation_fresh() and (
            self.model_matches() or not self.require_model_match
        )
        action = self.composer.compose(
            self.latest_positions,
            self.robot_metadata or {},
            permitted,
        )
        if action is None:
            return
        if self.transport.send_action(action):
            self.command_count += 1

    @staticmethod
    def trajectory_samples(goal) -> tuple[TrajectorySample, ...]:
        names = tuple(goal.trajectory.joint_names)
        samples = []
        for point in goal.trajectory.points:
            if len(point.positions) != len(names):
                raise ValueError(
                    "trajectory point position count does not match joint_names"
                )
            duration = point.time_from_start
            samples.append(
                TrajectorySample(
                    float(duration.sec) + float(duration.nanosec) * 1e-9,
                    dict(zip(names, (float(value) for value in point.positions))),
                )
            )
        return tuple(samples)

    def validate_trajectory_for_host(self, resource: str, goal) -> None:
        samples = self.trajectory_samples(goal)
        controller = self.composer.resources[resource]
        controller.validate(goal.trajectory.joint_names, samples)
        metadata = self.robot_metadata or {}
        for sample in samples:
            for joint, position in sample.positions.items():
                if resource == "lift":
                    self.mapper.lift_urdf_to_height(position)
                else:
                    self.mapper.urdf_to_lerobot(joint, position, metadata)
        # Activation commands the complete resource. Joints omitted by a goal
        # are latched from this fresh measured state, never filled with zero.
        for joint in controller.joints:
            if joint not in self.latest_positions:
                raise ValueError(f"fresh measured state lacks {joint}")
            if resource == "lift":
                self.mapper.lift_urdf_to_height(self.latest_positions[joint])
            else:
                self.mapper.urdf_to_lerobot(
                    joint, self.latest_positions[joint], metadata
                )

    def on_trajectory_goal(self, goal, resource: str) -> GoalResponse:
        if not self.composer.enabled:
            self.get_logger().warning(f"Rejected {resource} goal: commands are disabled")
            return GoalResponse.REJECT
        if not self.observation_fresh():
            self.get_logger().warning(f"Rejected {resource} goal: observation is stale")
            return GoalResponse.REJECT
        if self.require_model_match and not self.model_matches():
            self.get_logger().warning(f"Rejected {resource} goal: robot model mismatch")
            return GoalResponse.REJECT
        if resource == "lift" and not self.lift_ready_for_commands():
            self.get_logger().warning(
                "Rejected lift goal: Host lift is not homed with torque enabled"
            )
            return GoalResponse.REJECT
        try:
            self.validate_trajectory_for_host(resource, goal)
        except (KeyError, TypeError, ValueError) as error:
            self.get_logger().warning(f"Rejected {resource} goal: {error}")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def on_gripper_goal(self, goal, resource: str) -> GoalResponse:
        if not self.composer.enabled:
            self.get_logger().warning(f"Rejected {resource} goal: commands are disabled")
            return GoalResponse.REJECT
        if not self.observation_fresh():
            self.get_logger().warning(f"Rejected {resource} goal: observation is stale")
            return GoalResponse.REJECT
        if self.require_model_match and not self.model_matches():
            self.get_logger().warning(f"Rejected {resource} goal: robot model mismatch")
            return GoalResponse.REJECT
        position = float(goal.command.position)
        max_effort = float(goal.command.max_effort)
        if not math.isfinite(position):
            self.get_logger().warning(f"Rejected {resource} goal: position is not finite")
            return GoalResponse.REJECT
        if not math.isfinite(max_effort) or max_effort < 0.0:
            self.get_logger().warning(
                f"Rejected {resource} goal: max_effort must be finite and non-negative"
            )
            return GoalResponse.REJECT
        if max_effort > 0.0:
            self.get_logger().warning(
                f"Rejected {resource} goal: per-goal max_effort is unsupported; "
                "the verified Host owns gripper current limiting"
            )
            return GoalResponse.REJECT
        joint = self.composer.resources[resource].joints[0]
        try:
            if joint not in self.latest_positions:
                raise ValueError(f"fresh measured state lacks {joint}")
            self.mapper.urdf_to_lerobot(
                joint, position, self.robot_metadata or {}
            )
        except (KeyError, TypeError, ValueError) as error:
            self.get_logger().warning(f"Rejected {resource} goal: {error}")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    @staticmethod
    def on_trajectory_cancel(_goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def trajectory_feedback(self, resource: str) -> FollowJointTrajectory.Feedback:
        controller = self.composer.resources[resource]
        feedback = FollowJointTrajectory.Feedback()
        feedback.header.stamp = self.get_clock().now().to_msg()
        feedback.joint_names = list(controller.joints)
        desired = controller.desired or self.latest_positions
        feedback.desired = JointTrajectoryPoint(
            positions=[float(desired[joint]) for joint in controller.joints]
        )
        feedback.actual = JointTrajectoryPoint(
            positions=[
                float(self.latest_positions[joint]) for joint in controller.joints
            ]
        )
        feedback.error = JointTrajectoryPoint(
            positions=[
                feedback.desired.positions[index] - feedback.actual.positions[index]
                for index in range(len(controller.joints))
            ]
        )
        return feedback

    def execute_trajectory(self, goal_handle, resource: str):
        result = FollowJointTrajectory.Result()
        try:
            samples = self.trajectory_samples(goal_handle.request)
            if samples and resource != "lift":
                preview = {}
                for joint, position in samples[0].positions.items():
                    key, value = self.mapper.urdf_to_lerobot(
                        joint, position, self.robot_metadata or {}
                    )
                    preview[key] = round(value, 2)
                self.get_logger().info(
                    f"[{resource}] trajectory accepted; first sample maps to {preview}"
                )
            goal_id = self.composer.start_trajectory(
                resource,
                goal_handle.request.trajectory.joint_names,
                samples,
                self.latest_positions,
                self.observation_fresh(),
            )
        except (KeyError, TypeError, ValueError) as error:
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = str(error)
            self.get_logger().warning(
                f"[{resource}] trajectory aborted at start: {error}"
            )
            return result

        controller = self.composer.resources[resource]
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.composer.cancel_trajectory(
                    resource,
                    goal_id,
                    self.latest_positions,
                    self.observation_fresh(),
                )
            event = controller.terminal(goal_id)
            if event is not None:
                result.error_string = event.message
                self.get_logger().info(
                    f"[{resource}] trajectory terminal: {event.state.name} "
                    f"{event.message or ''}"
                )
                if event.state is TerminalState.SUCCEEDED:
                    goal_handle.succeed()
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                elif event.state is TerminalState.CANCELED:
                    goal_handle.canceled()
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                else:
                    goal_handle.abort()
                    if event.state is TerminalState.GOAL_TOLERANCE:
                        result.error_code = (
                            FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
                        )
                    elif event.state in (TerminalState.ABORTED, TerminalState.STALE):
                        result.error_code = (
                            FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                        )
                    else:
                        result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                return result
            if self.observation_fresh():
                goal_handle.publish_feedback(self.trajectory_feedback(resource))
            time.sleep(0.02)

        goal_handle.abort()
        result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
        result.error_string = "ROS shutdown"
        return result

    def gripper_feedback(self, resource: str, reached_goal: bool = False):
        joint = self.composer.resources[resource].joints[0]
        feedback = GripperCommand.Feedback()
        feedback.position = float(self.latest_positions[joint])
        feedback.effort = 0.0
        feedback.stalled = False
        feedback.reached_goal = reached_goal
        return feedback

    def execute_gripper_command(self, goal_handle, resource: str):
        result = GripperCommand.Result()
        controller = self.composer.resources[resource]
        joint = controller.joints[0]
        target = float(goal_handle.request.command.position)
        try:
            goal_id = self.composer.start_trajectory(
                resource,
                [joint],
                [
                    TrajectorySample(
                        self.gripper_command_duration,
                        {joint: target},
                    )
                ],
                self.latest_positions,
                self.observation_fresh(),
            )
        except (KeyError, TypeError, ValueError) as error:
            self.get_logger().warning(f"Failed to start {resource} goal: {error}")
            goal_handle.abort()
            result.position = float(self.latest_positions.get(joint, 0.0))
            result.effort = 0.0
            result.stalled = False
            result.reached_goal = False
            return result

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.composer.cancel_trajectory(
                    resource,
                    goal_id,
                    self.latest_positions,
                    self.observation_fresh(),
                )
            event = controller.terminal(goal_id)
            if event is not None:
                reached = event.state is TerminalState.SUCCEEDED
                result.position = float(self.latest_positions.get(joint, 0.0))
                result.effort = 0.0
                result.stalled = False
                result.reached_goal = reached
                if reached:
                    goal_handle.succeed()
                elif event.state is TerminalState.CANCELED:
                    goal_handle.canceled()
                else:
                    self.get_logger().warning(f"{resource} command failed: {event.message}")
                    goal_handle.abort()
                return result
            if self.observation_fresh():
                goal_handle.publish_feedback(self.gripper_feedback(resource))
            time.sleep(0.02)

        goal_handle.abort()
        result.position = float(self.latest_positions.get(joint, 0.0))
        result.effort = 0.0
        result.stalled = False
        result.reached_goal = False
        return result

    def publish_diagnostics(self) -> None:
        age = (
            math.inf
            if self.last_observation_monotonic is None
            else time.monotonic() - self.last_observation_monotonic
        )
        status = DiagnosticStatus()
        status.name = "AlohaMini LeRobot Host bridge"
        status.hardware_id = str(self.get_parameter("host").value)
        if not self.observation_fresh():
            status.level = DiagnosticStatus.ERROR
            status.message = "Host observation stale or unavailable"
        elif self.require_model_match and not self.model_matches():
            status.level = DiagnosticStatus.ERROR
            status.message = "Host robot model mismatch"
        elif not self.lift_ready_for_commands():
            status.level = DiagnosticStatus.WARN
            status.message = "Lift is not command-ready"
        elif self.composer.enabled:
            status.level = DiagnosticStatus.WARN
            status.message = "ROS command channel enabled"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "State bridge healthy; command stream disabled"
        model = (
            self.robot_metadata.get("robot_model", "unknown")
            if self.robot_metadata
            else "unknown"
        )
        status.values = [
            KeyValue(key="observation_age_sec", value=f"{age:.3f}"),
            KeyValue(key="host_robot_model", value=str(model)),
            KeyValue(
                key="lift_command_ready",
                value=str(self.lift_ready_for_commands()).lower(),
            ),
            KeyValue(
                key="command_enabled", value=str(self.composer.enabled).lower()
            ),
            KeyValue(
                key="command_stream_started",
                value=str(self.composer.ever_commanded).lower(),
            ),
            *[
                KeyValue(
                    key=f"resource_{name}_active",
                    value=str(resource.active).lower(),
                )
                for name, resource in self.composer.resources.items()
            ],
            KeyValue(key="observation_count", value=str(self.observation_count)),
            KeyValue(key="invalid_observations", value=str(self.invalid_observations)),
            KeyValue(key="last_observation_error", value=self.last_observation_error),
            KeyValue(key="command_count", value=str(self.command_count)),
            KeyValue(key="pending_requests", value=str(len(self.transport.pending))),
            KeyValue(key="expired_requests", value=str(self.request_expirations)),
            KeyValue(key="malformed_responses", value=str(self.malformed_responses)),
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.diagnostics_pub.publish(array)

    def destroy_node(self) -> None:
        stop_action = self.composer.disable_action(
            self.latest_positions,
            self.robot_metadata or {},
            self.observation_fresh(),
        )
        self.composer.disable()
        if stop_action is not None:
            self.transport.send_action(stop_action)
        for server in self.action_servers:
            server.destroy()
        self.transport.close()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AlohaMiniLeRobotBridge()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
