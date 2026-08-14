from __future__ import annotations

import json
import math
import time
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool

from .protocol import (
    BodyVelocity,
    CommandGate,
    JointMapper,
    ZmqHostTransport,
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
        with (calibration_share / "config/hardware/hardware_joint_map_right.yaml").open(
            encoding="utf-8"
        ) as stream:
            self.mapper = JointMapper(yaml.safe_load(stream))

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
        self.command_gate = CommandGate()
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.base_radius = float(self.get_parameter("base_radius").value)
        for name in ("linear_x_scale", "linear_y_scale", "angular_z_scale"):
            value = float(self.get_parameter(name).value)
            if not math.isfinite(value) or value == 0.0:
                raise ValueError(f"{name} must be finite and non-zero")

        self.latest_observation: dict | None = None
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
        self.command_gate.accept(
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
            if self.command_gate.disable():
                self.transport.send_action(
                    {"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}
                )
            self.command_gate.enable()
            response.success = True
            response.message = (
                "ROS command channel enabled but unarmed; a new /cmd_vel is required"
            )
            return response
        if self.command_gate.disable():
            self.transport.send_action({"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0})
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
        command = self.command_gate.resolve(permitted, self.command_timeout)
        if command is None:
            return
        if self.transport.send_action(
            {
                "x.vel": command.x,
                "y.vel": command.y,
                "theta.vel": math.degrees(command.yaw),
            }
        ):
            self.command_count += 1

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
        elif self.command_gate.enabled:
            status.level = DiagnosticStatus.WARN
            status.message = (
                "ROS command channel enabled"
                if self.command_gate.stream_started
                else "ROS command channel enabled; waiting for a new /cmd_vel"
            )
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
                key="command_enabled", value=str(self.command_gate.enabled).lower()
            ),
            KeyValue(
                key="command_stream_started",
                value=str(self.command_gate.stream_started).lower(),
            ),
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
        if self.command_gate.disable():
            self.transport.send_action({"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0})
        self.transport.close()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AlohaMiniLeRobotBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
