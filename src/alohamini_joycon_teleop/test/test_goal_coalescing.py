from types import SimpleNamespace

import pytest
from action_msgs.msg import GoalStatus
from alohamini_joycon_teleop.teleop_node import ArmControl, JoyConTeleop
from sensor_msgs.msg import JointState


def test_busy_arm_keeps_only_the_latest_goal():
    node = object.__new__(JoyConTeleop)
    arm = ArmControl(goal_busy=True)
    node.arms = {"left": arm}

    node.send_arm_goal("left", [1.0, 2.0])
    node.send_arm_goal("left", [3.0, 4.0])

    assert arm.queued_positions == [3.0, 4.0]


class _ResultFuture:
    def __init__(self):
        self.callback = None

    def add_done_callback(self, callback):
        self.callback = callback


class _AcceptedHandle:
    accepted = True

    def __init__(self):
        self.result_future = _ResultFuture()

    def get_result_async(self):
        return self.result_future


def test_accepted_goal_immediately_streams_latest_queued_target():
    node = object.__new__(JoyConTeleop)
    arm = ArmControl(
        active=True,
        goal_busy=True,
        queued_positions=[3.0, 4.0],
    )
    node.arms = {"left": arm}
    node.get_parameter = lambda _name: SimpleNamespace(value=True)
    streamed = []
    node.send_arm_goal = lambda side, positions: streamed.append(
        (side, positions)
    )
    handle = _AcceptedHandle()

    JoyConTeleop.on_arm_goal_response(
        node, "left", SimpleNamespace(result=lambda: handle)
    )

    assert not arm.goal_busy
    assert arm.goal_handle is handle
    assert arm.queued_positions is None
    assert streamed == [("left", [3.0, 4.0])]
    assert handle.result_future.callback is not None


def test_expected_preemption_does_not_latch_arm_fault():
    node = object.__new__(JoyConTeleop)
    handle = object()
    arm = ArmControl(active=True, goal_busy=True, goal_handle=handle)
    node.arms = {"left": arm}
    response = SimpleNamespace(
        status=GoalStatus.STATUS_ABORTED,
        result=SimpleNamespace(error_code=-1, error_string=""),
    )

    JoyConTeleop.on_arm_result(
        node, "left", handle, SimpleNamespace(result=lambda: response)
    )

    assert not arm.fault_latched
    assert arm.active


class _ArmParameter:
    value = 0.03


class _ArmHarness:
    hardware_mode = True
    ik_period = 0.1

    def __init__(self):
        self.arms = {"left": ArmControl(cancel_after_accept=True)}

    def get_parameter(self, _name):
        return _ArmParameter()

    def request_fk(self, side, _sample):
        self.arms[side].pending_fk = True

    def toggle_gripper(self, _side):
        raise AssertionError("gripper should remain inactive")


def test_fresh_arm_engagement_clears_startup_cancel_latch():
    harness = _ArmHarness()
    sample = {
        "buttons": {
            "up": True,
            "down": False,
            "left": False,
            "right": False,
            "shoulder": False,
            "sl": False,
            "sr": False,
            "relatch": False,
            "trigger": False,
        },
        "orientation_rpy": [0.0, 0.0, 0.0],
    }

    JoyConTeleop.update_arm(harness, "left", sample, 1.0, 0.05)

    arm = harness.arms["left"]
    assert arm.active
    assert not arm.cancel_after_accept
    assert arm.pending_fk


def test_arm_tcp_step_uses_elapsed_ik_time_not_timer_tick():
    harness = _ArmHarness()
    arm = ArmControl(
        active=True,
        target_pose={
            "position": [0.0, 0.0, 0.0],
            "orientation": [0.0, 0.0, 0.0, 1.0],
        },
        last_ik_request=0.89,
    )
    harness.arms["left"] = arm
    requested = {}
    harness.request_ik = lambda side, pose, now, **kwargs: requested.update(
        side=side, pose=pose, now=now, kwargs=kwargs
    )
    sample = {
        "buttons": {
            "up": True,
            "down": False,
            "left": False,
            "right": False,
            "shoulder": False,
            "sl": False,
            "sr": False,
            "relatch": False,
            "trigger": False,
        },
        "orientation_rpy": [0.0, 0.0, 0.0],
    }

    JoyConTeleop.update_arm(harness, "left", sample, 1.0, 1.0 / 30.0)

    # 30 mm/s over the 100 ms since the previous IK request = 3 mm.
    assert requested["pose"]["position"][1] == pytest.approx(-0.003)


class _Parameter:
    value = 0.02


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _LiftHarness:
    hardware_mode = True
    lift_active = False
    deadzone = 0.25

    def __init__(self):
        self.lift_jog_pub = _Publisher()

    def get_parameter(self, _name):
        return _Parameter()


def test_lift_joint_jog_streams_while_pressed_and_stops_once():
    harness = _LiftHarness()
    up = {"stick": (2000.0, 4000.0)}
    center = {"stick": (2000.0, 2000.0)}

    JoyConTeleop.update_lift(harness, up, True, 0.0, 0.05)
    assert list(harness.lift_jog_pub.messages[-1].velocities) == [0.02]

    JoyConTeleop.update_lift(harness, center, True, 0.05, 0.05)
    assert list(harness.lift_jog_pub.messages[-1].velocities) == [0.0]

    count = len(harness.lift_jog_pub.messages)
    JoyConTeleop.update_lift(harness, center, True, 0.1, 0.05)
    assert len(harness.lift_jog_pub.messages) == count


def test_first_measured_lift_state_updates_removed_hold_state_safely():
    harness = object.__new__(JoyConTeleop)
    harness.hardware_mode = True
    harness.positions = {}
    harness.lift_active = False
    harness.lift_target = 0.0
    harness.measured_lift = 0.0
    harness.last_measured_state = None
    message = JointState()
    message.name = ["vertical_move"]
    message.position = [-0.12]

    JoyConTeleop.on_joint_state(harness, message)

    assert harness.measured_lift == -0.12
    assert harness.lift_target == -0.12


def test_gripper_trigger_is_edge_detected_and_debounced_on_both_sides():
    harness = object.__new__(JoyConTeleop)
    harness.arms = {"left": ArmControl(), "right": ArmControl()}
    parameters = {
        "button_release_grace_sec": 0.12,
        "gripper_button_debounce_sec": 0.30,
    }
    harness.get_parameter = lambda name: SimpleNamespace(value=parameters[name])
    toggles = []
    harness.toggle_gripper = lambda side: toggles.append(side)

    for side in ("left", "right"):
        JoyConTeleop.update_gripper_button(harness, side, True, 1.00)
        JoyConTeleop.update_gripper_button(harness, side, False, 1.05)
        # A short false report followed by true is contact/report bounce, not a
        # second physical press, even after the minimum press interval expires.
        JoyConTeleop.update_gripper_button(harness, side, True, 1.10)
        JoyConTeleop.update_gripper_button(harness, side, False, 1.35)
        JoyConTeleop.update_gripper_button(harness, side, False, 1.48)
        JoyConTeleop.update_gripper_button(harness, side, True, 1.50)

    assert toggles == ["left", "left", "right", "right"]


class _GripperClient:
    def __init__(self):
        self.goals = []

    def server_is_ready(self):
        return True

    def send_goal_async(self, goal):
        self.goals.append(goal)


def test_gripper_toggle_reverses_last_command_even_without_measured_motion():
    node = object.__new__(JoyConTeleop)
    node.hardware_mode = True
    node.positions = {"left_gripper": 0.32}
    node.arms = {"left": ArmControl()}
    client = _GripperClient()
    node.gripper_clients = {"left": client}

    JoyConTeleop.toggle_gripper(node, "left")
    # Keep measured feedback unchanged to model a stalled/loaded opening command.
    JoyConTeleop.toggle_gripper(node, "left")

    assert [goal.command.position for goal in client.goals] == pytest.approx(
        [-1.8030294104, 0.32]
    )


def test_command_buttons_ignore_short_false_reports_then_release():
    harness = object.__new__(JoyConTeleop)
    harness.input_timeout = 0.25
    harness.button_last_true = {"left": {}, "right": {}}
    harness.get_parameter = lambda _name: SimpleNamespace(value=0.12)
    buttons = {
        "shoulder": True,
        "sl": False,
        "sr": False,
        "up": False,
        "down": False,
        "left": False,
        "right": False,
    }
    harness.samples = {
        "left": SimpleNamespace(received_at=1.0, payload={"buttons": buttons})
    }

    assert JoyConTeleop.sample(harness, "left", 1.00)["buttons"]["shoulder"]
    buttons["shoulder"] = False
    assert JoyConTeleop.sample(harness, "left", 1.05)["buttons"]["shoulder"]
    assert not JoyConTeleop.sample(harness, "left", 1.13)["buttons"]["shoulder"]
