import pytest

from alohamini_joycon_teleop.teleop_node import ArmControl, JoyConTeleop
from sensor_msgs.msg import JointState


def test_busy_arm_keeps_only_the_latest_goal():
    node = object.__new__(JoyConTeleop)
    arm = ArmControl(goal_busy=True)
    node.arms = {"left": arm}

    node.send_arm_goal("left", [1.0, 2.0])
    node.send_arm_goal("left", [3.0, 4.0])

    assert arm.queued_positions == [3.0, 4.0]


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
