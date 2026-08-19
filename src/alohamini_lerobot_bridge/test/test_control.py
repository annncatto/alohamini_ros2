import pytest

from alohamini_lerobot_bridge.control import (
    LEFT_ARM_JOINTS,
    CommandComposer,
    TerminalState,
    TrajectorySample,
)
from alohamini_lerobot_bridge.protocol import BodyVelocity, JointMapper


def _mapper():
    joints = {}
    for joint in (
        "shoulder_pan",
        "shoulder_lift",
        "elbow_flex",
        "wrist_flex",
        "wrist_yaw",
        "wrist_roll",
    ):
        joints[joint] = {
            "reference_tick": 2048,
            "reference_q_rad": 0.0,
            "sign": 1,
            "safe_q_min_rad": -2.0,
            "safe_q_max_rad": 2.0,
        }
    joints["gripper"] = {
        "reference_tick": 2048,
        "reference_q_rad": 0.32,
        "sign": -1,
        "urdf_open_rad": -1.8,
        "urdf_closed_rad": 0.32,
    }
    return JointMapper(
        {"ticks_per_revolution": 4096, "joints": joints},
        {
            "mechanism": {"physical_min_mm": 0.0, "physical_max_mm": 600.0},
            "urdf": {
                "q_at_physical_min_m": -0.3,
                "q_at_physical_max_m": 0.3,
            },
        },
    )


def _metadata():
    motors = {}
    for side in ("left", "right"):
        for joint in (
            "shoulder_pan",
            "shoulder_lift",
            "elbow_flex",
            "wrist_flex",
            "wrist_yaw",
            "wrist_roll",
            "gripper",
        ):
            motors[f"arm_{side}_{joint}"] = {
                "normalization": "degrees",
                "drive_mode": 0,
                "range_min": 0,
                "range_max": 4095,
            }
    return {"motors": motors}


def _composer():
    return CommandComposer(_mapper(), 0.5, 0.3, 0.03, 0.2)


def _measured():
    return {
        **{joint: 0.0 for joint in LEFT_ARM_JOINTS},
        **{joint.replace("left_", "right_", 1): 0.0 for joint in LEFT_ARM_JOINTS},
        "left_gripper": 0.32,
        "right_gripper": 0.32,
        "vertical_move": 0.0,
    }


def test_inactive_resources_emit_no_default_joint_targets():
    composer = _composer()
    composer.enable()
    composer.accept_base(BodyVelocity(x=0.1), now=1.0)

    action = composer.compose(_measured(), _metadata(), True, now=1.1)

    assert action == {"x.vel": 0.1, "y.vel": 0.0, "theta.vel": 0.0}
    assert not any(key.endswith(".pos") for key in action)
    assert "lift_axis.height_mm" not in action


def test_partial_arm_goal_latches_unspecified_joints_from_fresh_measurement():
    composer = _composer()
    composer.enable()
    measured = _measured()
    measured["left_wrist_roll"] = 0.42
    composer.start_trajectory(
        "left_arm",
        ["left_shoulder_pan"],
        [TrajectorySample(1.0, {"left_shoulder_pan": 0.2})],
        measured,
        True,
        now=1.0,
    )

    action = composer.compose(measured, _metadata(), True, now=1.1)

    arm_keys = {key for key in action if key.startswith("arm_left_")}
    assert len(arm_keys) == len(LEFT_ARM_JOINTS)
    roll_key, roll_value = composer.mapper.urdf_to_lerobot(
        "left_wrist_roll", 0.42, _metadata()
    )
    assert action[roll_key] == pytest.approx(roll_value)
    assert not any(key.startswith("arm_right_") for key in action)
    assert {key: action[key] for key in ("x.vel", "y.vel", "theta.vel")} == {
        "x.vel": 0.0,
        "y.vel": 0.0,
        "theta.vel": 0.0,
    }


def test_stale_state_sends_one_base_zero_then_yields_to_host_watchdog():
    composer = _composer()
    composer.enable()
    composer.start_trajectory(
        "lift",
        ["vertical_move"],
        [TrajectorySample(1.0, {"vertical_move": 0.1})],
        _measured(),
        True,
        now=1.0,
    )
    assert composer.compose(_measured(), _metadata(), True, now=1.1)

    first = composer.compose(_measured(), _metadata(), False, now=1.2)
    second = composer.compose(_measured(), _metadata(), False, now=1.3)

    assert first == {"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}
    assert second is None
    event = composer.resources["lift"].terminal(1)
    assert event is not None and event.state is TerminalState.STALE


def test_stale_state_invalidates_old_base_command_epoch():
    composer = _composer()
    composer.enable()
    composer.accept_base(BodyVelocity(x=0.1), now=1.0)
    assert composer.compose(_measured(), _metadata(), True, now=1.1)

    assert composer.compose(_measured(), _metadata(), False, now=1.2) == {
        "x.vel": 0.0,
        "y.vel": 0.0,
        "theta.vel": 0.0,
    }
    assert composer.compose(_measured(), _metadata(), True, now=1.3) is None
    assert composer.compose(_measured(), _metadata(), False, now=1.4) is None


def test_preempt_terminates_old_goal_and_new_goal_latches_current_state():
    composer = _composer()
    composer.enable()
    measured = _measured()
    first = composer.start_trajectory(
        "left_arm",
        ["left_shoulder_pan"],
        [TrajectorySample(1.0, {"left_shoulder_pan": 0.2})],
        measured,
        True,
        now=1.0,
    )
    measured["left_wrist_roll"] = 0.3
    second = composer.start_trajectory(
        "left_arm",
        ["left_shoulder_lift"],
        [TrajectorySample(1.0, {"left_shoulder_lift": 0.1})],
        measured,
        True,
        now=1.1,
    )

    assert second != first
    event = composer.resources["left_arm"].terminal(first)
    assert event is not None and event.state is TerminalState.PREEMPTED
    action = composer.compose(measured, _metadata(), True, now=1.2)
    key, expected = composer.mapper.urdf_to_lerobot(
        "left_wrist_roll", 0.3, _metadata()
    )
    assert action[key] == pytest.approx(expected)


def test_independently_active_resources_compose_without_inactive_resource_fields():
    composer = _composer()
    composer.enable()
    measured = _measured()
    composer.start_trajectory(
        "left_arm",
        ["left_shoulder_pan"],
        [TrajectorySample(1.0, {"left_shoulder_pan": 0.1})],
        measured,
        True,
        now=1.0,
    )
    composer.start_trajectory(
        "lift",
        ["vertical_move"],
        [TrajectorySample(1.0, {"vertical_move": 0.1})],
        measured,
        True,
        now=1.0,
    )

    action = composer.compose(measured, _metadata(), True, now=1.1)

    assert any(key.startswith("arm_left_") for key in action)
    assert "lift_axis.height_mm" in action
    assert not any(key.startswith("arm_right_") for key in action)
    assert {key: action[key] for key in ("x.vel", "y.vel", "theta.vel")} == {
        "x.vel": 0.0,
        "y.vel": 0.0,
        "theta.vel": 0.0,
    }


def test_left_gripper_is_independent_and_does_not_activate_arm_or_right_gripper():
    composer = _composer()
    composer.enable()
    measured = _measured()
    composer.start_trajectory(
        "left_gripper",
        ["left_gripper"],
        [TrajectorySample(3.0, {"left_gripper": -1.0})],
        measured,
        True,
        now=0.0,
    )

    action = composer.compose(measured, _metadata(), True, now=0.5)

    assert "arm_left_gripper.pos" in action
    assert not any(
        key.startswith("arm_left_") and key != "arm_left_gripper.pos"
        for key in action
    )
    assert not any(key.startswith("arm_right_") for key in action)


def test_stale_gripper_feedback_stops_generating_gripper_targets():
    composer = _composer()
    composer.enable()
    measured = _measured()
    goal_id = composer.start_trajectory(
        "right_gripper",
        ["right_gripper"],
        [TrajectorySample(3.0, {"right_gripper": -1.0})],
        measured,
        True,
        now=0.0,
    )
    assert "arm_right_gripper.pos" in composer.compose(
        measured, _metadata(), True, now=0.2
    )

    first = composer.compose(measured, _metadata(), False, now=0.3)
    second = composer.compose(measured, _metadata(), False, now=0.4)

    assert first == {"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}
    assert second is None
    event = composer.resources["right_gripper"].terminal(goal_id)
    assert event is not None and event.state is TerminalState.STALE


def test_tracking_error_aborts_and_holds_latest_measurement_not_old_target():
    composer = _composer()
    composer.enable()
    measured = _measured()
    goal_id = composer.start_trajectory(
        "left_arm",
        ["left_shoulder_pan"],
        [TrajectorySample(1.0, {"left_shoulder_pan": 1.0})],
        measured,
        True,
        now=1.0,
    )
    measured["left_shoulder_pan"] = -0.5

    action = composer.compose(measured, _metadata(), True, now=1.5)

    event = composer.resources["left_arm"].terminal(goal_id)
    assert event is not None and event.state is TerminalState.ABORTED
    key, expected = composer.mapper.urdf_to_lerobot(
        "left_shoulder_pan", -0.5, _metadata()
    )
    assert action[key] == pytest.approx(expected)


def test_lift_command_changes_while_feedback_is_fixed_then_path_aborts():
    composer = _composer()
    composer.enable()
    measured = _measured()
    measured["vertical_move"] = -0.3
    goal_id = composer.start_trajectory(
        "lift",
        ["vertical_move"],
        [TrajectorySample(5.0, {"vertical_move": -0.1})],
        measured,
        True,
        now=0.0,
    )

    first = composer.compose(measured, _metadata(), True, now=0.2)
    second = composer.compose(measured, _metadata(), True, now=0.5)
    aborted = composer.compose(measured, _metadata(), True, now=0.8)

    assert second["lift_axis.height_mm"] > first["lift_axis.height_mm"]
    event = composer.resources["lift"].terminal(goal_id)
    assert event is not None and event.state is TerminalState.ABORTED
    assert aborted["lift_axis.height_mm"] == pytest.approx(0.0)


def test_lift_small_goal_requires_real_arrival_before_success():
    composer = _composer()
    composer.enable()
    measured = _measured()
    measured["vertical_move"] = -0.3
    goal_id = composer.start_trajectory(
        "lift",
        ["vertical_move"],
        [TrajectorySample(5.0, {"vertical_move": -0.295})],
        measured,
        True,
        now=0.0,
    )

    at_nominal_end = composer.compose(measured, _metadata(), True, now=5.0)

    assert at_nominal_end["lift_axis.height_mm"] == pytest.approx(5.0)
    assert composer.resources["lift"].terminal(goal_id) is None

    after_grace = composer.compose(measured, _metadata(), True, now=6.01)
    event = composer.resources["lift"].terminal(goal_id)
    assert event is not None and event.state is TerminalState.GOAL_TOLERANCE
    assert after_grace["lift_axis.height_mm"] == pytest.approx(0.0)


def test_lift_goal_succeeds_only_with_feedback_inside_goal_tolerance():
    composer = _composer()
    composer.enable()
    measured = _measured()
    measured["vertical_move"] = -0.3
    goal_id = composer.start_trajectory(
        "lift",
        ["vertical_move"],
        [TrajectorySample(5.0, {"vertical_move": -0.295})],
        measured,
        True,
        now=0.0,
    )
    measured["vertical_move"] = -0.296

    composer.compose(measured, _metadata(), True, now=5.0)

    event = composer.resources["lift"].terminal(goal_id)
    assert event is not None and event.state is TerminalState.SUCCEEDED


def test_cancel_holds_only_when_measurement_is_fresh():
    composer = _composer()
    composer.enable()
    measured = _measured()
    goal_id = composer.start_trajectory(
        "lift",
        ["vertical_move"],
        [TrajectorySample(1.0, {"vertical_move": 0.1})],
        measured,
        True,
        now=1.0,
    )
    measured["vertical_move"] = 0.02
    assert composer.cancel_trajectory("lift", goal_id, measured, True, now=1.1)
    assert composer.compose(measured, _metadata(), True, now=1.15) == {
        "lift_axis.height_mm": pytest.approx(320.0),
        "x.vel": 0.0,
        "y.vel": 0.0,
        "theta.vel": 0.0,
    }

    goal_id = composer.start_trajectory(
        "lift",
        ["vertical_move"],
        [TrajectorySample(1.0, {"vertical_move": 0.1})],
        measured,
        True,
        now=2.0,
    )
    assert composer.cancel_trajectory("lift", goal_id, measured, False, now=2.1)
    assert composer.compose(measured, _metadata(), True, now=2.15) is None


def test_disable_stops_active_lift_without_stale_position_hold():
    composer = _composer()
    composer.enable()
    measured = _measured()
    composer.start_trajectory(
        "lift",
        ["vertical_move"],
        [TrajectorySample(1.0, {"vertical_move": 0.1})],
        measured,
        True,
        now=1.0,
    )
    assert composer.compose(measured, _metadata(), True, now=1.1)

    fresh_stop = composer.disable_action(measured, _metadata(), True)
    stale_stop = composer.disable_action(measured, _metadata(), False)

    assert fresh_stop == {
        "x.vel": 0.0,
        "y.vel": 0.0,
        "theta.vel": 0.0,
        "lift_axis.vel": 0.0,
    }
    assert stale_stop == {"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}


def test_lift_hold_persists_indefinitely_after_goal_success():
    composer = _composer()
    composer.enable()
    measured = _measured()
    measured["vertical_move"] = -0.3
    composer.start_trajectory(
        "lift",
        ["vertical_move"],
        [TrajectorySample(1.0, {"vertical_move": -0.15})],
        measured,
        True,
        now=0.0,
    )
    # The measured lift tracks the streamed command (30 Hz cadence).
    for tick in (index * 0.05 for index in range(1, 23)):
        action = composer.compose(measured, _metadata(), True, now=tick)
        measured["vertical_move"] = composer.mapper.lift_height_to_urdf(
            action["lift_axis.height_mm"]
        )
    # Long after the goal ended the velocity-mode lift must keep being
    # commanded at the held height so the Host holds the axis (self-lock).
    for tick in (2.0, 10.0, 3600.0):
        action = composer.compose(measured, _metadata(), True, now=tick)
        assert action is not None
        assert composer.mapper.lift_height_to_urdf(
            action["lift_axis.height_mm"]
        ) == pytest.approx(-0.15, abs=1e-6)


def test_lift_hold_is_preempted_by_a_new_goal():
    composer = _composer()
    composer.enable()
    measured = _measured()
    measured["vertical_move"] = -0.3
    composer.start_trajectory(
        "lift",
        ["vertical_move"],
        [TrajectorySample(0.2, {"vertical_move": -0.15})],
        measured,
        True,
        now=0.0,
    )
    for tick in (0.05, 0.1, 0.15, 0.2, 0.25):
        action = composer.compose(measured, _metadata(), True, now=tick)
        measured["vertical_move"] = composer.mapper.lift_height_to_urdf(
            action["lift_axis.height_mm"]
        )
    composer.start_trajectory(
        "lift",
        ["vertical_move"],
        [TrajectorySample(0.2, {"vertical_move": -0.2})],
        measured,
        True,
        now=1.0,
    )
    action = composer.compose(measured, _metadata(), True, now=1.05)
    assert action is not None
    assert composer.mapper.lift_height_to_urdf(
        action["lift_axis.height_mm"]
    ) < -0.15
