from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from alohamini_lerobot_bridge.bridge_node import AlohaMiniLeRobotBridge
from alohamini_lerobot_bridge.control import (
    LEFT_ARM_JOINTS,
    TerminalState,
    TrajectorySample,
)
from alohamini_lerobot_bridge.protocol import ZmqHostTransport
from alohamini_lerobot_bridge.safety import HostSafety
from builtin_interfaces.msg import Time
from test_control import _composer, _mapper, _measured, _metadata


def _status(**updates):
    return {
        "version": 1,
        "host_session_id": "host",
        "control_owner": "ros",
        "control_epoch": 2,
        "joint_holds": {},
        "gripper_holds": {},
        "watchdog_active": False,
        "joint_hold_events": 0,
        "watchdog_events": 0,
        **updates,
    }


@pytest.mark.parametrize(
    "change",
    [
        {"joint_holds": {"arm_left_elbow_flex": 1.0}},
        {"joint_hold_events": 1},
        {"watchdog_events": 1},
        {"host_session_id": "restarted"},
        {"control_owner": "pc"},
    ],
)
def test_protection_or_owner_change_invalidates_ros_epoch(change):
    safety = HostSafety("ros")
    assert safety.update(_status()) is None
    assert safety.update(_status(**change))


def test_gripper_contact_is_not_a_joint_fault():
    safety = HostSafety("ros")
    assert safety.update(_status(gripper_holds={"arm_left_gripper": 0.3})) is None


def test_idle_watchdog_can_be_acknowledged_by_explicit_reenable():
    safety = HostSafety("ros")
    status = _status(watchdog_active=True, watchdog_events=1, control_owner=None)
    assert safety.update(status)
    assert safety.blocked(status) is None
    assert safety.update(status) is None


def test_legacy_host_is_read_only_until_upgraded():
    assert "Update the Host" in HostSafety("ros").blocked({})


def test_ros_transport_does_not_send_to_another_owner():
    transport = object.__new__(ZmqHostTransport)
    transport.client_id = "ros"
    transport.safety_status = _status(control_owner="pc")
    transport.command = Mock()
    assert transport.send_action({"x.vel": 0.0}) is False
    transport.command.send_string.assert_not_called()


def test_ros_transport_sends_identified_session_bound_commands():
    import json

    transport = object.__new__(ZmqHostTransport)
    transport.client_id = "ros"
    transport.command_sequence = 0
    transport.safety_status = _status()
    transport.command = Mock()
    action = {"x.vel": 0.1}
    assert transport.send_action(action)
    payload = json.loads(transport.command.send_string.call_args.args[0])
    assert payload["_command"] == {
        "client_id": "ros",
        "sequence": 1,
        "host_session_id": "host",
        "control_epoch": 2,
    }
    assert action == {"x.vel": 0.1}


@pytest.mark.parametrize("resource", ["left_arm", "left_gripper"])
def test_accepted_goal_cannot_start_after_disable_and_reenable(resource):
    composer = _composer()
    composer.enable()
    request = object()
    assert composer.accept_goal(request, composer.epoch)
    composer.disable("Host protection")
    composer.enable()
    joints = composer.resources[resource].joints
    with pytest.raises(ValueError, match="expired command epoch"):
        composer.start_trajectory(
            resource,
            joints,
            [TrajectorySample(1, dict.fromkeys(joints, 0.1))],
            _measured(),
            True,
            accepted_request=request,
        )
    assert composer.resources[resource].active_goal_id is None
    assert composer.accept_goal(request, composer.epoch)
    assert composer.start_trajectory(
        resource,
        joints,
        [TrajectorySample(1, dict.fromkeys(joints, 0.1))],
        _measured(),
        True,
        accepted_request=request,
    )


def test_goal_validation_cannot_cross_enable_epoch():
    composer = _composer()
    composer.enable()
    epoch = composer.epoch
    composer.disable()
    composer.enable()
    assert not composer.accept_goal(object(), epoch)


@pytest.mark.parametrize("stamp_ns, expected", [(99, False), (150, True), (201, False)])
def test_jog_must_be_newer_than_enable_and_not_from_future(stamp_ns, expected):
    bridge = SimpleNamespace(
        command_enabled_at_ns=100,
        composer=SimpleNamespace(command_timeout=0.1),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=200)),
    )
    jog = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=stamp_ns))
    )
    assert AlohaMiniLeRobotBridge.jog_is_current(bridge, jog) is expected


def test_stale_feedback_invalidates_accepted_but_unstarted_goal():
    composer = _composer()
    composer.enable()
    request = object()
    assert composer.accept_goal(request, composer.epoch)
    composer.compose(_measured(), _metadata(), False)
    with pytest.raises(ValueError, match="expired command epoch"):
        composer.start_trajectory(
            "left_arm",
            LEFT_ARM_JOINTS,
            [TrajectorySample(1, dict.fromkeys(LEFT_ARM_JOINTS, 0.1))],
            _measured(),
            True,
            accepted_request=request,
        )


@pytest.mark.parametrize("gripper", [False, True])
def test_bridge_callbacks_abort_delayed_goal_after_reenable(gripper):
    from rclpy.action import GoalResponse

    composer = _composer()
    composer.enable()
    resource = "left_gripper" if gripper else "left_arm"
    joints = composer.resources[resource].joints
    request = SimpleNamespace(
        command=SimpleNamespace(position=0.1, max_effort=0.0),
        trajectory=SimpleNamespace(joint_names=joints),
    )
    bridge = SimpleNamespace(
        composer=composer,
        latest_positions=_measured(),
        robot_metadata=_metadata(),
        require_model_match=False,
        observation_fresh=lambda: True,
        get_logger=lambda: Mock(),
        mapper=_mapper(),
        gripper_command_duration=0.2,
        validate_trajectory_for_host=lambda *_args: None,
        trajectory_samples=lambda _request: [
            TrajectorySample(1, dict.fromkeys(joints, 0.1))
        ],
    )
    accept = (
        AlohaMiniLeRobotBridge.on_gripper_goal
        if gripper
        else AlohaMiniLeRobotBridge.on_trajectory_goal
    )
    execute = (
        AlohaMiniLeRobotBridge.execute_gripper_command
        if gripper
        else AlohaMiniLeRobotBridge.execute_trajectory
    )
    assert accept(bridge, request, resource) == GoalResponse.ACCEPT
    composer.disable("Host protection")
    composer.enable()
    handle = Mock(request=request)
    execute(bridge, handle, resource)
    handle.abort.assert_called_once()
    assert composer.resources[resource].active_goal_id is None


def test_actual_bridge_protection_aborts_trajectory_before_tracking_tolerance():
    mapper = _mapper()
    metadata = {**_metadata(), "schema_version": 1, "robot_model": "alohamini2pro"}
    positions = _measured()
    composer = _composer()
    composer.enable()
    goal_id = composer.start_trajectory(
        "left_arm",
        LEFT_ARM_JOINTS,
        (TrajectorySample(1.0, dict.fromkeys(LEFT_ARM_JOINTS, 0.1)),),
        positions,
        True,
        now=0.0,
    )
    bridge = SimpleNamespace(
        mapper=mapper,
        composer=composer,
        transport=SimpleNamespace(safety_status={}, send_action=Mock()),
        host_safety=HostSafety("ros"),
        host_safety_fault="",
        expected_measured_joints=set(),
        observation_count=0,
        base_frame="base_link",
        get_parameter=lambda _name: SimpleNamespace(value=1.0),
        observation_stamp=lambda _obs: Time(),
        get_logger=lambda: Mock(),
        base_velocity_pub=Mock(),
        raw_pub=Mock(),
        joint_pub=Mock(),
        measured_joint_pub=Mock(),
    )
    observation = {
        "_robot_metadata": metadata,
        "x.vel": 0.0,
        "y.vel": 0.0,
        "theta.vel": 0.0,
        **{key + ".pos": 0.0 for key in metadata["motors"]},
        "_safety": _status(joint_holds={"arm_left_elbow_flex": 0.0}),
    }
    AlohaMiniLeRobotBridge.handle_observation(bridge, observation)
    assert not composer.enabled
    event = composer.resources["left_arm"].terminals[goal_id]
    assert event.state == TerminalState.ABORTED
    assert "Host joint protection" in event.message
    assert composer.compose(positions, metadata, True, now=0.1) is None
