from types import SimpleNamespace

from alohamini_lerobot_bridge.bridge_node import AlohaMiniLeRobotBridge


class _Stamp:
    pass


class _Now:
    def __init__(self, stamp):
        self._stamp = stamp

    def to_msg(self):
        return self._stamp


class _Clock:
    def __init__(self, stamp):
        self._stamp = stamp

    def now(self):
        return _Now(self._stamp)


def test_receipt_timestamp_uses_ros_clock_not_host_wall_time():
    expected = _Stamp()
    bridge = SimpleNamespace(
        state_timestamp_mode="receipt",
        last_state_clock_offset_ms=123.0,
        get_clock=lambda: _Clock(expected),
    )
    observation = {"_host_timing": {"state_sample_unix_ns": 1}}

    actual = AlohaMiniLeRobotBridge.observation_stamp(bridge, observation)

    assert actual is expected
    assert bridge.last_state_clock_offset_ms != bridge.last_state_clock_offset_ms
