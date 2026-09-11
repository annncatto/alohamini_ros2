"""Host protection events that invalidate a ROS execution epoch."""


class HostSafety:
    def __init__(self, client_id: str) -> None:
        self.client_id = client_id
        self._session: str | None = None
        self._events: tuple[int, int] = (0, 0)

    def blocked(self, status: dict) -> str | None:
        if (
            status.get("version") != 1
            or "control_owner" not in status
            or type(status.get("control_epoch")) is not int
        ):
            return "Update the Host for protection and command ownership support"
        if not isinstance(status.get("host_session_id"), str) or not isinstance(
            status.get("joint_holds"), dict
        ):
            return "Invalid Host protection metadata"
        if any(
            type(status.get(key)) is not int
            for key in ("joint_hold_events", "watchdog_events")
        ):
            return "Invalid Host protection counters"
        if status["control_owner"] not in (None, self.client_id):
            return "Another client owns robot control"
        if status.get("joint_holds"):
            return "Host joint protection: " + ", ".join(status["joint_holds"])
        return None

    def update(self, status: dict) -> str | None:
        reason = self.blocked(status)
        if status.get("version") != 1:
            return reason
        session = status.get("host_session_id")
        events = (status.get("joint_hold_events", 0), status.get("watchdog_events", 0))
        if self._session is not None and session != self._session:
            reason = "Host restarted; enable a new ROS command epoch"
        elif self._session is not None and events != self._events:
            reason = reason or "Host protection event; enable a new ROS command epoch"
        elif self._session is None and status.get("watchdog_active"):
            reason = reason or "Host command watchdog active"
        self._session = session
        self._events = events
        return reason
