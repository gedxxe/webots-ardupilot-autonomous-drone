from __future__ import annotations

from collections.abc import Iterator
from typing import Any

from pymavlink import mavutil


class MavlinkClient:
    """Small MAVLink read client used by smoke tests.

    This class contains blocking reads by design. Do not call it directly from
    `GateAutonomyMission.update()`. A real runtime should read MAVLink in an
    adapter loop and pass the latest fused telemetry snapshot into the mission.
    """

    def __init__(self, connection: str) -> None:
        self.connection = connection
        self.master = mavutil.mavlink_connection(connection)

    def wait_heartbeat(self, timeout: float = 10.0) -> Any:
        """Block until a heartbeat arrives or the timeout expires."""

        heartbeat = self.master.wait_heartbeat(timeout=timeout)
        if heartbeat is None:
            raise TimeoutError(f"No MAVLink heartbeat received from {self.connection}")
        return heartbeat

    def iter_messages(self, count: int | None = None, timeout: float = 1.0) -> Iterator[Any]:
        """Yield MAVLink messages using timeout-bounded blocking receives."""

        received = 0
        while count is None or received < count:
            message = self.master.recv_match(blocking=True, timeout=timeout)
            if message is None:
                continue
            received += 1
            yield message
