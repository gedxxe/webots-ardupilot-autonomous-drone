from __future__ import annotations

from dataclasses import dataclass
from math import hypot
from typing import Any

from pymavlink import mavutil

from drone_autonomy.autonomy.mission import MissionTelemetry
from drone_autonomy.perception.detections import GateDetection


@dataclass(frozen=True)
class CourseFrame:
    """Projection from ArduPilot LOCAL_POSITION_NED into course-forward meters.

    Webots worlds should align the course with this vector or override it in the
    runtime config. Defaults to local NED x/North as the forward direction.
    """

    forward_x: float = 1.0
    forward_y: float = 0.0

    def __post_init__(self) -> None:
        norm = hypot(self.forward_x, self.forward_y)
        if norm <= 0.0:
            raise ValueError("course forward vector must be non-zero")

    @property
    def unit_x(self) -> float:
        norm = hypot(self.forward_x, self.forward_y)
        return self.forward_x / norm

    @property
    def unit_y(self) -> float:
        norm = hypot(self.forward_x, self.forward_y)
        return self.forward_y / norm

    def project_forward(
        self,
        origin_x_m: float,
        origin_y_m: float,
        x_m: float,
        y_m: float,
    ) -> float:
        return ((x_m - origin_x_m) * self.unit_x) + (
            (y_m - origin_y_m) * self.unit_y
        )


class MavlinkTelemetryAdapter:
    """Convert MAVLink messages into `MissionTelemetry` snapshots.

    This adapter consumes fused ArduPilot telemetry such as LOCAL_POSITION_NED.
    It does not fuse raw GPS, rangefinder, or optical-flow samples locally.
    """

    def __init__(self, course_frame: CourseFrame | None = None) -> None:
        self.course_frame = course_frame or CourseFrame()
        self.mode = "UNKNOWN"
        self.armed = False
        self.landed = False
        self.local_x_m: float | None = None
        self.local_y_m: float | None = None
        self.altitude_m: float | None = None
        self.origin_x_m: float | None = None
        self.origin_y_m: float | None = None

    def update_message(self, message: Any) -> None:
        message_type = self._message_type(message)
        if message_type == "HEARTBEAT":
            self._update_heartbeat(message)
        elif message_type == "LOCAL_POSITION_NED":
            self._update_local_position(message)
        elif message_type == "EXTENDED_SYS_STATE":
            self._update_extended_state(message)

    def snapshot(
        self,
        now_s: float,
        gate_detection: GateDetection | None = None,
    ) -> MissionTelemetry | None:
        if (
            self.local_x_m is None
            or self.local_y_m is None
            or self.altitude_m is None
            or self.origin_x_m is None
            or self.origin_y_m is None
        ):
            return None

        forward_position_m = self.course_frame.project_forward(
            self.origin_x_m,
            self.origin_y_m,
            self.local_x_m,
            self.local_y_m,
        )
        return MissionTelemetry(
            now_s=now_s,
            altitude_m=self.altitude_m,
            forward_position_m=forward_position_m,
            mode=self.mode,
            armed=self.armed,
            landed=self.landed,
            gate_detection=gate_detection,
        )

    def _update_heartbeat(self, message: Any) -> None:
        base_mode = int(getattr(message, "base_mode", 0) or 0)
        self.armed = bool(base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        self.mode = self._mode_name(message)

    def _update_local_position(self, message: Any) -> None:
        self.local_x_m = float(message.x)
        self.local_y_m = float(message.y)
        self.altitude_m = -float(message.z)
        if self.origin_x_m is None or self.origin_y_m is None:
            self.origin_x_m = self.local_x_m
            self.origin_y_m = self.local_y_m

    def _update_extended_state(self, message: Any) -> None:
        landed_state = int(getattr(message, "landed_state", 0) or 0)
        self.landed = landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND

    def _mode_name(self, message: Any) -> str:
        explicit_mode = getattr(message, "mode_name", None)
        if explicit_mode:
            return str(explicit_mode)
        try:
            return str(mavutil.mode_string_v10(message))
        except Exception:
            return "UNKNOWN"

    def _message_type(self, message: Any) -> str:
        if hasattr(message, "get_type"):
            return str(message.get_type())
        return str(getattr(message, "message_type", "UNKNOWN"))
