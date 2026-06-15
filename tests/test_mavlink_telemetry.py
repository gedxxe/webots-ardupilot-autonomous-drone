import pytest

mavutil = pytest.importorskip("pymavlink.mavutil")

from drone_autonomy.mavlink.telemetry import CourseFrame, MavlinkTelemetryAdapter


class FakeMessage:
    def __init__(self, message_type: str, **fields: object) -> None:
        self._message_type = message_type
        for key, value in fields.items():
            setattr(self, key, value)

    def get_type(self) -> str:
        return self._message_type


def test_telemetry_adapter_builds_mission_snapshot_from_local_position() -> None:
    adapter = MavlinkTelemetryAdapter()
    adapter.update_message(
        FakeMessage(
            "HEARTBEAT",
            base_mode=mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED,
            mode_name="GUIDED",
        )
    )
    adapter.update_message(FakeMessage("LOCAL_POSITION_NED", x=10.0, y=5.0, z=-1.2))
    adapter.update_message(FakeMessage("LOCAL_POSITION_NED", x=12.5, y=5.0, z=-1.1))

    telemetry = adapter.snapshot(now_s=3.0)

    assert telemetry is not None
    assert telemetry.mode == "GUIDED"
    assert telemetry.armed is True
    assert telemetry.altitude_m == 1.1
    assert telemetry.forward_position_m == 2.5


def test_course_frame_can_project_forward_along_local_y() -> None:
    adapter = MavlinkTelemetryAdapter(CourseFrame(forward_x=0.0, forward_y=1.0))
    adapter.update_message(FakeMessage("LOCAL_POSITION_NED", x=0.0, y=2.0, z=-1.0))
    adapter.update_message(FakeMessage("LOCAL_POSITION_NED", x=4.0, y=5.0, z=-1.0))

    telemetry = adapter.snapshot(now_s=1.0)

    assert telemetry is not None
    assert telemetry.forward_position_m == 3.0


def test_landed_state_maps_to_mission_landed_flag() -> None:
    adapter = MavlinkTelemetryAdapter()
    adapter.update_message(FakeMessage("LOCAL_POSITION_NED", x=0.0, y=0.0, z=-0.1))
    adapter.update_message(
        FakeMessage(
            "EXTENDED_SYS_STATE",
            landed_state=mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
        )
    )

    telemetry = adapter.snapshot(now_s=1.0)

    assert telemetry is not None
    assert telemetry.landed is True
