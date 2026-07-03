from types import SimpleNamespace

from drone_autonomy.mavlink.telemetry import MavlinkTelemetryAdapter


def test_telemetry_reports_stale_heartbeat() -> None:
    adapter = MavlinkTelemetryAdapter()
    adapter.update_message(
        SimpleNamespace(
            get_type=lambda: "HEARTBEAT",
            base_mode=0,
            mode_name="GUIDED",
        ),
        observed_at_s=1.0,
    )
    adapter.update_message(
        SimpleNamespace(get_type=lambda: "LOCAL_POSITION_NED", x=0.0, y=0.0, z=-1.0),
        observed_at_s=1.0,
    )

    reason = adapter.stale_reason(
        5.0,
        heartbeat_stale_s=3.0,
        local_position_stale_s=1.0,
    )

    assert reason is not None
    assert "heartbeat stale" in reason


def test_telemetry_reports_stale_local_position() -> None:
    adapter = MavlinkTelemetryAdapter()
    adapter.update_message(
        SimpleNamespace(
            get_type=lambda: "HEARTBEAT",
            base_mode=0,
            mode_name="GUIDED",
        ),
        observed_at_s=4.5,
    )
    adapter.update_message(
        SimpleNamespace(get_type=lambda: "LOCAL_POSITION_NED", x=0.0, y=0.0, z=-1.0),
        observed_at_s=1.0,
    )

    reason = adapter.stale_reason(
        5.0,
        heartbeat_stale_s=3.0,
        local_position_stale_s=1.0,
    )

    assert reason is not None
    assert "LOCAL_POSITION_NED stale" in reason
