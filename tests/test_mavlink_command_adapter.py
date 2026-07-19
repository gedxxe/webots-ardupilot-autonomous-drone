from pymavlink import mavutil

from drone_autonomy.autonomy.commands import VehicleCommand
from drone_autonomy.mavlink.commands import (
    VELOCITY_AND_YAWRATE_TYPE_MASK,
    MavlinkCommandAdapter,
    MavlinkCommandAdapterConfig,
)


class FakeMav:
    def __init__(self) -> None:
        self.command_long_calls: list[tuple[object, ...]] = []
        self.position_target_calls: list[tuple[object, ...]] = []

    def command_long_send(self, *args: object) -> None:
        self.command_long_calls.append(args)

    def set_position_target_local_ned_send(self, *args: object) -> None:
        self.position_target_calls.append(args)


class FakeMaster:
    def __init__(self) -> None:
        self.target_system = 42
        self.target_component = 84
        self.mav = FakeMav()
        self.set_mode_calls: list[str] = []

    def set_mode(self, mode: str) -> None:
        self.set_mode_calls.append(mode)


def adapter_for(master: FakeMaster) -> MavlinkCommandAdapter:
    return MavlinkCommandAdapter(
        master,
        MavlinkCommandAdapterConfig(command_repeat_interval_s=0.0),
    )


class FakeCommandAck:
    def __init__(self, command: int, result: int) -> None:
        self.command = command
        self.result = result

    def get_type(self) -> str:
        return "COMMAND_ACK"


def test_body_velocity_uses_body_ned_velocity_and_yaw_rate_mask() -> None:
    master = FakeMaster()
    adapter = adapter_for(master)

    adapter.send(
        VehicleCommand.body_velocity(
            body_vx_m_s=1.0,
            body_vy_m_s=0.2,
            body_vz_m_s=-0.1,
            yaw_rate_rad_s=0.3,
        ),
        now_s=12.0,
    )

    call = master.mav.position_target_calls[0]
    assert call[1] == 42
    assert call[2] == 84
    assert call[3] == mavutil.mavlink.MAV_FRAME_BODY_NED
    assert call[4] == VELOCITY_AND_YAWRATE_TYPE_MASK
    assert call[8] == 1.0
    assert call[9] == 0.2
    assert call[10] == -0.1
    assert call[15] == 0.3


def test_set_mode_uses_master_mode_mapping_helper() -> None:
    master = FakeMaster()
    adapter = adapter_for(master)

    assert adapter.send(VehicleCommand.set_mode("GUIDED"), now_s=1.0) is True

    assert master.set_mode_calls == ["GUIDED"]


def test_arm_and_takeoff_send_command_long() -> None:
    master = FakeMaster()
    adapter = adapter_for(master)

    adapter.send(VehicleCommand.arm_vehicle(), now_s=1.0)
    adapter.send(VehicleCommand.takeoff(1.0), now_s=2.0)

    arm_call = master.mav.command_long_calls[0]
    takeoff_call = master.mav.command_long_calls[1]
    assert arm_call[2] == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
    assert arm_call[4] == 1.0
    assert takeoff_call[2] == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
    assert takeoff_call[10] == 1.0


def test_tracked_command_ack_acceptance_clears_pending() -> None:
    master = FakeMaster()
    adapter = adapter_for(master)

    adapter.send(VehicleCommand.arm_vehicle(), now_s=1.0)

    assert adapter.pending_ack_count == 1
    adapter.update_message(
        FakeCommandAck(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            mavutil.mavlink.MAV_RESULT_ACCEPTED,
        )
    )

    events = adapter.pop_ack_events()
    assert adapter.pending_ack_count == 0
    assert adapter.pop_ack_failures() == ()
    assert len(events) == 1
    assert events[0].label == "arm"
    assert "MAV_RESULT_ACCEPTED" in events[0].detail


def test_accepted_tracked_commands_are_not_sent_again() -> None:
    cases = (
        (
            VehicleCommand.arm_vehicle(),
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        ),
        (VehicleCommand.takeoff(1.0), mavutil.mavlink.MAV_CMD_NAV_TAKEOFF),
        (VehicleCommand.land(), mavutil.mavlink.MAV_CMD_NAV_LAND),
    )

    for command, command_id in cases:
        master = FakeMaster()
        adapter = adapter_for(master)

        assert adapter.send(command, now_s=1.0) is True
        adapter.update_message(
            FakeCommandAck(command_id, mavutil.mavlink.MAV_RESULT_ACCEPTED)
        )

        assert adapter.send(command, now_s=100.0) is False
        assert len(master.mav.command_long_calls) == 1
        assert adapter.pending_ack_count == 0
        assert adapter.pop_ack_failures() == ()


def test_new_intent_replaces_accepted_command_in_same_family() -> None:
    master = FakeMaster()
    adapter = adapter_for(master)

    assert adapter.send(VehicleCommand.arm_vehicle(), now_s=1.0) is True
    adapter.update_message(
        FakeCommandAck(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            mavutil.mavlink.MAV_RESULT_ACCEPTED,
        )
    )

    assert adapter.send(VehicleCommand.disarm_vehicle(), now_s=2.0) is True
    adapter.update_message(
        FakeCommandAck(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            mavutil.mavlink.MAV_RESULT_ACCEPTED,
        )
    )

    assert adapter.send(VehicleCommand.arm_vehicle(), now_s=3.0) is True
    assert len(master.mav.command_long_calls) == 3


def test_same_command_id_cannot_have_multiple_pending_intents() -> None:
    master = FakeMaster()
    adapter = adapter_for(master)

    assert adapter.send(VehicleCommand.takeoff(1.0), now_s=1.0) is True
    assert adapter.send(VehicleCommand.takeoff(1.5), now_s=2.0) is False

    assert adapter.pending_ack_count == 1
    assert len(master.mav.command_long_calls) == 1


def test_tracked_command_ack_rejection_becomes_failure() -> None:
    master = FakeMaster()
    adapter = adapter_for(master)

    adapter.send(VehicleCommand.takeoff(1.0), now_s=1.0)
    adapter.update_message(
        FakeCommandAck(
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            mavutil.mavlink.MAV_RESULT_DENIED,
        )
    )

    failures = adapter.pop_ack_failures()
    assert adapter.pending_ack_count == 0
    assert len(failures) == 1
    assert failures[0].label == "takeoff"
    assert "MAV_RESULT_DENIED" in failures[0].detail


def test_in_progress_command_ack_keeps_pending_command() -> None:
    master = FakeMaster()
    adapter = adapter_for(master)

    adapter.send(VehicleCommand.takeoff(1.0), now_s=1.0)
    adapter.update_message(
        FakeCommandAck(
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
        )
    )

    events = adapter.pop_ack_events()
    assert adapter.pending_ack_count == 1
    assert adapter.pop_ack_failures() == ()
    assert len(events) == 1
    assert events[0].label == "takeoff"
    assert "MAV_RESULT_IN_PROGRESS" in events[0].detail


def test_command_ack_timeout_retries_then_fails() -> None:
    master = FakeMaster()
    adapter = MavlinkCommandAdapter(
        master,
        MavlinkCommandAdapterConfig(
            command_repeat_interval_s=0.0,
            command_ack_timeout_s=1.0,
            command_ack_max_retries=1,
        ),
    )

    adapter.send(VehicleCommand.land(), now_s=0.0)
    adapter.update_ack_timeouts(now_s=1.1)

    retry_events = adapter.pop_ack_events()
    assert len(master.mav.command_long_calls) == 2
    assert adapter.pending_ack_count == 1
    assert adapter.pop_ack_failures() == ()
    assert len(retry_events) == 1
    assert "retry 1/1" in retry_events[0].detail

    adapter.update_ack_timeouts(now_s=2.2)

    failures = adapter.pop_ack_failures()
    assert adapter.pending_ack_count == 0
    assert len(failures) == 1
    assert failures[0].label == "land"
    assert "timeout after 2 send attempt" in failures[0].detail


def test_untracked_command_ack_is_not_fail_closed() -> None:
    master = FakeMaster()
    adapter = adapter_for(master)

    adapter.update_message(
        FakeCommandAck(
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            mavutil.mavlink.MAV_RESULT_DENIED,
        )
    )

    events = adapter.pop_ack_events()
    assert adapter.pop_ack_failures() == ()
    assert len(events) == 1
    assert events[0].severity == "warning"
    assert events[0].label.startswith("untracked command")
