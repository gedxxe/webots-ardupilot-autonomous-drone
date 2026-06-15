import pytest

mavutil = pytest.importorskip("pymavlink.mavutil")

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
