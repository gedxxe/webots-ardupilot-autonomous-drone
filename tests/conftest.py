"""Shared pytest setup for offline unit tests.

Runtime code depends on real pymavlink. Unit tests can still validate adapter
logic on machines where pymavlink is not installed by installing a tiny
constants-only stub before test modules import runtime code.
"""

from __future__ import annotations

import sys
import types


def pytest_configure() -> None:
    """Install a minimal pymavlink stub only when the dependency is absent."""

    try:
        __import__("pymavlink")
        return
    except ModuleNotFoundError:
        pass

    mavlink = types.SimpleNamespace(
        MAV_FRAME_BODY_NED=8,
        MAV_CMD_SET_MESSAGE_INTERVAL=511,
        MAVLINK_MSG_ID_LOCAL_POSITION_NED=32,
        MAVLINK_MSG_ID_EXTENDED_SYS_STATE=245,
        MAV_MODE_FLAG_SAFETY_ARMED=128,
        MAV_MODE_FLAG_CUSTOM_MODE_ENABLED=1,
        MAV_CMD_DO_SET_MODE=176,
        MAV_LANDED_STATE_ON_GROUND=1,
        MAV_CMD_COMPONENT_ARM_DISARM=400,
        MAV_CMD_NAV_TAKEOFF=22,
        MAV_CMD_NAV_LAND=21,
        MAV_RESULT_ACCEPTED=0,
        MAV_RESULT_TEMPORARILY_REJECTED=1,
        MAV_RESULT_DENIED=2,
        MAV_RESULT_UNSUPPORTED=3,
        MAV_RESULT_FAILED=4,
        MAV_RESULT_IN_PROGRESS=5,
    )
    mavutil = types.ModuleType("pymavlink.mavutil")
    mavutil.mavlink = mavlink
    pymavlink = types.ModuleType("pymavlink")
    pymavlink.mavutil = mavutil

    sys.modules["pymavlink"] = pymavlink
    sys.modules["pymavlink.mavutil"] = mavutil
