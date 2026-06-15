from drone_autonomy.control.altitude import AltitudeHoldConfig, AltitudeHoldController


def test_altitude_hold_climbs_when_below_target() -> None:
    controller = AltitudeHoldController(
        AltitudeHoldConfig(target_altitude_m=1.0, deadband_m=0.0, kp=1.0)
    )

    assert controller.body_vz_for_altitude(0.7) < 0.0


def test_altitude_hold_descends_when_above_target() -> None:
    controller = AltitudeHoldController(
        AltitudeHoldConfig(target_altitude_m=1.0, deadband_m=0.0, kp=1.0)
    )

    assert controller.body_vz_for_altitude(1.3) > 0.0


def test_altitude_hold_deadband_outputs_zero_near_target() -> None:
    controller = AltitudeHoldController(
        AltitudeHoldConfig(target_altitude_m=1.0, deadband_m=0.1, kp=1.0)
    )

    assert controller.body_vz_for_altitude(0.95) == 0.0
