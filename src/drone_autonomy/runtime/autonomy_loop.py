from __future__ import annotations

from dataclasses import dataclass
from time import monotonic, sleep
from typing import Protocol

from pymavlink import mavutil

from drone_autonomy.autonomy.mission import (
    GateAutonomyMission,
    GateMissionConfig,
    MissionOutput,
    MissionPhase,
)
from drone_autonomy.control.visual_servo import (
    GateVisualServoController,
    VisualServoConfig,
)
from drone_autonomy.mavlink.commands import MavlinkCommandAdapter
from drone_autonomy.mavlink.telemetry import CourseFrame, MavlinkTelemetryAdapter
from drone_autonomy.perception.detections import GateDetection
from drone_autonomy.perception.synthetic import SyntheticGateProvider
from drone_autonomy.runtime.config import AutonomyRuntimeConfig


class GateDetectionProvider(Protocol):
    """Runtime-owned detector source interface.

    Providers may own I/O resources. The mission still receives only
    `GateDetection | None`, so camera/model details do not leak into autonomy.
    """

    def detect(self, now_s: float) -> GateDetection | None:
        """Return the latest gate observation or `None`."""

    def close(self) -> None:
        """Release provider resources."""


@dataclass(frozen=True)
class AutonomyRuntimeResult:
    """Final status returned after the blocking runtime exits."""

    completed: bool
    final_phase: MissionPhase
    last_output: MissionOutput | None


class AutonomyRuntime:
    """Blocking process loop that wires MAVLink, perception, and mission logic.

    Blocking belongs here, not inside `GateAutonomyMission`. This keeps the
    mission unit-testable and lets SITL/hardware adapters evolve independently.
    """

    def __init__(self, config: AutonomyRuntimeConfig) -> None:
        self.config = config
        if self.config.loop_hz <= 0.0:
            raise ValueError("loop_hz must be positive")
        if self.config.detector not in {"none", "synthetic", "webots-yolo"}:
            raise ValueError("detector must be 'none', 'synthetic', or 'webots-yolo'")

    def run(self) -> AutonomyRuntimeResult:
        """Run the complete SITL/hardware-facing autonomy process.

        High-level sequence:
        1. Connect to MAVLink and wait for heartbeat.
        2. Build adapters for telemetry, commands, and detector source.
        3. Repeatedly drain MAVLink, build a telemetry snapshot, run mission,
           and optionally send the returned command.
        """

        master = mavutil.mavlink_connection(
            self.config.connection,
            baud=self.config.mavlink_baud,
        )
        heartbeat = master.wait_heartbeat(timeout=self.config.heartbeat_timeout_s)
        if heartbeat is None:
            raise TimeoutError(f"No heartbeat from {self.config.connection}")

        command_mode = "send-commands" if self.config.send_commands else "dry-run"
        print(
            f"autonomy connection={self.config.connection} "
            f"baud={self.config.mavlink_baud} "
            f"detector={self.config.detector} "
            f"command_mode={command_mode} "
            f"loop_hz={self.config.loop_hz}"
        )

        mission = self._build_mission()
        command_adapter = MavlinkCommandAdapter(master)
        telemetry_adapter = MavlinkTelemetryAdapter(
            CourseFrame(
                forward_x=self.config.course_forward_x,
                forward_y=self.config.course_forward_y,
            )
        )
        telemetry_adapter.update_message(heartbeat)
        synthetic_gate = (
            SyntheticGateProvider() if self.config.detector == "synthetic" else None
        )
        webots_yolo_gate = self._build_webots_yolo_provider()

        try:
            command_adapter.request_default_telemetry(rate_hz=self.config.loop_hz)
        except Exception as exc:
            print(f"warning: telemetry interval request failed: {exc}")

        start_s = monotonic()
        next_status_s = start_s
        last_output: MissionOutput | None = None

        try:
            while monotonic() - start_s <= self.config.max_runtime_s:
                loop_started_s = monotonic()
                # Drain all queued MAVLink messages before building the snapshot so
                # the mission always sees the freshest available fused telemetry.
                self._drain_mavlink(master, telemetry_adapter)

                detection = None
                if synthetic_gate is not None:
                    # Synthetic perception is phase-aware and exists only to test
                    # mission/MAVLink wiring while bypassing real camera/model I/O.
                    detection = synthetic_gate.detect_for_phase(
                        loop_started_s,
                        mission.phase,
                        mission.gate_index,
                    )
                elif webots_yolo_gate is not None:
                    # Real perception still returns only the `GateDetection`
                    # contract. Camera/model details remain outside the mission.
                    webots_yolo_gate.update_context(
                        phase=mission.phase.value,
                        gate_index=mission.gate_index,
                    )
                    detection = webots_yolo_gate.detect(loop_started_s)

                telemetry = telemetry_adapter.snapshot(loop_started_s, detection)
                if telemetry is None:
                    # Mission must not run until local-position telemetry exists;
                    # otherwise altitude and forward distance would be fabricated.
                    if loop_started_s >= next_status_s:
                        print("waiting for LOCAL_POSITION_NED telemetry")
                        next_status_s = loop_started_s + self.config.status_interval_s
                    self._sleep_until_next_tick(loop_started_s)
                    continue

                last_output = mission.update(telemetry)
                if self.config.send_commands:
                    # This is the only point where mission output becomes a real
                    # MAVLink command. Without `--send-commands`, runtime is dry-run.
                    command_adapter.send(last_output.command, now_s=loop_started_s)

                if loop_started_s >= next_status_s:
                    sent = "sent" if self.config.send_commands else "dry-run"
                    servo_detail = ""
                    if last_output.servo is not None:
                        servo = last_output.servo
                        servo_detail = (
                            f" servo_err=({servo.error_x:+0.3f},"
                            f"{servo.error_y:+0.3f})"
                            f" area={servo.area_ratio:0.3f}"
                            f" aligned={servo.is_aligned}"
                            f" clearance={servo.clearance_ready}"
                            f" pass_ready={servo.pass_ready}"
                        )
                    print(
                        f"{sent} phase={last_output.phase.value} "
                        f"gate={last_output.gate_index + 1} "
                        f"cmd={last_output.command.kind.value} "
                        f"detail={last_output.detail}"
                        f"{servo_detail}"
                    )
                    next_status_s = loop_started_s + self.config.status_interval_s

                if last_output.phase == MissionPhase.COMPLETE:
                    return AutonomyRuntimeResult(True, mission.phase, last_output)
                if last_output.phase == MissionPhase.FAILSAFE:
                    return AutonomyRuntimeResult(False, mission.phase, last_output)

                self._sleep_until_next_tick(loop_started_s)
        finally:
            if webots_yolo_gate is not None:
                webots_yolo_gate.close()

        return AutonomyRuntimeResult(False, mission.phase, last_output)

    def _build_mission(self) -> GateAutonomyMission:
        """Build mission/control objects from process-level runtime config."""

        servo_config = VisualServoConfig(
            frame_width_px=self.config.visual_frame_width_px,
            frame_height_px=self.config.visual_frame_height_px,
            min_confidence=self.config.visual_min_confidence,
            filter_alpha=self.config.visual_filter_alpha,
            command_filter_alpha=self.config.visual_command_filter_alpha,
            center_deadband_x=self.config.visual_center_deadband_x,
            center_deadband_y=self.config.visual_center_deadband_y,
            aligned_error_x=self.config.visual_aligned_error_x,
            aligned_error_y=self.config.visual_aligned_error_y,
            pass_target_offset_x=self.config.visual_pass_target_offset_x,
            pass_target_offset_y=self.config.visual_pass_target_offset_y,
            pass_clearance_left_error=self.config.visual_pass_clearance_left_error,
            pass_clearance_right_error=self.config.visual_pass_clearance_right_error,
            pass_clearance_up_error=self.config.visual_pass_clearance_up_error,
            pass_clearance_down_error=self.config.visual_pass_clearance_down_error,
            max_error_for_forward=self.config.visual_max_error_for_forward,
            min_forward_speed_m_s=self.config.visual_min_forward_speed_m_s,
            max_forward_speed_m_s=self.config.visual_max_forward_speed_m_s,
            lateral_kp=self.config.visual_lateral_kp,
            vertical_kp=self.config.visual_vertical_kp,
            yaw_kp=self.config.visual_yaw_kp,
            max_lateral_speed_m_s=self.config.visual_max_lateral_speed_m_s,
            max_vertical_speed_m_s=self.config.visual_max_vertical_speed_m_s,
            max_yaw_rate_rad_s=self.config.visual_max_yaw_rate_rad_s,
        )
        mission_config = GateMissionConfig(
            max_detection_age_s=self.config.mission_max_detection_age_s,
            required_detection_ticks=self.config.mission_required_detection_ticks,
            center_dwell_s=self.config.mission_center_dwell_s,
            center_clearance_required_s=(
                self.config.mission_center_clearance_required_s
            ),
            center_lost_detection_grace_ticks=(
                self.config.mission_center_lost_detection_grace_ticks
            ),
            seek_yaw_rate_rad_s=self.config.mission_seek_yaw_rate_rad_s,
            gate_pass_distance_m=self.config.mission_gate_pass_distance_m,
            gate_pass_speed_m_s=self.config.mission_gate_pass_speed_m_s,
            next_gate_acquire_speed_m_s=(
                self.config.mission_next_gate_acquire_speed_m_s
            ),
            next_gate_acquire_min_clear_distance_m=(
                self.config.mission_next_gate_acquire_min_clear_distance_m
            ),
            next_gate_acquire_min_area_ratio=(
                self.config.mission_next_gate_acquire_min_area_ratio
            ),
            gate_ready_area_ratio=self.config.mission_gate_ready_area_ratio,
            next_gate_acquire_max_distance_m=(
                self.config.mission_next_gate_acquire_max_distance_m
            ),
            next_gate_acquire_timeout_s=(
                self.config.mission_next_gate_acquire_timeout_s
            ),
            brake_settle_s=self.config.mission_brake_settle_s,
            brake_ramp_s=self.config.mission_brake_ramp_s,
            brake_altitude_hold_enabled=(
                self.config.mission_brake_altitude_hold_enabled
            ),
            final_exit_distance_m=self.config.mission_final_exit_distance_m,
            final_exit_speed_m_s=self.config.mission_final_exit_speed_m_s,
        )
        return GateAutonomyMission(
            mission_config,
            GateVisualServoController(servo_config),
        )

    def _build_webots_yolo_provider(self) -> GateDetectionProvider | None:
        """Build the optional Webots+YOLO provider only when requested."""

        if self.config.detector != "webots-yolo":
            return None
        if not self.config.yolo_model_path:
            raise ValueError("--yolo-model is required when --detector webots-yolo")

        from drone_autonomy.perception.webots_camera import WebotsCameraConfig
        from drone_autonomy.perception.webots_yolo import (
            WebotsYoloConfig,
            WebotsYoloGateProvider,
        )
        from drone_autonomy.perception.target_selector import GateTargetSelectorConfig
        from drone_autonomy.perception.yolo import YoloGateConfig

        print(
            "webots-yolo camera="
            f"tcp://{self.config.webots_camera_host}:{self.config.webots_camera_port} "
            f"encoding={self.config.webots_camera_encoding} "
            f"model={self.config.yolo_model_path}"
        )
        print(
            "webots-yolo class_filter "
            f"names={_format_filter_values(self.config.yolo_gate_class_names)} "
            f"ids={_format_filter_values(self.config.yolo_gate_class_ids)}"
        )
        return WebotsYoloGateProvider(
            WebotsYoloConfig(
                camera=WebotsCameraConfig(
                    host=self.config.webots_camera_host,
                    port=self.config.webots_camera_port,
                    encoding=self.config.webots_camera_encoding,
                    idle_reconnect_s=self.config.webots_camera_idle_reconnect_s,
                ),
                yolo=YoloGateConfig(
                    model_path=self.config.yolo_model_path,
                    confidence=self.config.yolo_confidence,
                    image_size_px=self.config.yolo_image_size_px,
                    device=self.config.yolo_device,
                    gate_class_names=self.config.yolo_gate_class_names,
                    gate_class_ids=self.config.yolo_gate_class_ids,
                ),
                selector=GateTargetSelectorConfig(
                    min_seek_confidence=self.config.gate_selector_min_seek_confidence,
                    min_track_confidence=self.config.gate_selector_min_track_confidence,
                    min_area_ratio=self.config.gate_selector_min_area_ratio,
                    min_aspect_ratio=self.config.gate_selector_min_aspect_ratio,
                    max_aspect_ratio=self.config.gate_selector_max_aspect_ratio,
                    min_appearance_score=(
                        self.config.gate_selector_min_appearance_score
                    ),
                    appearance_weight=self.config.gate_selector_appearance_weight,
                    stable_window_frames=self.config.gate_selector_stable_window_frames,
                    required_stable_frames=self.config.gate_selector_required_stable_frames,
                ),
                detection_stale_s=self.config.webots_detection_stale_s,
                diagnostics_window=self.config.webots_diagnostics_window,
                diagnostics_pass_target_offset_x=(
                    self.config.visual_pass_target_offset_x
                ),
                diagnostics_pass_target_offset_y=(
                    self.config.visual_pass_target_offset_y
                ),
                diagnostics_pass_clearance_left_error=(
                    self.config.visual_pass_clearance_left_error
                ),
                diagnostics_pass_clearance_right_error=(
                    self.config.visual_pass_clearance_right_error
                ),
                diagnostics_pass_clearance_up_error=(
                    self.config.visual_pass_clearance_up_error
                ),
                diagnostics_pass_clearance_down_error=(
                    self.config.visual_pass_clearance_down_error
                ),
                diagnostics_next_gate_min_area_ratio=(
                    self.config.mission_next_gate_acquire_min_area_ratio
                ),
                diagnostics_gate_ready_area_ratio=(
                    self.config.mission_gate_ready_area_ratio
                ),
            )
        )

    def _drain_mavlink(
        self,
        master: object,
        telemetry_adapter: MavlinkTelemetryAdapter,
    ) -> None:
        """Read all currently available MAVLink messages without blocking."""

        while True:
            message = master.recv_match(blocking=False)
            if message is None:
                return
            telemetry_adapter.update_message(message)

    def _sleep_until_next_tick(self, loop_started_s: float) -> None:
        """Keep the loop near the configured rate without hiding slow ticks."""

        period_s = 1.0 / self.config.loop_hz
        remaining_s = period_s - (monotonic() - loop_started_s)
        if remaining_s > 0.0:
            sleep(remaining_s)


def _format_filter_values(values: tuple[object, ...]) -> str:
    """Return a compact operator-facing display for YOLO class filters."""

    if not values:
        return "<none>"
    return ",".join(str(value) for value in values)
