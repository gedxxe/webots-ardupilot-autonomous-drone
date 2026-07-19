from __future__ import annotations

from dataclasses import dataclass
from time import monotonic, sleep
from typing import Protocol

from pymavlink import mavutil

from drone_autonomy.autonomy.commands import VehicleCommand
from drone_autonomy.autonomy.mission import (
    GateAutonomyMission,
    MissionOutput,
    MissionPhase,
)
from drone_autonomy.control.visual_servo import GateVisualServoController
from drone_autonomy.mavlink.commands import (
    MavlinkCommandAdapter,
    MavlinkCommandAdapterConfig,
)
from drone_autonomy.mavlink.telemetry import CourseFrame, MavlinkTelemetryAdapter
from drone_autonomy.perception.detections import GateDetection
from drone_autonomy.perception.synthetic import SyntheticGateProvider
from drone_autonomy.perception.webots_camera import WebotsCameraConfig
from drone_autonomy.perception.webots_yolo import (
    WebotsYoloConfig,
    WebotsYoloGateProvider,
)
from drone_autonomy.perception.yolo import YoloGateConfig
from drone_autonomy.runtime.config import AutonomyRuntimeConfig
from drone_autonomy.runtime.jsonl_logger import RuntimeJsonlLogger


class GateDetectionProvider(Protocol):
    """Runtime-owned detector source interface.

    Providers may own I/O resources. The mission still receives only
    `GateDetection | None`, so camera/model details do not leak into autonomy.
    """

    def detect(self, now_s: float) -> GateDetection | None:
        """Return the latest gate observation or `None`."""

    def close(self) -> None:
        """Release provider resources."""


class ContextualGateDetectionProvider(GateDetectionProvider, Protocol):
    """Detector provider that accepts mission context for target selection."""

    def update_context(self, *, phase: str, gate_index: int) -> None:
        """Update mission phase/gate context before `detect()` is called."""

    def process_events(self) -> None:
        """Process optional provider UI events without blocking mission logic."""

    def stale_reason(self, now_s: float) -> str | None:
        """Return why perception progress is unsafe, or `None` when healthy."""


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
        if self.config.detector not in {
            "none",
            "synthetic",
            "webots-yolo",
            "opencv-yolo",
        }:
            raise ValueError(
                "detector must be 'none', 'synthetic', 'webots-yolo', or 'opencv-yolo'"
            )
        # Validate all pure domain policies before `run()` opens MAVLink or
        # starts perception workers. The same builders are used by preflight.
        self.config.mission_config()
        self.config.visual_servo_config()
        self.config.target_selector_config()

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
        logger = RuntimeJsonlLogger(self.config.log_jsonl_path)
        logger.write(
            "runtime_start",
            connection=self.config.connection,
            baud=self.config.mavlink_baud,
            detector=self.config.detector,
            send_commands=self.config.send_commands,
            loop_hz=self.config.loop_hz,
        )

        mission = self._build_mission()
        command_adapter = MavlinkCommandAdapter(
            master,
            MavlinkCommandAdapterConfig(
                command_ack_required=self.config.command_ack_required,
                command_ack_timeout_s=self.config.command_ack_timeout_s,
                command_ack_max_retries=self.config.command_ack_max_retries,
            ),
        )
        telemetry_adapter = MavlinkTelemetryAdapter(
            CourseFrame(
                forward_x=self.config.course_forward_x,
                forward_y=self.config.course_forward_y,
            )
        )
        telemetry_adapter.update_message(heartbeat, observed_at_s=monotonic())
        synthetic_gate = (
            SyntheticGateProvider() if self.config.detector == "synthetic" else None
        )
        yolo_gate = self._build_yolo_provider()

        start_s = monotonic()
        next_status_s = start_s
        last_output: MissionOutput | None = None

        try:
            try:
                command_adapter.request_default_telemetry(rate_hz=self.config.loop_hz)
            except Exception as exc:
                print(f"warning: telemetry interval request failed: {exc}")

            while monotonic() - start_s <= self.config.max_runtime_s:
                loop_started_s = monotonic()
                # Drain all queued MAVLink messages before building the snapshot so
                # the mission always sees the freshest available fused telemetry.
                self._drain_mavlink(
                    master,
                    telemetry_adapter,
                    command_adapter,
                    loop_started_s,
                )
                if yolo_gate is not None:
                    # Qt-backed OpenCV windows must be pumped by this main
                    # runtime thread, including while telemetry is stale.
                    yolo_gate.process_events()
                command_adapter.update_ack_timeouts(loop_started_s)
                for ack_event in command_adapter.pop_ack_events():
                    print(
                        "mavlink-ack "
                        f"severity={ack_event.severity} "
                        f"command={ack_event.command_id} "
                        f"label={ack_event.label} "
                        f"attempts={ack_event.attempts} "
                        f"detail={ack_event.detail}"
                    )
                    logger.write(
                        "mavlink_ack",
                        severity=ack_event.severity,
                        command_id=ack_event.command_id,
                        label=ack_event.label,
                        attempts=ack_event.attempts,
                        detail=ack_event.detail,
                    )
                ack_failures = command_adapter.pop_ack_failures()
                if ack_failures:
                    failure = ack_failures[0]
                    reason = (
                        f"COMMAND_ACK failure command={failure.command_id} "
                        f"label={failure.label} detail={failure.detail}"
                    )
                    if self.config.send_commands:
                        command_adapter.send(
                            VehicleCommand.hold(f"fail-closed: {reason}"),
                            now_s=loop_started_s,
                        )
                    print(f"fail-closed {reason}")
                    logger.write(
                        "runtime_fail_closed",
                        reason=reason,
                        phase=mission.phase.value,
                    )
                    return AutonomyRuntimeResult(False, mission.phase, last_output)

                stale_reason = telemetry_adapter.stale_reason(
                    loop_started_s,
                    heartbeat_stale_s=self.config.mavlink_heartbeat_stale_s,
                    local_position_stale_s=(
                        self.config.mavlink_local_position_stale_s
                    ),
                )
                if stale_reason is not None:
                    if self.config.send_commands:
                        command_adapter.send(
                            VehicleCommand.hold(f"fail-closed: {stale_reason}"),
                            now_s=loop_started_s,
                        )
                    if loop_started_s >= next_status_s:
                        print(f"fail-closed waiting for fresh MAVLink: {stale_reason}")
                        logger.write(
                            "mavlink_wait_fresh",
                            reason=stale_reason,
                            phase=mission.phase.value,
                        )
                        next_status_s = loop_started_s + self.config.status_interval_s
                    self._sleep_until_next_tick(loop_started_s)
                    continue

                perception_stale_reason = (
                    yolo_gate.stale_reason(loop_started_s)
                    if yolo_gate is not None
                    else None
                )
                if perception_stale_reason is not None:
                    if self.config.send_commands:
                        command_adapter.send(
                            VehicleCommand.hold(
                                f"fail-closed: {perception_stale_reason}"
                            ),
                            now_s=loop_started_s,
                        )
                    if loop_started_s >= next_status_s:
                        print(
                            "fail-closed waiting for fresh perception: "
                            f"{perception_stale_reason}"
                        )
                        logger.write(
                            "perception_wait_fresh",
                            reason=perception_stale_reason,
                            phase=mission.phase.value,
                        )
                        next_status_s = loop_started_s + self.config.status_interval_s
                    self._sleep_until_next_tick(loop_started_s)
                    continue

                detection = None
                if synthetic_gate is not None:
                    # Synthetic perception is phase-aware and exists only to test
                    # mission/MAVLink wiring while bypassing real camera/model I/O.
                    detection = synthetic_gate.detect_for_phase(
                        loop_started_s,
                        mission.phase,
                        mission.gate_index,
                    )
                elif yolo_gate is not None:
                    # Webots and C920 paths share YOLO + target selection. The
                    # frame source changes; mission input stays GateDetection.
                    yolo_gate.update_context(
                        phase=mission.phase.value,
                        gate_index=mission.gate_index,
                    )
                    detection = yolo_gate.detect(loop_started_s)

                telemetry = telemetry_adapter.snapshot(loop_started_s, detection)
                if telemetry is None:
                    # Mission must not run until local-position telemetry exists;
                    # otherwise altitude and forward distance would be fabricated.
                    if loop_started_s >= next_status_s:
                        print("waiting for LOCAL_POSITION_NED telemetry")
                        logger.write(
                            "local_position_wait",
                            phase=mission.phase.value,
                        )
                        next_status_s = loop_started_s + self.config.status_interval_s
                    self._sleep_until_next_tick(loop_started_s)
                    continue

                last_output = mission.update(telemetry)
                command_emitted: bool | None = None
                if self.config.send_commands:
                    # This is the only point where mission output becomes a real
                    # MAVLink command. Without `--send-commands`, runtime is dry-run.
                    command_emitted = command_adapter.send(
                        last_output.command,
                        now_s=loop_started_s,
                    )
                self._log_mission_tick(
                    logger,
                    last_output,
                    telemetry,
                    detection,
                    command_emitted=command_emitted,
                )

                if loop_started_s >= next_status_s:
                    if not self.config.send_commands:
                        command_status = "dry-run"
                    elif command_emitted:
                        command_status = "sent"
                    else:
                        command_status = "suppressed"
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
                        f"{command_status} phase={last_output.phase.value} "
                        f"gate={last_output.gate_index + 1} "
                        f"cmd={last_output.command.kind.value} "
                        f"detail={last_output.detail}"
                        f"{servo_detail}"
                    )
                    next_status_s = loop_started_s + self.config.status_interval_s

                if last_output.phase == MissionPhase.COMPLETE:
                    logger.write(
                        "runtime_complete",
                        phase=mission.phase.value,
                        gate_index=last_output.gate_index,
                    )
                    return AutonomyRuntimeResult(True, mission.phase, last_output)
                if last_output.phase == MissionPhase.FAILSAFE:
                    logger.write(
                        "runtime_failsafe",
                        phase=mission.phase.value,
                        gate_index=last_output.gate_index,
                        detail=last_output.detail,
                    )
                    return AutonomyRuntimeResult(False, mission.phase, last_output)

                self._sleep_until_next_tick(loop_started_s)
            timeout_reason = (
                f"runtime exceeded MAX_RUNTIME={self.config.max_runtime_s:0.1f}s"
            )
            timeout_output = MissionOutput(
                phase=MissionPhase.FAILSAFE,
                command=VehicleCommand.land(f"fail-closed: {timeout_reason}"),
                gate_index=mission.gate_index,
                detail=timeout_reason,
            )
            timeout_command_emitted: bool | None = None
            if self.config.send_commands:
                timeout_command_emitted = command_adapter.send(
                    timeout_output.command,
                    now_s=monotonic(),
                )
            if not self.config.send_commands:
                command_status = "dry-run"
                timeout_action = "dry-run"
            elif timeout_command_emitted:
                command_status = "sent"
                timeout_action = "land_sent"
            else:
                command_status = "suppressed"
                timeout_action = "land_suppressed"
            print(
                f"{command_status} phase={MissionPhase.FAILSAFE.value} "
                f"gate={mission.gate_index + 1} cmd=land detail={timeout_reason}"
            )
            logger.write(
                "runtime_timeout",
                phase=mission.phase.value,
                max_runtime_s=self.config.max_runtime_s,
                action=timeout_action,
            )
            return AutonomyRuntimeResult(
                False,
                MissionPhase.FAILSAFE,
                timeout_output,
            )
        finally:
            if yolo_gate is not None:
                yolo_gate.close()
            logger.close()

    def _log_mission_tick(
        self,
        logger: RuntimeJsonlLogger,
        output: MissionOutput,
        telemetry: object,
        detection: GateDetection | None,
        *,
        command_emitted: bool | None,
    ) -> None:
        """Write one optional mission/control diagnostics record."""

        command = output.command
        payload: dict[str, object] = {
            "phase": output.phase.value,
            "gate_index": output.gate_index,
            "detail": output.detail,
            "command_kind": command.kind.value,
            "command_reason": command.reason,
            "command_emitted": command_emitted,
            "body_vx_m_s": command.body_vx_m_s,
            "body_vy_m_s": command.body_vy_m_s,
            "body_vz_m_s": command.body_vz_m_s,
            "yaw_rate_rad_s": command.yaw_rate_rad_s,
            "altitude_m": getattr(telemetry, "altitude_m", None),
            "forward_position_m": getattr(telemetry, "forward_position_m", None),
            "mode": getattr(telemetry, "mode", None),
            "armed": getattr(telemetry, "armed", None),
            "landed": getattr(telemetry, "landed", None),
        }
        if detection is not None:
            bbox = detection.bbox
            payload.update(
                {
                    "detection_confidence": detection.confidence,
                    "detection_class": detection.class_name,
                    "detection_track_id": detection.track_id,
                    "bbox_x_min": bbox.x_min,
                    "bbox_y_min": bbox.y_min,
                    "bbox_x_max": bbox.x_max,
                    "bbox_y_max": bbox.y_max,
                }
            )
        if output.servo is not None:
            servo = output.servo
            payload.update(
                {
                    "servo_error_x": servo.error_x,
                    "servo_error_y": servo.error_y,
                    "servo_area_ratio": servo.area_ratio,
                    "servo_aligned": servo.is_aligned,
                    "servo_clearance_ready": servo.clearance_ready,
                    "servo_pass_ready": servo.pass_ready,
                }
            )
        logger.write("mission_tick", **payload)

    def _build_mission(self) -> GateAutonomyMission:
        """Build mission/control objects from process-level runtime config."""

        return GateAutonomyMission(
            self.config.mission_config(),
            GateVisualServoController(self.config.visual_servo_config()),
        )

    def _build_yolo_provider(self) -> ContextualGateDetectionProvider | None:
        """Build the selected YOLO-backed provider, if any."""

        if self.config.detector == "webots-yolo":
            return self._build_webots_yolo_provider()
        if self.config.detector == "opencv-yolo":
            return self._build_opencv_yolo_provider()
        return None

    def _build_webots_yolo_provider(self) -> ContextualGateDetectionProvider:
        """Build the optional Webots+YOLO provider only when requested."""

        if not self.config.yolo_model_path:
            raise ValueError("--yolo-model is required when --detector webots-yolo")

        print(
            "webots-yolo camera="
            f"tcp://{self.config.webots_camera_host}:{self.config.webots_camera_port} "
            f"encoding={self.config.webots_camera_encoding} "
            f"model={self.config.yolo_model_path}"
        )
        self._print_yolo_class_filter("webots-yolo")
        return WebotsYoloGateProvider(
            self._build_yolo_provider_config(
                camera=WebotsCameraConfig(
                    host=self.config.webots_camera_host,
                    port=self.config.webots_camera_port,
                    encoding=self.config.webots_camera_encoding,
                    idle_reconnect_s=self.config.webots_camera_idle_reconnect_s,
                ),
                detection_stale_s=self.config.webots_detection_stale_s,
                pipeline_name="webots-yolo",
                diagnostics_window=self.config.webots_diagnostics_window,
            )
        )

    def _build_opencv_yolo_provider(self) -> ContextualGateDetectionProvider:
        """Build the optional Raspberry Pi OpenCV camera plus YOLO provider."""

        if not self.config.yolo_model_path:
            raise ValueError("--yolo-model is required when --detector opencv-yolo")

        from drone_autonomy.perception.opencv_camera import (
            OpenCvCameraConfig,
            OpenCvCameraSource,
        )

        print(
            "opencv-yolo camera="
            f"source={self.config.opencv_camera_source} "
            f"backend={self.config.opencv_camera_backend} "
            f"requested={self.config.opencv_camera_width_px}x"
            f"{self.config.opencv_camera_height_px}@"
            f"{self.config.opencv_camera_fps:0.1f} "
            f"model={self.config.yolo_model_path}"
        )
        self._print_yolo_class_filter("opencv-yolo")
        camera_source = OpenCvCameraSource(
            OpenCvCameraConfig(
                source=self.config.opencv_camera_source,
                backend=self.config.opencv_camera_backend,
                width_px=self.config.opencv_camera_width_px,
                height_px=self.config.opencv_camera_height_px,
                fps=self.config.opencv_camera_fps,
                read_timeout_s=self.config.opencv_camera_read_timeout_s,
                open_retry_s=self.config.opencv_camera_open_retry_s,
            )
        )
        return WebotsYoloGateProvider(
            self._build_yolo_provider_config(
                # The generic provider needs a timing fallback. The real frame
                # source is passed explicitly below, so no Webots TCP socket is used.
                camera=WebotsCameraConfig(
                    read_timeout_s=self.config.opencv_camera_read_timeout_s,
                    idle_reconnect_s=max(
                        self.config.opencv_camera_open_retry_s,
                        self.config.opencv_camera_read_timeout_s * 2.0,
                    ),
                ),
                detection_stale_s=self.config.opencv_detection_stale_s,
                pipeline_name="opencv-yolo",
                camera_wait_source=f"opencv:{self.config.opencv_camera_source}",
                camera_read_timeout_s=self.config.opencv_camera_read_timeout_s,
                diagnostics_window=self.config.opencv_diagnostics_window,
                diagnostics_window_name="OpenCV YOLO Gate Diagnostics",
            ),
            camera=camera_source,
        )

    def _build_yolo_provider_config(
        self,
        *,
        camera: WebotsCameraConfig,
        detection_stale_s: float,
        pipeline_name: str,
        diagnostics_window: bool,
        camera_wait_source: str = "",
        camera_read_timeout_s: float = 0.0,
        diagnostics_window_name: str = "Webots YOLO Gate Diagnostics",
    ) -> WebotsYoloConfig:
        """Build shared YOLO/selector/diagnostics config for any frame source."""

        return WebotsYoloConfig(
            camera=camera,
            yolo=YoloGateConfig(
                model_path=self.config.yolo_model_path,
                confidence=self.config.yolo_confidence,
                image_size_px=self.config.yolo_image_size_px,
                device=self.config.yolo_device,
                gate_class_names=self.config.yolo_gate_class_names,
                gate_class_ids=self.config.yolo_gate_class_ids,
            ),
            selector=self.config.target_selector_config(),
            detection_stale_s=detection_stale_s,
            pipeline_name=pipeline_name,
            camera_wait_source=camera_wait_source,
            camera_read_timeout_s=camera_read_timeout_s,
            diagnostics_window=diagnostics_window,
            diagnostics_window_name=diagnostics_window_name,
            diagnostics_pass_target_offset_x=self.config.visual_pass_target_offset_x,
            diagnostics_pass_target_offset_y=self.config.visual_pass_target_offset_y,
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
            diagnostics_gate_ready_area_ratio=self.config.mission_gate_ready_area_ratio,
        )

    def _print_yolo_class_filter(self, pipeline_name: str) -> None:
        print(
            f"{pipeline_name} class_filter "
            f"names={_format_filter_values(self.config.yolo_gate_class_names)} "
            f"ids={_format_filter_values(self.config.yolo_gate_class_ids)}"
        )

    def _drain_mavlink(
        self,
        master: object,
        telemetry_adapter: MavlinkTelemetryAdapter,
        command_adapter: MavlinkCommandAdapter,
        observed_at_s: float,
    ) -> None:
        """Read all currently available MAVLink messages without blocking."""

        while True:
            message = master.recv_match(blocking=False)
            if message is None:
                return
            telemetry_adapter.update_message(message, observed_at_s=observed_at_s)
            command_adapter.update_message(message)

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
