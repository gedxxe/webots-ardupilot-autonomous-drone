from __future__ import annotations

import argparse
from collections.abc import Sequence

from drone_autonomy.perception.yolo_profile import csv_ids, csv_names
from drone_autonomy.runtime.config import AutonomyRuntimeConfig

_RUNTIME_DEFAULTS = AutonomyRuntimeConfig()


def _csv_strings(value: str) -> tuple[str, ...]:
    """Parse comma-separated CLI/env values into a stable tuple."""

    return tuple(item.strip() for item in value.split(",") if item.strip())


def _csv_ints(value: str) -> tuple[int, ...]:
    """Parse comma-separated integer values used for YOLO class ids."""

    if not value.strip():
        return ()
    return tuple(int(item.strip()) for item in value.split(",") if item.strip())


def build_parser() -> argparse.ArgumentParser:
    """Build the CLI used for both MAVLink smoke tests and autonomy runtime."""

    parser = argparse.ArgumentParser(
        prog="drone-autonomy",
        description="Python companion entry point for Webots + ArduPilot SITL.",
    )
    parser.add_argument(
        "--connection",
        default=_RUNTIME_DEFAULTS.connection,
        help="MAVLink connection string.",
    )
    parser.add_argument(
        "--mode",
        choices=["heartbeat", "listen", "autonomy"],
        default="heartbeat",
        help="heartbeat/listen are MAVLink checks; autonomy runs the mission loop.",
    )
    parser.add_argument(
        "--count",
        type=int,
        default=10,
        help="Number of messages to print in listen mode.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=10.0,
        help="Heartbeat wait timeout in seconds.",
    )
    parser.add_argument(
        "--detector",
        choices=["none", "synthetic", "webots-yolo"],
        default="none",
        help="Detection source for autonomy mode.",
    )
    parser.add_argument(
        "--send-commands",
        action="store_true",
        help="Actually send MAVLink motion commands in autonomy mode.",
    )
    parser.add_argument(
        "--loop-hz",
        type=float,
        default=_RUNTIME_DEFAULTS.loop_hz,
        help="Autonomy loop rate in Hz.",
    )
    parser.add_argument(
        "--max-runtime",
        type=float,
        default=_RUNTIME_DEFAULTS.max_runtime_s,
        help="Maximum autonomy runtime in seconds.",
    )
    parser.add_argument(
        "--course-forward-x",
        type=float,
        default=_RUNTIME_DEFAULTS.course_forward_x,
        help="LOCAL_POSITION_NED x component of course-forward direction.",
    )
    parser.add_argument(
        "--course-forward-y",
        type=float,
        default=_RUNTIME_DEFAULTS.course_forward_y,
        help="LOCAL_POSITION_NED y component of course-forward direction.",
    )
    parser.add_argument(
        "--webots-camera-host",
        default=_RUNTIME_DEFAULTS.webots_camera_host,
        help="Host for ArduPilot Webots TCP camera stream.",
    )
    parser.add_argument(
        "--webots-camera-port",
        type=int,
        default=_RUNTIME_DEFAULTS.webots_camera_port,
        help="Port for ArduPilot Webots TCP camera stream.",
    )
    parser.add_argument(
        "--webots-camera-encoding",
        choices=["gray8", "rgb24"],
        default=_RUNTIME_DEFAULTS.webots_camera_encoding,
        help="Camera stream payload format. This repo's iris_camera.wbt uses rgb24.",
    )
    parser.add_argument(
        "--webots-camera-idle-reconnect",
        type=float,
        default=_RUNTIME_DEFAULTS.webots_camera_idle_reconnect_s,
        help="Reconnect Webots camera TCP if no bytes arrive for this many seconds.",
    )
    parser.add_argument(
        "--webots-detection-stale",
        type=float,
        default=_RUNTIME_DEFAULTS.webots_detection_stale_s,
        help="Maximum age in seconds for reusing the latest Webots YOLO detection.",
    )
    parser.add_argument(
        "--webots-diagnostics-window",
        action="store_true",
        help="Show an OpenCV diagnostics window for Webots YOLO frames.",
    )
    parser.add_argument(
        "--yolo-model",
        default="",
        help="Path to YOLO gate model, required for --detector webots-yolo.",
    )
    parser.add_argument(
        "--yolo-confidence",
        type=float,
        default=_RUNTIME_DEFAULTS.yolo_confidence,
        help="Minimum YOLO confidence used before GateDetection conversion.",
    )
    parser.add_argument(
        "--yolo-imgsz",
        type=int,
        default=_RUNTIME_DEFAULTS.yolo_image_size_px,
        help="YOLO inference/letterbox size, not camera frame width/height.",
    )
    parser.add_argument(
        "--yolo-device",
        default=_RUNTIME_DEFAULTS.yolo_device,
        help="YOLO device string such as cpu, cuda, or cuda:0.",
    )
    parser.add_argument(
        "--gate-class-names",
        default=csv_names(_RUNTIME_DEFAULTS.yolo_gate_class_names),
        help=(
            "Comma-separated YOLO class names accepted as gates. Name filtering "
            "is safer than numeric ids when retrained models add classes."
        ),
    )
    parser.add_argument(
        "--gate-class-ids",
        default=csv_ids(_RUNTIME_DEFAULTS.yolo_gate_class_ids),
        help="Comma-separated YOLO class ids accepted as gates.",
    )
    parser.add_argument(
        "--gate-selector-min-seek-confidence",
        type=float,
        default=_RUNTIME_DEFAULTS.gate_selector_min_seek_confidence,
        help="Minimum candidate confidence before a new gate target is locked.",
    )
    parser.add_argument(
        "--gate-selector-min-track-confidence",
        type=float,
        default=_RUNTIME_DEFAULTS.gate_selector_min_track_confidence,
        help="Minimum candidate confidence while tracking a locked gate.",
    )
    parser.add_argument(
        "--gate-selector-min-area-ratio",
        type=float,
        default=_RUNTIME_DEFAULTS.gate_selector_min_area_ratio,
        help="Minimum candidate bbox area as a fraction of frame area.",
    )
    parser.add_argument(
        "--gate-selector-min-aspect-ratio",
        type=float,
        default=_RUNTIME_DEFAULTS.gate_selector_min_aspect_ratio,
        help="Minimum accepted gate bbox width/height ratio.",
    )
    parser.add_argument(
        "--gate-selector-max-aspect-ratio",
        type=float,
        default=_RUNTIME_DEFAULTS.gate_selector_max_aspect_ratio,
        help="Maximum accepted gate bbox width/height ratio.",
    )
    parser.add_argument(
        "--gate-selector-min-appearance-score",
        type=float,
        default=_RUNTIME_DEFAULTS.gate_selector_min_appearance_score,
        help="Minimum hollow-gate appearance score; 0 disables appearance gating.",
    )
    parser.add_argument(
        "--gate-selector-appearance-weight",
        type=float,
        default=_RUNTIME_DEFAULTS.gate_selector_appearance_weight,
        help="Selector scoring weight for hollow-gate appearance evidence.",
    )
    parser.add_argument(
        "--gate-selector-stable-window",
        type=int,
        default=_RUNTIME_DEFAULTS.gate_selector_stable_window_frames,
        help="Frame window used for target validation stability.",
    )
    parser.add_argument(
        "--gate-selector-required-stable",
        type=int,
        default=_RUNTIME_DEFAULTS.gate_selector_required_stable_frames,
        help="Required valid frames inside the selector stability window.",
    )
    parser.add_argument(
        "--mission-max-detection-age",
        type=float,
        default=_RUNTIME_DEFAULTS.mission_max_detection_age_s,
        help="Maximum selected-detection age accepted by mission logic.",
    )
    parser.add_argument(
        "--mission-required-detection-ticks",
        type=int,
        default=_RUNTIME_DEFAULTS.mission_required_detection_ticks,
        help="Consecutive mission ticks required before locking/acquiring a gate.",
    )
    parser.add_argument(
        "--mission-center-dwell",
        type=float,
        default=_RUNTIME_DEFAULTS.mission_center_dwell_s,
        help="Seconds to keep visual centering before a gate pass can commit.",
    )
    parser.add_argument(
        "--mission-center-clearance-required",
        type=float,
        default=_RUNTIME_DEFAULTS.mission_center_clearance_required_s,
        help="Seconds that clearance validator must stay true before passing.",
    )
    parser.add_argument(
        "--mission-center-lost-grace-ticks",
        type=int,
        default=_RUNTIME_DEFAULTS.mission_center_lost_detection_grace_ticks,
        help="CENTER_GATE ticks tolerated without detection before seeking again.",
    )
    parser.add_argument(
        "--mission-seek-yaw-rate",
        type=float,
        default=_RUNTIME_DEFAULTS.mission_seek_yaw_rate_rad_s,
        help="Body yaw rate used while scanning for a gate.",
    )
    parser.add_argument(
        "--mission-gate-pass-distance",
        type=float,
        default=_RUNTIME_DEFAULTS.mission_gate_pass_distance_m,
        help="Forward distance to command after committing through each gate.",
    )
    parser.add_argument(
        "--mission-gate-pass-speed",
        type=float,
        default=_RUNTIME_DEFAULTS.mission_gate_pass_speed_m_s,
        help="Forward body speed while committed to passing a gate.",
    )
    parser.add_argument(
        "--mission-next-gate-acquire-speed",
        type=float,
        default=_RUNTIME_DEFAULTS.mission_next_gate_acquire_speed_m_s,
        help="Forward body speed while clearing gate 1 and acquiring gate 2.",
    )
    parser.add_argument(
        "--mission-next-gate-clear-distance",
        type=float,
        default=_RUNTIME_DEFAULTS.mission_next_gate_acquire_min_clear_distance_m,
        help="Forward clear distance after gate 1 before gate 2 detections count.",
    )
    parser.add_argument(
        "--mission-next-gate-min-area",
        type=float,
        default=_RUNTIME_DEFAULTS.mission_next_gate_acquire_min_area_ratio,
        help="Minimum bbox area ratio before a next-gate candidate is considered.",
    )
    parser.add_argument(
        "--mission-gate-ready-area",
        type=float,
        default=_RUNTIME_DEFAULTS.mission_gate_ready_area_ratio,
        help="BBox area ratio required before centering/pass commitment is allowed.",
    )
    parser.add_argument(
        "--mission-next-gate-max-distance",
        type=float,
        default=_RUNTIME_DEFAULTS.mission_next_gate_acquire_max_distance_m,
        help="Maximum forward acquire distance before falling back to seek.",
    )
    parser.add_argument(
        "--mission-next-gate-timeout",
        type=float,
        default=_RUNTIME_DEFAULTS.mission_next_gate_acquire_timeout_s,
        help="Maximum acquire time before falling back to seek.",
    )
    parser.add_argument(
        "--mission-brake-settle",
        type=float,
        default=_RUNTIME_DEFAULTS.mission_brake_settle_s,
        help="Seconds to command zero forward velocity before center/land transitions.",
    )
    parser.add_argument(
        "--mission-brake-ramp",
        type=float,
        default=_RUNTIME_DEFAULTS.mission_brake_ramp_s,
        help="Seconds used to ramp forward velocity down during BRAKE.",
    )
    parser.add_argument(
        "--mission-brake-altitude-hold",
        action="store_true",
        default=_RUNTIME_DEFAULTS.mission_brake_altitude_hold_enabled,
        help=(
            "Enable companion altitude correction during BRAKE. Default is off "
            "to avoid vertical bounce while decelerating."
        ),
    )
    parser.add_argument(
        "--mission-final-exit-distance",
        type=float,
        default=_RUNTIME_DEFAULTS.mission_final_exit_distance_m,
        help="Forward distance after the last gate before landing.",
    )
    parser.add_argument(
        "--mission-final-exit-speed",
        type=float,
        default=_RUNTIME_DEFAULTS.mission_final_exit_speed_m_s,
        help="Forward body speed during the final exit segment.",
    )
    parser.add_argument(
        "--visual-frame-width",
        type=int,
        default=_RUNTIME_DEFAULTS.visual_frame_width_px,
        help="Camera frame width used for visual-servo geometry.",
    )
    parser.add_argument(
        "--visual-frame-height",
        type=int,
        default=_RUNTIME_DEFAULTS.visual_frame_height_px,
        help="Camera frame height used for visual-servo geometry.",
    )
    parser.add_argument(
        "--visual-min-confidence",
        type=float,
        default=_RUNTIME_DEFAULTS.visual_min_confidence,
        help="Minimum GateDetection confidence accepted by visual servo.",
    )
    parser.add_argument(
        "--visual-filter-alpha",
        type=float,
        default=_RUNTIME_DEFAULTS.visual_filter_alpha,
        help="Low-pass alpha for image errors; lower is smoother.",
    )
    parser.add_argument(
        "--visual-command-filter-alpha",
        type=float,
        default=_RUNTIME_DEFAULTS.visual_command_filter_alpha,
        help="Low-pass alpha for commanded centering velocities.",
    )
    parser.add_argument(
        "--visual-center-deadband-x",
        type=float,
        default=_RUNTIME_DEFAULTS.visual_center_deadband_x,
        help="Normalized horizontal deadband during visual centering.",
    )
    parser.add_argument(
        "--visual-center-deadband-y",
        type=float,
        default=_RUNTIME_DEFAULTS.visual_center_deadband_y,
        help="Normalized vertical deadband during visual centering.",
    )
    parser.add_argument(
        "--visual-aligned-error-x",
        type=float,
        default=_RUNTIME_DEFAULTS.visual_aligned_error_x,
        help="Normalized horizontal error reported as aligned in diagnostics.",
    )
    parser.add_argument(
        "--visual-aligned-error-y",
        type=float,
        default=_RUNTIME_DEFAULTS.visual_aligned_error_y,
        help="Normalized vertical error reported as aligned in diagnostics.",
    )
    parser.add_argument(
        "--visual-pass-target-offset-x",
        type=float,
        default=_RUNTIME_DEFAULTS.visual_pass_target_offset_x,
        help="Normalized x offset for the desired gate center during pass.",
    )
    parser.add_argument(
        "--visual-pass-target-offset-y",
        type=float,
        default=_RUNTIME_DEFAULTS.visual_pass_target_offset_y,
        help="Normalized y offset for the desired gate center during pass.",
    )
    parser.add_argument(
        "--visual-pass-clearance-left",
        type=float,
        default=_RUNTIME_DEFAULTS.visual_pass_clearance_left_error,
        help="Allowed normalized gate-center error to the left of target.",
    )
    parser.add_argument(
        "--visual-pass-clearance-right",
        type=float,
        default=_RUNTIME_DEFAULTS.visual_pass_clearance_right_error,
        help="Allowed normalized gate-center error to the right of target.",
    )
    parser.add_argument(
        "--visual-pass-clearance-up",
        type=float,
        default=_RUNTIME_DEFAULTS.visual_pass_clearance_up_error,
        help="Allowed normalized gate-center error above target.",
    )
    parser.add_argument(
        "--visual-pass-clearance-down",
        type=float,
        default=_RUNTIME_DEFAULTS.visual_pass_clearance_down_error,
        help="Allowed normalized gate-center error below target.",
    )
    parser.add_argument(
        "--visual-max-error-for-forward",
        type=float,
        default=_RUNTIME_DEFAULTS.visual_max_error_for_forward,
        help="Largest normalized centering error that still allows approach speed.",
    )
    parser.add_argument(
        "--visual-min-forward-speed",
        type=float,
        default=_RUNTIME_DEFAULTS.visual_min_forward_speed_m_s,
        help="Minimum forward speed during CENTER_GATE visual servoing.",
    )
    parser.add_argument(
        "--visual-max-forward-speed",
        type=float,
        default=_RUNTIME_DEFAULTS.visual_max_forward_speed_m_s,
        help="Maximum forward speed during CENTER_GATE visual servoing.",
    )
    parser.add_argument(
        "--visual-lateral-kp",
        type=float,
        default=_RUNTIME_DEFAULTS.visual_lateral_kp,
        help="Proportional gain for body-right correction during centering.",
    )
    parser.add_argument(
        "--visual-vertical-kp",
        type=float,
        default=_RUNTIME_DEFAULTS.visual_vertical_kp,
        help="Proportional gain for body-down correction during centering.",
    )
    parser.add_argument(
        "--visual-yaw-kp",
        type=float,
        default=_RUNTIME_DEFAULTS.visual_yaw_kp,
        help="Proportional gain for yaw correction during centering.",
    )
    parser.add_argument(
        "--visual-max-lateral-speed",
        type=float,
        default=_RUNTIME_DEFAULTS.visual_max_lateral_speed_m_s,
        help="Maximum body-right speed during centering.",
    )
    parser.add_argument(
        "--visual-max-vertical-speed",
        type=float,
        default=_RUNTIME_DEFAULTS.visual_max_vertical_speed_m_s,
        help="Maximum body-down speed during centering.",
    )
    parser.add_argument(
        "--visual-max-yaw-rate",
        type=float,
        default=_RUNTIME_DEFAULTS.visual_max_yaw_rate_rad_s,
        help="Maximum yaw rate during centering.",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """Run the requested CLI mode.

    `heartbeat` and `listen` are diagnostics. `autonomy` starts the blocking
    runtime loop that wires telemetry, detector, mission, and command adapter.
    """

    args = build_parser().parse_args(argv)

    if args.mode == "autonomy":
        from drone_autonomy.runtime.autonomy_loop import AutonomyRuntime

        result = AutonomyRuntime(
            AutonomyRuntimeConfig(
                connection=args.connection,
                loop_hz=args.loop_hz,
                max_runtime_s=args.max_runtime,
                heartbeat_timeout_s=args.timeout,
                detector=args.detector,
                send_commands=args.send_commands,
                course_forward_x=args.course_forward_x,
                course_forward_y=args.course_forward_y,
                webots_camera_host=args.webots_camera_host,
                webots_camera_port=args.webots_camera_port,
                webots_camera_encoding=args.webots_camera_encoding,
                webots_camera_idle_reconnect_s=args.webots_camera_idle_reconnect,
                webots_detection_stale_s=args.webots_detection_stale,
                webots_diagnostics_window=args.webots_diagnostics_window,
                yolo_model_path=args.yolo_model,
                yolo_confidence=args.yolo_confidence,
                yolo_image_size_px=args.yolo_imgsz,
                yolo_device=args.yolo_device,
                yolo_gate_class_names=_csv_strings(args.gate_class_names),
                yolo_gate_class_ids=_csv_ints(args.gate_class_ids),
                gate_selector_min_seek_confidence=args.gate_selector_min_seek_confidence,
                gate_selector_min_track_confidence=args.gate_selector_min_track_confidence,
                gate_selector_min_area_ratio=args.gate_selector_min_area_ratio,
                gate_selector_min_aspect_ratio=args.gate_selector_min_aspect_ratio,
                gate_selector_max_aspect_ratio=args.gate_selector_max_aspect_ratio,
                gate_selector_min_appearance_score=(
                    args.gate_selector_min_appearance_score
                ),
                gate_selector_appearance_weight=args.gate_selector_appearance_weight,
                gate_selector_stable_window_frames=args.gate_selector_stable_window,
                gate_selector_required_stable_frames=args.gate_selector_required_stable,
                mission_max_detection_age_s=args.mission_max_detection_age,
                mission_required_detection_ticks=args.mission_required_detection_ticks,
                mission_center_dwell_s=args.mission_center_dwell,
                mission_center_clearance_required_s=(
                    args.mission_center_clearance_required
                ),
                mission_center_lost_detection_grace_ticks=(
                    args.mission_center_lost_grace_ticks
                ),
                mission_seek_yaw_rate_rad_s=args.mission_seek_yaw_rate,
                mission_gate_pass_distance_m=args.mission_gate_pass_distance,
                mission_gate_pass_speed_m_s=args.mission_gate_pass_speed,
                mission_next_gate_acquire_speed_m_s=(
                    args.mission_next_gate_acquire_speed
                ),
                mission_next_gate_acquire_min_clear_distance_m=(
                    args.mission_next_gate_clear_distance
                ),
                mission_next_gate_acquire_min_area_ratio=(
                    args.mission_next_gate_min_area
                ),
                mission_gate_ready_area_ratio=args.mission_gate_ready_area,
                mission_next_gate_acquire_max_distance_m=(
                    args.mission_next_gate_max_distance
                ),
                mission_next_gate_acquire_timeout_s=args.mission_next_gate_timeout,
                mission_brake_settle_s=args.mission_brake_settle,
                mission_brake_ramp_s=args.mission_brake_ramp,
                mission_brake_altitude_hold_enabled=args.mission_brake_altitude_hold,
                mission_final_exit_distance_m=args.mission_final_exit_distance,
                mission_final_exit_speed_m_s=args.mission_final_exit_speed,
                visual_frame_width_px=args.visual_frame_width,
                visual_frame_height_px=args.visual_frame_height,
                visual_min_confidence=args.visual_min_confidence,
                visual_filter_alpha=args.visual_filter_alpha,
                visual_command_filter_alpha=args.visual_command_filter_alpha,
                visual_center_deadband_x=args.visual_center_deadband_x,
                visual_center_deadband_y=args.visual_center_deadband_y,
                visual_aligned_error_x=args.visual_aligned_error_x,
                visual_aligned_error_y=args.visual_aligned_error_y,
                visual_pass_target_offset_x=args.visual_pass_target_offset_x,
                visual_pass_target_offset_y=args.visual_pass_target_offset_y,
                visual_pass_clearance_left_error=args.visual_pass_clearance_left,
                visual_pass_clearance_right_error=args.visual_pass_clearance_right,
                visual_pass_clearance_up_error=args.visual_pass_clearance_up,
                visual_pass_clearance_down_error=args.visual_pass_clearance_down,
                visual_max_error_for_forward=args.visual_max_error_for_forward,
                visual_min_forward_speed_m_s=args.visual_min_forward_speed,
                visual_max_forward_speed_m_s=args.visual_max_forward_speed,
                visual_lateral_kp=args.visual_lateral_kp,
                visual_vertical_kp=args.visual_vertical_kp,
                visual_yaw_kp=args.visual_yaw_kp,
                visual_max_lateral_speed_m_s=args.visual_max_lateral_speed,
                visual_max_vertical_speed_m_s=args.visual_max_vertical_speed,
                visual_max_yaw_rate_rad_s=args.visual_max_yaw_rate,
            )
        ).run()
        return 0 if result.completed else 2

    from drone_autonomy.mavlink.connection import MavlinkClient

    client = MavlinkClient(args.connection)
    heartbeat = client.wait_heartbeat(timeout=args.timeout)
    print(
        "heartbeat "
        f"system={heartbeat.get_srcSystem()} "
        f"component={heartbeat.get_srcComponent()} "
        f"type={heartbeat.type} "
        f"autopilot={heartbeat.autopilot}"
    )

    if args.mode == "listen":
        for message in client.iter_messages(count=args.count):
            print(message)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
