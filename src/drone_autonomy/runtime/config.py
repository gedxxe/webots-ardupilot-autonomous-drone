from __future__ import annotations

from dataclasses import dataclass

from drone_autonomy.autonomy.mission import GateMissionConfig
from drone_autonomy.control.visual_servo import VisualServoConfig
from drone_autonomy.perception.target_selector import GateTargetSelectorConfig
from drone_autonomy.perception.yolo_profile import (
    DEFAULT_GATE_CLASS_IDS,
    DEFAULT_GATE_CLASS_NAMES,
)

_MISSION_DEFAULTS = GateMissionConfig()
_SELECTOR_DEFAULTS = GateTargetSelectorConfig()
_VISUAL_DEFAULTS = VisualServoConfig()


@dataclass(frozen=True)
class AutonomyRuntimeConfig:
    """Configuration for the process-level autonomy loop.

    Defaults are intentionally dry-run oriented. Launch scripts and local env
    files select the active detector/profile; this object remains adapter-free
    so CLI defaults and `--help` do not require MAVLink imports.
    """

    connection: str = "udp:127.0.0.1:14551"
    mavlink_baud: int = 115200
    loop_hz: float = 20.0
    max_runtime_s: float = 180.0
    heartbeat_timeout_s: float = 30.0
    mavlink_heartbeat_stale_s: float = 3.0
    mavlink_local_position_stale_s: float = 1.0
    command_ack_required: bool = True
    command_ack_timeout_s: float = 1.0
    command_ack_max_retries: int = 2
    status_interval_s: float = 1.0
    log_jsonl_path: str = ""
    detector: str = "none"
    send_commands: bool = False
    course_forward_x: float = 1.0
    course_forward_y: float = 0.0
    webots_camera_host: str = "127.0.0.1"
    webots_camera_port: int = 5599
    webots_camera_encoding: str = "rgb24"
    webots_camera_idle_reconnect_s: float = 2.0
    webots_detection_stale_s: float = 0.75
    webots_diagnostics_window: bool = False
    opencv_camera_source: str = "0"
    opencv_camera_backend: str = "default"
    opencv_camera_width_px: int = 640
    opencv_camera_height_px: int = 480
    opencv_camera_fps: float = 30.0
    opencv_camera_read_timeout_s: float = 0.05
    opencv_camera_open_retry_s: float = 1.0
    opencv_detection_stale_s: float = 0.75
    opencv_diagnostics_window: bool = False
    yolo_model_path: str = ""
    yolo_confidence: float = 0.35
    yolo_image_size_px: int = 640
    yolo_device: str = "cpu"
    yolo_gate_class_names: tuple[str, ...] = DEFAULT_GATE_CLASS_NAMES
    yolo_gate_class_ids: tuple[int, ...] = DEFAULT_GATE_CLASS_IDS
    gate_selector_min_seek_confidence: float = _SELECTOR_DEFAULTS.min_seek_confidence
    gate_selector_min_track_confidence: float = _SELECTOR_DEFAULTS.min_track_confidence
    gate_selector_min_area_ratio: float = _SELECTOR_DEFAULTS.min_area_ratio
    gate_selector_min_aspect_ratio: float = _SELECTOR_DEFAULTS.min_aspect_ratio
    gate_selector_max_aspect_ratio: float = _SELECTOR_DEFAULTS.max_aspect_ratio
    gate_selector_min_appearance_score: float = (
        _SELECTOR_DEFAULTS.min_appearance_score
    )
    gate_selector_appearance_weight: float = _SELECTOR_DEFAULTS.appearance_weight
    gate_selector_stable_window_frames: int = (
        _SELECTOR_DEFAULTS.stable_window_frames
    )
    gate_selector_required_stable_frames: int = (
        _SELECTOR_DEFAULTS.required_stable_frames
    )
    mission_takeoff_altitude_m: float = _MISSION_DEFAULTS.takeoff_altitude_m
    mission_takeoff_settle_tolerance_m: float = (
        _MISSION_DEFAULTS.takeoff_settle_tolerance_m
    )
    mission_takeoff_required_stable_ticks: int = (
        _MISSION_DEFAULTS.takeoff_required_stable_ticks
    )
    mission_takeoff_timeout_s: float = _MISSION_DEFAULTS.takeoff_timeout_s
    mission_max_detection_age_s: float = _MISSION_DEFAULTS.max_detection_age_s
    mission_required_detection_ticks: int = (
        _MISSION_DEFAULTS.required_detection_ticks
    )
    mission_center_dwell_s: float = _MISSION_DEFAULTS.center_dwell_s
    mission_center_clearance_required_s: float = (
        _MISSION_DEFAULTS.center_clearance_required_s
    )
    mission_center_lost_detection_grace_ticks: int = (
        _MISSION_DEFAULTS.center_lost_detection_grace_ticks
    )
    mission_seek_yaw_rate_rad_s: float = _MISSION_DEFAULTS.seek_yaw_rate_rad_s
    mission_gate_pass_distance_m: float = _MISSION_DEFAULTS.gate_pass_distance_m
    mission_gate_pass_speed_m_s: float = _MISSION_DEFAULTS.gate_pass_speed_m_s
    mission_next_gate_acquire_speed_m_s: float = (
        _MISSION_DEFAULTS.next_gate_acquire_speed_m_s
    )
    mission_next_gate_acquire_min_clear_distance_m: float = (
        _MISSION_DEFAULTS.next_gate_acquire_min_clear_distance_m
    )
    mission_next_gate_acquire_min_area_ratio: float = (
        _MISSION_DEFAULTS.next_gate_acquire_min_area_ratio
    )
    mission_gate_ready_area_ratio: float = _MISSION_DEFAULTS.gate_ready_area_ratio
    mission_next_gate_acquire_max_distance_m: float = (
        _MISSION_DEFAULTS.next_gate_acquire_max_distance_m
    )
    mission_next_gate_acquire_timeout_s: float = (
        _MISSION_DEFAULTS.next_gate_acquire_timeout_s
    )
    mission_brake_settle_s: float = _MISSION_DEFAULTS.brake_settle_s
    mission_brake_ramp_s: float = _MISSION_DEFAULTS.brake_ramp_s
    mission_brake_altitude_hold_enabled: bool = (
        _MISSION_DEFAULTS.brake_altitude_hold_enabled
    )
    mission_final_exit_distance_m: float = _MISSION_DEFAULTS.final_exit_distance_m
    mission_final_exit_speed_m_s: float = _MISSION_DEFAULTS.final_exit_speed_m_s
    mission_min_centering_altitude_m: float = (
        _MISSION_DEFAULTS.min_centering_altitude_m
    )
    mission_max_centering_altitude_m: float = (
        _MISSION_DEFAULTS.max_centering_altitude_m
    )
    mission_altitude_hold_enabled: bool = _MISSION_DEFAULTS.altitude_hold_enabled
    mission_altitude_hold_deadband_m: float = (
        _MISSION_DEFAULTS.altitude_hold_deadband_m
    )
    mission_altitude_hold_kp: float = _MISSION_DEFAULTS.altitude_hold_kp
    mission_altitude_hold_max_climb_m_s: float = (
        _MISSION_DEFAULTS.altitude_hold_max_climb_m_s
    )
    mission_altitude_hold_max_descent_m_s: float = (
        _MISSION_DEFAULTS.altitude_hold_max_descent_m_s
    )
    mission_landing_complete_altitude_m: float = (
        _MISSION_DEFAULTS.landing_complete_altitude_m
    )
    mission_timeout_s: float = _MISSION_DEFAULTS.mission_timeout_s
    visual_frame_width_px: int = _VISUAL_DEFAULTS.frame_width_px
    visual_frame_height_px: int = _VISUAL_DEFAULTS.frame_height_px
    visual_min_confidence: float = _VISUAL_DEFAULTS.min_confidence
    visual_filter_alpha: float = _VISUAL_DEFAULTS.filter_alpha
    visual_command_filter_alpha: float = _VISUAL_DEFAULTS.command_filter_alpha
    visual_center_deadband_x: float = _VISUAL_DEFAULTS.center_deadband_x
    visual_center_deadband_y: float = _VISUAL_DEFAULTS.center_deadband_y
    visual_aligned_error_x: float = _VISUAL_DEFAULTS.aligned_error_x
    visual_aligned_error_y: float = _VISUAL_DEFAULTS.aligned_error_y
    visual_pass_target_offset_x: float = _VISUAL_DEFAULTS.pass_target_offset_x
    visual_pass_target_offset_y: float = _VISUAL_DEFAULTS.pass_target_offset_y
    visual_pass_clearance_left_error: float = (
        _VISUAL_DEFAULTS.pass_clearance_left_error
    )
    visual_pass_clearance_right_error: float = (
        _VISUAL_DEFAULTS.pass_clearance_right_error
    )
    visual_pass_clearance_up_error: float = _VISUAL_DEFAULTS.pass_clearance_up_error
    visual_pass_clearance_down_error: float = (
        _VISUAL_DEFAULTS.pass_clearance_down_error
    )
    visual_max_error_for_forward: float = _VISUAL_DEFAULTS.max_error_for_forward
    visual_min_forward_speed_m_s: float = _VISUAL_DEFAULTS.min_forward_speed_m_s
    visual_max_forward_speed_m_s: float = _VISUAL_DEFAULTS.max_forward_speed_m_s
    visual_lateral_kp: float = _VISUAL_DEFAULTS.lateral_kp
    visual_vertical_kp: float = _VISUAL_DEFAULTS.vertical_kp
    visual_yaw_kp: float = _VISUAL_DEFAULTS.yaw_kp
    visual_max_lateral_speed_m_s: float = _VISUAL_DEFAULTS.max_lateral_speed_m_s
    visual_max_vertical_speed_m_s: float = _VISUAL_DEFAULTS.max_vertical_speed_m_s
    visual_max_yaw_rate_rad_s: float = _VISUAL_DEFAULTS.max_yaw_rate_rad_s

    def mission_config(self) -> GateMissionConfig:
        """Build the domain mission policy from operator runtime settings."""

        return GateMissionConfig(
            takeoff_altitude_m=self.mission_takeoff_altitude_m,
            takeoff_settle_tolerance_m=self.mission_takeoff_settle_tolerance_m,
            takeoff_required_stable_ticks=(
                self.mission_takeoff_required_stable_ticks
            ),
            takeoff_timeout_s=self.mission_takeoff_timeout_s,
            max_detection_age_s=self.mission_max_detection_age_s,
            required_detection_ticks=self.mission_required_detection_ticks,
            center_dwell_s=self.mission_center_dwell_s,
            center_clearance_required_s=self.mission_center_clearance_required_s,
            center_lost_detection_grace_ticks=(
                self.mission_center_lost_detection_grace_ticks
            ),
            seek_yaw_rate_rad_s=self.mission_seek_yaw_rate_rad_s,
            gate_pass_distance_m=self.mission_gate_pass_distance_m,
            gate_pass_speed_m_s=self.mission_gate_pass_speed_m_s,
            next_gate_acquire_speed_m_s=self.mission_next_gate_acquire_speed_m_s,
            next_gate_acquire_min_clear_distance_m=(
                self.mission_next_gate_acquire_min_clear_distance_m
            ),
            next_gate_acquire_min_area_ratio=(
                self.mission_next_gate_acquire_min_area_ratio
            ),
            gate_ready_area_ratio=self.mission_gate_ready_area_ratio,
            next_gate_acquire_max_distance_m=(
                self.mission_next_gate_acquire_max_distance_m
            ),
            next_gate_acquire_timeout_s=self.mission_next_gate_acquire_timeout_s,
            brake_settle_s=self.mission_brake_settle_s,
            brake_ramp_s=self.mission_brake_ramp_s,
            brake_altitude_hold_enabled=self.mission_brake_altitude_hold_enabled,
            final_exit_distance_m=self.mission_final_exit_distance_m,
            final_exit_speed_m_s=self.mission_final_exit_speed_m_s,
            min_centering_altitude_m=self.mission_min_centering_altitude_m,
            max_centering_altitude_m=self.mission_max_centering_altitude_m,
            altitude_hold_enabled=self.mission_altitude_hold_enabled,
            altitude_hold_deadband_m=self.mission_altitude_hold_deadband_m,
            altitude_hold_kp=self.mission_altitude_hold_kp,
            altitude_hold_max_climb_m_s=(
                self.mission_altitude_hold_max_climb_m_s
            ),
            altitude_hold_max_descent_m_s=(
                self.mission_altitude_hold_max_descent_m_s
            ),
            landing_complete_altitude_m=self.mission_landing_complete_altitude_m,
            mission_timeout_s=self.mission_timeout_s,
        )

    def visual_servo_config(self) -> VisualServoConfig:
        """Build visual-servo policy from operator runtime settings."""

        return VisualServoConfig(
            frame_width_px=self.visual_frame_width_px,
            frame_height_px=self.visual_frame_height_px,
            min_confidence=self.visual_min_confidence,
            filter_alpha=self.visual_filter_alpha,
            command_filter_alpha=self.visual_command_filter_alpha,
            center_deadband_x=self.visual_center_deadband_x,
            center_deadband_y=self.visual_center_deadband_y,
            aligned_error_x=self.visual_aligned_error_x,
            aligned_error_y=self.visual_aligned_error_y,
            pass_target_offset_x=self.visual_pass_target_offset_x,
            pass_target_offset_y=self.visual_pass_target_offset_y,
            pass_clearance_left_error=self.visual_pass_clearance_left_error,
            pass_clearance_right_error=self.visual_pass_clearance_right_error,
            pass_clearance_up_error=self.visual_pass_clearance_up_error,
            pass_clearance_down_error=self.visual_pass_clearance_down_error,
            max_error_for_forward=self.visual_max_error_for_forward,
            min_forward_speed_m_s=self.visual_min_forward_speed_m_s,
            max_forward_speed_m_s=self.visual_max_forward_speed_m_s,
            lateral_kp=self.visual_lateral_kp,
            vertical_kp=self.visual_vertical_kp,
            yaw_kp=self.visual_yaw_kp,
            max_lateral_speed_m_s=self.visual_max_lateral_speed_m_s,
            max_vertical_speed_m_s=self.visual_max_vertical_speed_m_s,
            max_yaw_rate_rad_s=self.visual_max_yaw_rate_rad_s,
        )

    def target_selector_config(self) -> GateTargetSelectorConfig:
        """Build gate-selector policy from operator runtime settings."""

        return GateTargetSelectorConfig(
            min_seek_confidence=self.gate_selector_min_seek_confidence,
            min_track_confidence=self.gate_selector_min_track_confidence,
            min_area_ratio=self.gate_selector_min_area_ratio,
            min_aspect_ratio=self.gate_selector_min_aspect_ratio,
            max_aspect_ratio=self.gate_selector_max_aspect_ratio,
            min_appearance_score=self.gate_selector_min_appearance_score,
            appearance_weight=self.gate_selector_appearance_weight,
            stable_window_frames=self.gate_selector_stable_window_frames,
            required_stable_frames=self.gate_selector_required_stable_frames,
        )
