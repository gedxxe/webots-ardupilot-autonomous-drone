from __future__ import annotations

from dataclasses import dataclass

from drone_autonomy.perception.yolo_profile import (
    DEFAULT_GATE_CLASS_IDS,
    DEFAULT_GATE_CLASS_NAMES,
)


@dataclass(frozen=True)
class AutonomyRuntimeConfig:
    """Configuration for the process-level autonomy loop.

    Defaults are intentionally dry-run oriented. Launch scripts and local env
    files select the active detector/profile; this object remains adapter-free
    so CLI defaults and `--help` do not require MAVLink imports.
    """

    connection: str = "udp:127.0.0.1:14551"
    loop_hz: float = 20.0
    max_runtime_s: float = 180.0
    heartbeat_timeout_s: float = 30.0
    status_interval_s: float = 1.0
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
    yolo_model_path: str = ""
    yolo_confidence: float = 0.35
    yolo_image_size_px: int = 640
    yolo_device: str = "cpu"
    yolo_gate_class_names: tuple[str, ...] = DEFAULT_GATE_CLASS_NAMES
    yolo_gate_class_ids: tuple[int, ...] = DEFAULT_GATE_CLASS_IDS
    gate_selector_min_seek_confidence: float = 0.40
    gate_selector_min_track_confidence: float = 0.30
    gate_selector_min_area_ratio: float = 0.0015
    gate_selector_min_aspect_ratio: float = 0.35
    gate_selector_max_aspect_ratio: float = 4.0
    gate_selector_min_appearance_score: float = 0.00
    gate_selector_appearance_weight: float = 0.00
    gate_selector_stable_window_frames: int = 5
    gate_selector_required_stable_frames: int = 3
    mission_max_detection_age_s: float = 0.75
    mission_required_detection_ticks: int = 2
    mission_center_dwell_s: float = 5.0
    mission_center_clearance_required_s: float = 0.30
    mission_center_lost_detection_grace_ticks: int = 10
    mission_seek_yaw_rate_rad_s: float = 0.30
    mission_gate_pass_distance_m: float = 1.25
    mission_gate_pass_speed_m_s: float = 1.20
    mission_next_gate_acquire_speed_m_s: float = 1.50
    mission_next_gate_acquire_min_clear_distance_m: float = 2.00
    mission_next_gate_acquire_min_area_ratio: float = 0.015
    mission_gate_ready_area_ratio: float = 0.060
    mission_next_gate_acquire_max_distance_m: float = 6.00
    mission_next_gate_acquire_timeout_s: float = 6.00
    mission_brake_settle_s: float = 1.00
    mission_brake_ramp_s: float = 0.70
    mission_brake_altitude_hold_enabled: bool = False
    mission_final_exit_distance_m: float = 2.00
    mission_final_exit_speed_m_s: float = 1.50
    visual_frame_width_px: int = 640
    visual_frame_height_px: int = 480
    visual_min_confidence: float = 0.45
    visual_filter_alpha: float = 0.18
    visual_command_filter_alpha: float = 0.25
    visual_center_deadband_x: float = 0.070
    visual_center_deadband_y: float = 0.090
    visual_aligned_error_x: float = 0.075
    visual_aligned_error_y: float = 0.120
    visual_pass_target_offset_x: float = 0.0
    visual_pass_target_offset_y: float = 0.0
    visual_pass_clearance_left_error: float = 0.090
    visual_pass_clearance_right_error: float = 0.090
    visual_pass_clearance_up_error: float = 0.130
    visual_pass_clearance_down_error: float = 0.130
    visual_max_error_for_forward: float = 0.450
    visual_min_forward_speed_m_s: float = 0.00
    visual_max_forward_speed_m_s: float = 0.30
    visual_lateral_kp: float = 0.35
    visual_vertical_kp: float = 0.28
    visual_yaw_kp: float = 0.18
    visual_max_lateral_speed_m_s: float = 0.22
    visual_max_vertical_speed_m_s: float = 0.16
    visual_max_yaw_rate_rad_s: float = 0.18
