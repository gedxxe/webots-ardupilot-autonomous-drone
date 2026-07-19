from drone_autonomy.cli import build_parser


def test_cli_parser_defaults_are_available_without_runtime_adapters() -> None:
    parser = build_parser()

    args = parser.parse_args([])

    assert args.connection == "udp:127.0.0.1:14551"
    assert args.baud == 115200
    assert args.mavlink_heartbeat_stale == 3.0
    assert args.mavlink_local_position_stale == 1.0
    assert args.command_ack_required is True
    assert args.command_ack_timeout == 1.0
    assert args.command_ack_max_retries == 2
    assert args.log_jsonl == ""
    assert args.webots_camera_encoding == "rgb24"
    assert args.opencv_camera_source == "0"
    assert args.opencv_camera_backend == "default"
    assert args.opencv_camera_width == 640
    assert args.opencv_camera_height == 480
    assert args.opencv_camera_fps == 30.0
    assert args.opencv_detection_stale == 0.75
    assert args.gate_class_names == "Goals-Detection"
    assert args.gate_class_ids == "3"
    assert args.yolo_imgsz == 640
    assert args.gate_selector_min_appearance_score == 0.0
    assert args.gate_selector_appearance_weight == 0.0
    assert args.mission_takeoff_altitude == 1.0
    assert args.mission_takeoff_settle_tolerance == 0.06
    assert args.mission_takeoff_stable_ticks == 8
    assert args.mission_takeoff_timeout == 20.0
    assert args.mission_max_detection_age == 0.75
    assert args.mission_required_detection_ticks == 2
    assert args.mission_center_lost_grace_ticks == 10
    assert args.mission_brake_settle == 1.0
    assert args.mission_brake_ramp == 0.7
    assert args.mission_brake_altitude_hold is False
    assert args.mission_min_centering_altitude == 0.65
    assert args.mission_max_centering_altitude == 2.0
    assert args.mission_altitude_hold_enabled is True
    assert args.mission_altitude_hold_deadband == 0.08
    assert args.mission_landing_complete_altitude == 0.15
    assert args.mission_timeout == 180.0
    assert args.visual_frame_width == 640
    assert args.visual_frame_height == 480
    assert args.visual_max_error_for_forward == 0.45


def test_cli_help_distinguishes_yolo_imgsz_from_camera_resolution() -> None:
    help_text = build_parser().format_help()

    assert "YOLO inference/letterbox size" in help_text
    assert "width/height" in help_text


def test_cli_parser_accepts_hardware_serial_baud() -> None:
    parser = build_parser()

    args = parser.parse_args(["--connection", "/dev/ttyACM1", "--baud", "921600"])

    assert args.connection == "/dev/ttyACM1"
    assert args.baud == 921600


def test_cli_parser_accepts_command_ack_policy() -> None:
    parser = build_parser()

    args = parser.parse_args(
        [
            "--no-command-ack-required",
            "--command-ack-timeout",
            "2.5",
            "--command-ack-max-retries",
            "4",
        ]
    )

    assert args.command_ack_required is False
    assert args.command_ack_timeout == 2.5
    assert args.command_ack_max_retries == 4


def test_cli_parser_accepts_jsonl_log_path() -> None:
    parser = build_parser()

    args = parser.parse_args(["--log-jsonl", "logs/autonomy.jsonl"])

    assert args.log_jsonl == "logs/autonomy.jsonl"


def test_cli_parser_can_disable_companion_altitude_hold() -> None:
    args = build_parser().parse_args(["--no-mission-altitude-hold-enabled"])

    assert args.mission_altitude_hold_enabled is False


def test_cli_parser_accepts_opencv_yolo_detector_settings() -> None:
    parser = build_parser()

    args = parser.parse_args(
        [
            "--detector",
            "opencv-yolo",
            "--opencv-camera-source",
            "/dev/video2",
            "--opencv-camera-backend",
            "v4l2",
            "--opencv-camera-width",
            "1280",
            "--opencv-camera-height",
            "720",
            "--opencv-camera-fps",
            "30",
        ]
    )

    assert args.detector == "opencv-yolo"
    assert args.opencv_camera_source == "/dev/video2"
    assert args.opencv_camera_backend == "v4l2"
    assert args.opencv_camera_width == 1280
    assert args.opencv_camera_height == 720
    assert args.opencv_camera_fps == 30.0
