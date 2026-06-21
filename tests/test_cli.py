from drone_autonomy.cli import build_parser


def test_cli_parser_defaults_are_available_without_runtime_adapters() -> None:
    parser = build_parser()

    args = parser.parse_args([])

    assert args.connection == "udp:127.0.0.1:14551"
    assert args.baud == 115200
    assert args.webots_camera_encoding == "rgb24"
    assert args.gate_class_names == "Goals-Detection"
    assert args.gate_class_ids == "3"
    assert args.yolo_imgsz == 640
    assert args.gate_selector_min_appearance_score == 0.0
    assert args.gate_selector_appearance_weight == 0.0
    assert args.mission_max_detection_age == 0.75
    assert args.mission_required_detection_ticks == 2
    assert args.mission_center_lost_grace_ticks == 10
    assert args.mission_brake_settle == 1.0
    assert args.mission_brake_ramp == 0.7
    assert args.mission_brake_altitude_hold is False
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
