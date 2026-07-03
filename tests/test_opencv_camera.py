import pytest

from drone_autonomy.perception.opencv_camera import (
    OpenCvCameraConfig,
    _parse_opencv_source,
)


def test_opencv_camera_config_defaults_for_c920_shape() -> None:
    config = OpenCvCameraConfig()

    assert config.source == "0"
    assert config.backend == "default"
    assert config.width_px == 640
    assert config.height_px == 480
    assert config.fps == 30.0


def test_opencv_camera_source_parsing_keeps_device_paths() -> None:
    assert _parse_opencv_source("0") == 0
    assert _parse_opencv_source("/dev/video2") == "/dev/video2"


def test_opencv_camera_rejects_invalid_backend() -> None:
    with pytest.raises(ValueError, match="backend"):
        OpenCvCameraConfig(backend="gstreamer")
