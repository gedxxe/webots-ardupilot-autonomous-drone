from pathlib import Path
from os.path import expandvars

from drone_autonomy.config import load_simulator_config, parse_env_file


def test_parse_env_file_strips_quotes_and_comments(tmp_path: Path) -> None:
    env_file = tmp_path / "sitl.env"
    env_file.write_text(
        """
        # comment
        ARDUPILOT_HOME="$HOME/ardupilot"
        MAVLINK_OUT='udp:127.0.0.1:14550'
        """,
        encoding="utf-8",
    )

    values = parse_env_file(env_file)

    assert values["ARDUPILOT_HOME"] == "$HOME/ardupilot"
    assert values["MAVLINK_OUT"] == "udp:127.0.0.1:14550"


def test_load_simulator_config_defaults(tmp_path: Path) -> None:
    env_file = tmp_path / "sitl.env"
    env_file.write_text('ARDUPILOT_HOME="$HOME/ardupilot"\n', encoding="utf-8")

    config = load_simulator_config(env_file)

    assert config.ardupilot_home == Path(expandvars("$HOME/ardupilot"))
    assert config.mavlink_out == "udp:127.0.0.1:14550"
    assert config.ardupilot_vehicle == "ArduCopter"
    assert config.ardupilot_model == "webots-python"
