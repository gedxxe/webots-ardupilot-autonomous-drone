from __future__ import annotations

from dataclasses import dataclass
from os.path import expandvars
from pathlib import Path


@dataclass(frozen=True)
class SimulatorConfig:
    ardupilot_home: Path
    mavlink_out: str = "udp:127.0.0.1:14550"
    ardupilot_vehicle: str = "ArduCopter"
    ardupilot_model: str = "webots-python"


def parse_env_file(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    if not path.exists():
        return values

    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue

        key, value = line.split("=", 1)
        values[key.strip()] = value.strip().strip("\"'")

    return values


def load_simulator_config(path: Path) -> SimulatorConfig:
    values = parse_env_file(path)
    return SimulatorConfig(
        ardupilot_home=Path(expandvars(values["ARDUPILOT_HOME"])).expanduser(),
        mavlink_out=values.get("MAVLINK_OUT", "udp:127.0.0.1:14550"),
        ardupilot_vehicle=values.get("ARDUPILOT_VEHICLE", "ArduCopter"),
        ardupilot_model=values.get("ARDUPILOT_MODEL", "webots-python"),
    )
