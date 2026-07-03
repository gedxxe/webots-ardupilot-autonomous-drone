from __future__ import annotations

import json
from pathlib import Path
from time import time
from typing import TextIO


class RuntimeJsonlLogger:
    """Optional append-only JSONL logger for flight/debug telemetry.

    Logging is intentionally best-effort. A bad path must not stop the mission
    loop; the runtime prints one warning and continues without JSONL output.
    """

    def __init__(self, path: str) -> None:
        self.path = path
        self._file: TextIO | None = None
        self._disabled = False

    @property
    def enabled(self) -> bool:
        """Return whether logging is configured and not disabled by an error."""

        return bool(self.path) and not self._disabled

    def write(self, event: str, **payload: object) -> None:
        """Append one JSON record if logging is enabled."""

        if not self.enabled:
            return

        try:
            if self._file is None:
                output_path = Path(self.path).expanduser()
                output_path.parent.mkdir(parents=True, exist_ok=True)
                self._file = output_path.open("a", encoding="utf-8", buffering=1)
            record = {"event": event, "wall_time_s": time(), **payload}
            self._file.write(json.dumps(record, sort_keys=True, default=str) + "\n")
        except OSError as exc:
            print(f"warning: runtime JSONL logging disabled: {exc}")
            self._disabled = True

    def close(self) -> None:
        """Close the log file if it was opened."""

        if self._file is None:
            return
        try:
            self._file.close()
        finally:
            self._file = None

