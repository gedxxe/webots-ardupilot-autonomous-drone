#!/usr/bin/env python3
"""Small MAVLink heartbeat connection check."""

from drone_autonomy.cli import main


if __name__ == "__main__":
    raise SystemExit(main(["--mode", "heartbeat"]))
