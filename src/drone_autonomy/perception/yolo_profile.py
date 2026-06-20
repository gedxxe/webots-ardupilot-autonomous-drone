from __future__ import annotations

"""Shared YOLO profile constants for the current gate model.

Keep these values centralized so CLI defaults, runtime config, and detector
defaults do not drift apart when the model is retrained.
"""

DEFAULT_GATE_CLASS_NAMES: tuple[str, ...] = ("Goals-Detection",)
DEFAULT_GATE_CLASS_IDS: tuple[int, ...] = (3,)


def csv_names(values: tuple[str, ...] = DEFAULT_GATE_CLASS_NAMES) -> str:
    """Return a CLI/env friendly representation of accepted class names."""

    return ",".join(values)


def csv_ids(values: tuple[int, ...] = DEFAULT_GATE_CLASS_IDS) -> str:
    """Return a CLI/env friendly representation of accepted class ids."""

    return ",".join(str(value) for value in values)
