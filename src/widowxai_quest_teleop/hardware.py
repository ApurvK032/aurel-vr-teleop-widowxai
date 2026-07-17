from __future__ import annotations

from typing import Protocol

import numpy as np


class HardwareBackend(Protocol):
    def connect(self) -> None: ...
    def read_joint_positions(self) -> np.ndarray: ...
    def send_joint_positions(self, q_command: np.ndarray, goal_time_s: float) -> None: ...
    def safe_hold(self) -> None: ...
    def close(self) -> None: ...


class HardwareUnavailableError(RuntimeError):
    pass


class DisabledHardwareBackend:
    """Deliberate gate while the official arm driver is unavailable on Windows."""

    def connect(self) -> None:
        raise HardwareUnavailableError(
            "Physical-arm output is disabled. The official libtrossen_arm repository currently "
            "ships Linux/macOS libraries, not a Windows library. Keep using MuJoCo until a "
            "reviewed Windows driver bridge is installed and a hardware run is explicitly approved."
        )

