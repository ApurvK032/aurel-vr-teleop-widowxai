from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .math3d import normalize_quat


@dataclass(frozen=True)
class Pose:
    position: np.ndarray
    quaternion_wxyz: np.ndarray

    def __post_init__(self) -> None:
        object.__setattr__(self, "position", np.asarray(self.position, dtype=float).reshape(3).copy())
        object.__setattr__(self, "quaternion_wxyz", normalize_quat(self.quaternion_wxyz))


@dataclass(frozen=True)
class QuestSample:
    sequence: int
    capture_monotonic_ms: float
    capture_epoch_ms: float
    send_monotonic_ms: float
    pc_arrival_monotonic_ns: int
    pc_arrival_epoch_ns: int
    reconnect_generation: int
    controller_pose: Pose
    grip: float
    trigger: float
    hand: str
    mapping_mode: str
    head_quaternion_wxyz: np.ndarray | None = None
