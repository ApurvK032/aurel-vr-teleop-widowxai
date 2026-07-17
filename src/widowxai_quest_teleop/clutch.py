from __future__ import annotations

import numpy as np

from .mapping import ClutchPoseMapper
from .types import Pose


class ClutchController:
    """Grip-edge state machine that requires release after stale/reconnect."""

    def __init__(self, mapper: ClutchPoseMapper, grip_threshold: float = 0.65) -> None:
        self.mapper = mapper
        self.grip_threshold = float(grip_threshold)
        self._was_pressed = False
        self._blocked_until_release = False

    def force_reanchor(self) -> None:
        self.mapper.release()
        self._blocked_until_release = True

    def update(
        self,
        *,
        grip: float,
        stream_fresh: bool,
        controller_pose: Pose,
        robot_pose: Pose,
        wrist_pivot: np.ndarray,
    ) -> Pose | None:
        pressed = float(grip) >= self.grip_threshold
        if not stream_fresh:
            self.force_reanchor()
            self._was_pressed = pressed
            return None
        if not pressed:
            self._blocked_until_release = False
            if self.mapper.engaged:
                self.mapper.release()
            self._was_pressed = False
            return None
        if self._blocked_until_release:
            self._was_pressed = True
            return None
        if not self._was_pressed:
            self.mapper.engage(controller_pose, robot_pose, wrist_pivot)
        self._was_pressed = True
        return self.mapper.update(controller_pose, robot_pose)

