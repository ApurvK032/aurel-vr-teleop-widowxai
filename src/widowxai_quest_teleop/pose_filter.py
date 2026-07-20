from __future__ import annotations

import numpy as np

from .math3d import normalize_quat
from .types import Pose


def pose_ema(previous: Pose | None, current: Pose, alpha: float) -> Pose:
    """Reference-kit EMA: position blend plus hemisphere-safe quaternion nlerp."""
    if previous is None:
        return current
    weight = float(np.clip(alpha, 0.0, 1.0))
    position = (1.0 - weight) * previous.position + weight * current.position
    quaternion = current.quaternion_wxyz
    if float(np.dot(previous.quaternion_wxyz, quaternion)) < 0.0:
        quaternion = -quaternion
    quaternion = normalize_quat((1.0 - weight) * previous.quaternion_wxyz + weight * quaternion)
    return Pose(position, quaternion)
