from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .math3d import clip_norm, orientation_error_world, quat_to_matrix
from .model import WidowXAIModel
from .types import Pose


@dataclass(frozen=True)
class IKDiagnostics:
    position_residual_m: float
    orientation_residual_rad: float
    position_manipulability: float
    rotation_manipulability: float
    position_damping: float
    rotation_damping: float
    step_norm_rad: float
    iterations: int
    minimum_joint_limit_margin_rad: float
    joint_limit_clipped: bool
    wrist_parked: bool
    status: str


class DecoupledIK:
    """Sequential 3+3 differential IK for the WidowXAI.

    Joints 0-2 move the joint-3 wrist anchor. Joints 3-5 then track tool
    orientation. Each subproblem has independent manipulability-adaptive
    damping and every step is bounded before it reaches a command backend.
    """

    def __init__(
        self,
        model: WidowXAIModel,
        *,
        rest_q: np.ndarray,
        lambda_position: float = 0.05,
        lambda_position_extra: float = 0.15,
        position_manipulability_threshold: float = 0.05,
        rest_bias: float = 0.02,
        lambda_rotation: float = 0.05,
        lambda_rotation_extra: float = 0.40,
        rotation_manipulability_threshold: float = 0.50,
        orientation_hold_error_rad: float = 2.20,
        max_position_error_step_m: float = 0.025,
        max_rotation_error_step_rad: float = 0.12,
        max_joint_step_rad: np.ndarray | None = None,
        iterations_per_tick: int = 2,
    ) -> None:
        self.model = model
        self.rest_q = np.asarray(rest_q, dtype=float).reshape(6).copy()
        self.lambda_position = float(lambda_position)
        self.lambda_position_extra = float(lambda_position_extra)
        self.position_manipulability_threshold = float(position_manipulability_threshold)
        self.rest_bias = float(rest_bias)
        self.lambda_rotation = float(lambda_rotation)
        self.lambda_rotation_extra = float(lambda_rotation_extra)
        self.rotation_manipulability_threshold = float(rotation_manipulability_threshold)
        self.orientation_hold_error_rad = float(orientation_hold_error_rad)
        self.max_position_error_step_m = float(max_position_error_step_m)
        self.max_rotation_error_step_rad = float(max_rotation_error_step_rad)
        self.max_joint_step_rad = (
            np.full(6, np.inf)
            if max_joint_step_rad is None
            else np.asarray(max_joint_step_rad, dtype=float).reshape(6).copy()
        )
        self.iterations_per_tick = max(1, int(iterations_per_tick))

    @classmethod
    def from_config(cls, model: WidowXAIModel, config: dict) -> "DecoupledIK":
        ik = config["ik"]
        control = config["control"]
        rest_q = config["model"]["simulation_start_q_rad"]
        return cls(
            model,
            rest_q=np.asarray(rest_q, dtype=float),
            lambda_position=ik["lambda_position"],
            lambda_position_extra=ik["lambda_position_extra"],
            position_manipulability_threshold=ik["position_manipulability_threshold"],
            rest_bias=ik["rest_bias"],
            lambda_rotation=ik["lambda_rotation"],
            lambda_rotation_extra=ik["lambda_rotation_extra"],
            rotation_manipulability_threshold=ik["rotation_manipulability_threshold"],
            orientation_hold_error_rad=ik["orientation_hold_error_rad"],
            max_position_error_step_m=ik["max_position_error_step_m"],
            max_rotation_error_step_rad=ik["max_rotation_error_step_rad"],
            max_joint_step_rad=np.asarray(control["max_joint_step_rad"], dtype=float),
            iterations_per_tick=ik["iterations_per_tick"],
        )

    @staticmethod
    def _adaptive_damping(base: float, extra: float, manipulability: float, threshold: float) -> float:
        ramp = max(0.0, 1.0 - manipulability / max(threshold, 1e-12))
        return float(np.sqrt(base * base + extra * extra * ramp * ramp))

    @staticmethod
    def _dls(jacobian: np.ndarray, error: np.ndarray, damping: float) -> np.ndarray:
        task_matrix = jacobian @ jacobian.T + damping * damping * np.eye(jacobian.shape[0])
        return jacobian.T @ np.linalg.solve(task_matrix, error)

    def solve(
        self,
        target: Pose,
        q_seed: np.ndarray,
        *,
        iterations: int | None = None,
    ) -> tuple[np.ndarray, IKDiagnostics]:
        q_initial = self.model.clamp_joints(q_seed)
        q = q_initial.copy()
        count = self.iterations_per_tick if iterations is None else max(1, int(iterations))
        target_rotation = quat_to_matrix(target.quaternion_wxyz)
        limit_clipped = False
        wrist_parked = False
        status = "ok"
        position_manipulability = 0.0
        rotation_manipulability = 0.0
        position_damping = self.lambda_position
        rotation_damping = self.lambda_rotation

        try:
            for _ in range(count):
                current_ee, current_wrist = self.model.fk(q)
                current_rotation = quat_to_matrix(current_ee.quaternion_wxyz)
                ee_to_wrist_local = current_rotation.T @ (current_wrist.position - current_ee.position)
                target_wrist = target.position + target_rotation @ ee_to_wrist_local
                position_error = clip_norm(
                    target_wrist - current_wrist.position,
                    self.max_position_error_step_m,
                )

                jacobian_position, _ = self.model.jacobian("wrist")
                arm_jacobian = jacobian_position[:, :3]
                position_manipulability = abs(float(np.linalg.det(arm_jacobian)))
                position_damping = self._adaptive_damping(
                    self.lambda_position,
                    self.lambda_position_extra,
                    position_manipulability,
                    self.position_manipulability_threshold,
                )
                mu2 = self.rest_bias * self.rest_bias
                normal = arm_jacobian.T @ arm_jacobian
                normal += (position_damping * position_damping + mu2) * np.eye(3)
                rhs = arm_jacobian.T @ position_error + mu2 * (self.rest_q[:3] - q[:3])
                arm_step = np.linalg.solve(normal, rhs)
                arm_step = np.clip(arm_step, -self.max_joint_step_rad[:3], self.max_joint_step_rad[:3])
                q[:3] += arm_step
                unclamped = q.copy()
                q = self.model.clamp_joints(q)
                limit_clipped |= not np.allclose(q, unclamped)

                current_ee, _ = self.model.fk(q)
                rotation_error = orientation_error_world(target.quaternion_wxyz, current_ee.quaternion_wxyz)
                wrist_parked = float(np.linalg.norm(rotation_error)) > self.orientation_hold_error_rad
                _, jacobian_rotation = self.model.jacobian("ee")
                wrist_jacobian = jacobian_rotation[:, 3:6]
                rotation_manipulability = abs(float(np.linalg.det(wrist_jacobian)))
                rotation_damping = self._adaptive_damping(
                    self.lambda_rotation,
                    self.lambda_rotation_extra,
                    rotation_manipulability,
                    self.rotation_manipulability_threshold,
                )
                if not wrist_parked:
                    rotation_error = clip_norm(rotation_error, self.max_rotation_error_step_rad)
                    wrist_step = self._dls(wrist_jacobian, rotation_error, rotation_damping)
                    wrist_step = np.clip(
                        wrist_step,
                        -self.max_joint_step_rad[3:6],
                        self.max_joint_step_rad[3:6],
                    )
                    q[3:6] += wrist_step
                    unclamped = q.copy()
                    q = self.model.clamp_joints(q)
                    limit_clipped |= not np.allclose(q, unclamped)
                else:
                    status = "orientation_parked"
        except (np.linalg.LinAlgError, FloatingPointError, ValueError):
            q = q_initial.copy()
            status = "numerical_failure"

        if not np.all(np.isfinite(q)):
            q = q_initial.copy()
            status = "nonfinite_rejected"

        current_ee, current_wrist = self.model.fk(q)
        current_rotation = quat_to_matrix(current_ee.quaternion_wxyz)
        ee_to_wrist_local = current_rotation.T @ (current_wrist.position - current_ee.position)
        final_target_wrist = target.position + target_rotation @ ee_to_wrist_local
        position_residual = float(np.linalg.norm(final_target_wrist - current_wrist.position))
        orientation_residual = float(
            np.linalg.norm(orientation_error_world(target.quaternion_wxyz, current_ee.quaternion_wxyz))
        )
        margins = np.minimum(q - self.model.joint_limits[:, 0], self.model.joint_limits[:, 1] - q)
        diagnostics = IKDiagnostics(
            position_residual_m=position_residual,
            orientation_residual_rad=orientation_residual,
            position_manipulability=position_manipulability,
            rotation_manipulability=rotation_manipulability,
            position_damping=position_damping,
            rotation_damping=rotation_damping,
            step_norm_rad=float(np.linalg.norm(q - q_initial)),
            iterations=count,
            minimum_joint_limit_margin_rad=float(np.min(margins)),
            joint_limit_clipped=limit_clipped,
            wrist_parked=wrist_parked,
            status=status,
        )
        return q, diagnostics

