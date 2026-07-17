from __future__ import annotations

import numpy as np


class JointCommandShaper:
    """Time-based low-pass plus velocity/acceleration/jerk/step limits."""

    def __init__(
        self,
        initial_q: np.ndarray,
        joint_limits: np.ndarray,
        *,
        lowpass_hz: float,
        max_velocity: np.ndarray,
        max_acceleration: np.ndarray,
        max_jerk: np.ndarray,
        max_step: np.ndarray,
    ) -> None:
        self.q_command = np.asarray(initial_q, dtype=float).reshape(6).copy()
        self.q_filtered = self.q_command.copy()
        self.velocity = np.zeros(6)
        self.acceleration = np.zeros(6)
        self.joint_limits = np.asarray(joint_limits, dtype=float).reshape(6, 2).copy()
        self.lowpass_hz = float(lowpass_hz)
        self.max_velocity = np.asarray(max_velocity, dtype=float).reshape(6)
        self.max_acceleration = np.asarray(max_acceleration, dtype=float).reshape(6)
        self.max_jerk = np.asarray(max_jerk, dtype=float).reshape(6)
        self.max_step = np.asarray(max_step, dtype=float).reshape(6)
        self.last_limiter_flags: dict[str, bool] = {}

    @classmethod
    def from_config(cls, initial_q: np.ndarray, joint_limits: np.ndarray, config: dict) -> "JointCommandShaper":
        control = config["control"]
        return cls(
            initial_q,
            joint_limits,
            lowpass_hz=control["joint_target_lowpass_hz"],
            max_velocity=control["max_joint_velocity_rad_s"],
            max_acceleration=control["max_joint_acceleration_rad_s2"],
            max_jerk=control["max_joint_jerk_rad_s3"],
            max_step=control["max_joint_step_rad"],
        )

    def step(self, q_desired: np.ndarray, dt: float) -> np.ndarray:
        if dt <= 0.0:
            raise ValueError("dt must be positive")
        desired = np.clip(
            np.asarray(q_desired, dtype=float).reshape(6),
            self.joint_limits[:, 0],
            self.joint_limits[:, 1],
        )
        alpha = 1.0 if self.lowpass_hz <= 0.0 else 1.0 - np.exp(-2.0 * np.pi * self.lowpass_hz * dt)
        self.q_filtered += alpha * (desired - self.q_filtered)

        raw_velocity = (self.q_filtered - self.q_command) / dt
        velocity_target = np.clip(raw_velocity, -self.max_velocity, self.max_velocity)
        raw_acceleration = (velocity_target - self.velocity) / dt
        acceleration_target = np.clip(raw_acceleration, -self.max_acceleration, self.max_acceleration)
        raw_jerk = (acceleration_target - self.acceleration) / dt
        jerk = np.clip(raw_jerk, -self.max_jerk, self.max_jerk)

        self.acceleration = np.clip(
            self.acceleration + jerk * dt,
            -self.max_acceleration,
            self.max_acceleration,
        )
        self.velocity = np.clip(
            self.velocity + self.acceleration * dt,
            -self.max_velocity,
            self.max_velocity,
        )
        raw_step = self.velocity * dt
        joint_step = np.clip(raw_step, -self.max_step, self.max_step)
        next_q = np.clip(
            self.q_command + joint_step,
            self.joint_limits[:, 0],
            self.joint_limits[:, 1],
        )

        self.last_limiter_flags = {
            "joint_limit": not np.allclose(next_q, self.q_command + joint_step),
            "velocity": not np.allclose(velocity_target, raw_velocity),
            "acceleration": not np.allclose(acceleration_target, raw_acceleration),
            "jerk": not np.allclose(jerk, raw_jerk),
            "step": not np.allclose(joint_step, raw_step),
        }
        self.q_command = next_q
        return next_q.copy()

