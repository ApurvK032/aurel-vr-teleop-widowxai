from __future__ import annotations

import numpy as np
from ruckig import InputParameter, OutputParameter, Ruckig


class JointCommandShaper:
    """Online jerk-limited joint trajectory generation using Ruckig."""

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
        control_cycle_s: float = 0.01,
    ) -> None:
        self.control_cycle_s = float(control_cycle_s)
        if self.control_cycle_s <= 0.0:
            raise ValueError("control_cycle_s must be positive")

        self.joint_limits = np.asarray(joint_limits, dtype=float).reshape(6, 2).copy()
        self.lowpass_hz = float(lowpass_hz)
        self.max_velocity = np.asarray(max_velocity, dtype=float).reshape(6)
        self.max_acceleration = np.asarray(max_acceleration, dtype=float).reshape(6)
        self.max_jerk = np.asarray(max_jerk, dtype=float).reshape(6)
        self.max_step = np.asarray(max_step, dtype=float).reshape(6)
        self.effective_max_velocity = np.minimum(
            self.max_velocity,
            self.max_step / self.control_cycle_s,
        )

        self._otg = Ruckig(6, self.control_cycle_s)
        self._input = InputParameter(6)
        self._output = OutputParameter(6)
        self.q_command = np.zeros(6)
        self.q_filtered = np.zeros(6)
        self.velocity = np.zeros(6)
        self.acceleration = np.zeros(6)
        self.last_limiter_flags: dict[str, bool] = {}

        self._input.min_position = self.joint_limits[:, 0].tolist()
        self._input.max_position = self.joint_limits[:, 1].tolist()
        self._input.max_velocity = self.effective_max_velocity.tolist()
        self._input.max_acceleration = self.max_acceleration.tolist()
        self._input.max_jerk = self.max_jerk.tolist()
        self.hold(initial_q)

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
            control_cycle_s=1.0 / float(control["command_rate_hz"]),
        )

    def hold(self, q_hold: np.ndarray | None = None) -> np.ndarray:
        """Stop trajectory advancement and hold a zero-motion joint state."""
        q = self.q_command if q_hold is None else np.asarray(q_hold, dtype=float).reshape(6)
        q = np.clip(q, self.joint_limits[:, 0], self.joint_limits[:, 1]).copy()
        zeros = np.zeros(6)
        self.q_command = q
        self.q_filtered = q.copy()
        self.velocity = zeros.copy()
        self.acceleration = zeros.copy()
        self._input.current_position = q.tolist()
        self._input.current_velocity = zeros.tolist()
        self._input.current_acceleration = zeros.tolist()
        self._input.target_position = q.tolist()
        self._input.target_velocity = zeros.tolist()
        self._input.target_acceleration = zeros.tolist()
        self.last_limiter_flags = {
            "joint_limit": False,
            "velocity": False,
            "acceleration": False,
            "jerk": False,
            "step": False,
        }
        return q.copy()

    def step(self, q_desired: np.ndarray, dt: float) -> np.ndarray:
        if not np.isclose(dt, self.control_cycle_s, rtol=0.0, atol=1e-12):
            raise ValueError(f"dt must match the configured {self.control_cycle_s:g} s control cycle")

        requested = np.asarray(q_desired, dtype=float).reshape(6)
        desired = np.clip(requested, self.joint_limits[:, 0], self.joint_limits[:, 1])
        alpha = 1.0 if self.lowpass_hz <= 0.0 else 1.0 - np.exp(-2.0 * np.pi * self.lowpass_hz * dt)
        self.q_filtered += alpha * (desired - self.q_filtered)
        self._input.target_position = self.q_filtered.tolist()

        previous_q = self.q_command.copy()
        previous_acceleration = self.acceleration.copy()
        result = self._otg.update(self._input, self._output)
        if int(result) < 0:
            raise RuntimeError(f"Ruckig trajectory generation failed: {result}")

        next_q = np.asarray(self._output.new_position, dtype=float)
        next_velocity = np.asarray(self._output.new_velocity, dtype=float)
        next_acceleration = np.asarray(self._output.new_acceleration, dtype=float)
        joint_step = next_q - previous_q
        if np.any(np.abs(joint_step) > self.max_step + 1e-10):
            raise RuntimeError("Ruckig output exceeded the configured per-cycle joint step")
        if np.any(next_q < self.joint_limits[:, 0] - 1e-10) or np.any(
            next_q > self.joint_limits[:, 1] + 1e-10
        ):
            raise RuntimeError("Ruckig output exceeded the official model joint limits")

        jerk = (next_acceleration - previous_acceleration) / dt
        tolerance = 1e-7
        self.last_limiter_flags = {
            "joint_limit": not np.array_equal(desired, requested),
            "velocity": bool(np.any(np.abs(next_velocity) >= self.effective_max_velocity - tolerance)),
            "acceleration": bool(np.any(np.abs(next_acceleration) >= self.max_acceleration - tolerance)),
            "jerk": bool(np.any(np.abs(jerk) >= self.max_jerk - tolerance)),
            "step": bool(np.any(np.abs(joint_step) >= self.max_step - tolerance)),
        }
        self.q_command = next_q
        self.velocity = next_velocity
        self.acceleration = next_acceleration
        self._output.pass_to_input(self._input)
        return next_q.copy()
