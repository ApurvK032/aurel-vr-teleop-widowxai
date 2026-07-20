from __future__ import annotations

import importlib
import platform
from dataclasses import dataclass
from types import ModuleType
from typing import Protocol

import numpy as np


class HardwareUnavailableError(RuntimeError):
    pass


class HardwareSafetyError(RuntimeError):
    pass


@dataclass(frozen=True)
class HardwareState:
    q_arm: np.ndarray
    gripper_position_m: float
    joint_limits: np.ndarray
    firmware_version: str

    def __post_init__(self) -> None:
        object.__setattr__(self, "q_arm", np.asarray(self.q_arm, dtype=float).reshape(6).copy())
        object.__setattr__(self, "joint_limits", np.asarray(self.joint_limits, dtype=float).reshape(7, 2).copy())


class HardwareBackend(Protocol):
    def connect(self) -> HardwareState: ...
    def enable_position_control(self) -> None: ...
    def read_state(self) -> HardwareState: ...
    def send_positions(self, q_command: np.ndarray, gripper_position_m: float) -> None: ...
    def safe_hold(self) -> None: ...
    def close(self) -> None: ...


class CommandGate:
    """Reject unsafe commands without filtering, buffering, or modifying them."""

    def __init__(
        self,
        initial_q: np.ndarray,
        initial_gripper_m: float,
        joint_limits: np.ndarray,
        max_joint_delta_rad: np.ndarray,
        *,
        joint_limit_margin_rad: float,
        gripper_limits_m: tuple[float, float],
        max_gripper_delta_m: float,
    ) -> None:
        self.previous_q = np.asarray(initial_q, dtype=float).reshape(6).copy()
        self.previous_gripper_m = float(initial_gripper_m)
        limits = np.asarray(joint_limits, dtype=float).reshape(7, 2)
        margin = max(0.0, float(joint_limit_margin_rad))
        self.lower = limits[:6, 0] + margin
        self.upper = limits[:6, 1] - margin
        if np.any(self.lower >= self.upper):
            raise HardwareSafetyError("joint-limit margin leaves no valid command range")
        self.max_joint_delta = np.asarray(max_joint_delta_rad, dtype=float).reshape(6).copy()
        self.gripper_lower_m, self.gripper_upper_m = map(float, gripper_limits_m)
        self.max_gripper_delta_m = float(max_gripper_delta_m)
        self.validate(self.previous_q, self.previous_gripper_m, update=False)

    def validate(self, q_command: np.ndarray, gripper_position_m: float, *, update: bool = True) -> None:
        q = np.asarray(q_command, dtype=float).reshape(6)
        gripper = float(gripper_position_m)
        if not np.all(np.isfinite(q)) or not np.isfinite(gripper):
            raise HardwareSafetyError("non-finite hardware command rejected")
        if np.any(q < self.lower) or np.any(q > self.upper):
            raise HardwareSafetyError("joint command outside the demo limit margin")
        if np.any(np.abs(q - self.previous_q) > self.max_joint_delta + 1e-12):
            raise HardwareSafetyError("joint command exceeded the configured per-tick cap")
        if not self.gripper_lower_m <= gripper <= self.gripper_upper_m:
            raise HardwareSafetyError("gripper command outside the demo range")
        if abs(gripper - self.previous_gripper_m) > self.max_gripper_delta_m + 1e-12:
            raise HardwareSafetyError("gripper command exceeded the configured per-tick cap")
        if update:
            self.previous_q = q.copy()
            self.previous_gripper_m = gripper


class DryRunBackend:
    """In-memory backend for exercising the complete demo without arm I/O."""

    def __init__(self, initial_q: np.ndarray, joint_limits: np.ndarray, gripper_position_m: float = 0.044) -> None:
        self.q_arm = np.asarray(initial_q, dtype=float).reshape(6).copy()
        arm_limits = np.asarray(joint_limits, dtype=float).reshape(6, 2)
        self.limits = np.vstack([arm_limits, np.array([0.0, 0.044])])
        self.gripper = float(gripper_position_m)
        self.enabled = False
        self.commands = 0

    def connect(self) -> HardwareState:
        return self.read_state()

    def enable_position_control(self) -> None:
        self.enabled = True

    def read_state(self) -> HardwareState:
        return HardwareState(self.q_arm, self.gripper, self.limits, "dry-run")

    def send_positions(self, q_command: np.ndarray, gripper_position_m: float) -> None:
        if not self.enabled:
            raise HardwareSafetyError("dry-run position control is not enabled")
        self.q_arm = np.asarray(q_command, dtype=float).reshape(6).copy()
        self.gripper = float(gripper_position_m)
        self.commands += 1

    def safe_hold(self) -> None:
        return

    def close(self) -> None:
        self.enabled = False


class TrossenArmBackend:
    """Thin adapter over Trossen's official ``trossen_arm`` Python API."""

    def __init__(
        self,
        robot_ip: str,
        *,
        driver_module: ModuleType | object | None = None,
        platform_name: str | None = None,
    ) -> None:
        self.robot_ip = str(robot_ip)
        self._module = driver_module
        self._platform_name = platform.system() if platform_name is None else platform_name
        self._driver = None
        self._enabled = False
        self._last_positions: np.ndarray | None = None
        self._joint_limits: np.ndarray | None = None
        self._firmware_version = "unknown"

    def _load_module(self):
        if self._platform_name not in ("Linux", "Darwin"):
            raise HardwareUnavailableError(
                "Trossen's official trossen_arm driver supports Ubuntu and macOS, not Windows. "
                "No physical command was attempted."
            )
        if self._module is None:
            try:
                self._module = importlib.import_module("trossen_arm")
            except ImportError as exc:
                raise HardwareUnavailableError(
                    "Official trossen_arm Python driver is not installed. Install the version "
                    "whose major.minor matches the arm firmware."
                ) from exc
        return self._module

    @staticmethod
    def _enum_value(value: object) -> int:
        return int(getattr(value, "value", value))

    def connect(self) -> HardwareState:
        module = self._load_module()
        subnet = self.robot_ip.rsplit(".", 1)[0]
        host = int(self.robot_ip.rsplit(".", 1)[1])
        results = module.TrossenArmDriver.discover(
            subnet=subnet,
            ip_start=host,
            ip_end=host,
            timeout=0.05,
        )
        if len(results) != 1 or str(results[0].ip) != self.robot_ip:
            raise HardwareUnavailableError(f"official discovery did not find exactly one arm at {self.robot_ip}")
        result = results[0]
        if self._enum_value(result.error_state) != 0:
            raise HardwareSafetyError(f"arm discovery reported error state {result.error_state}")
        if self._enum_value(result.model) != self._enum_value(module.Model.wxai_v0):
            raise HardwareSafetyError("discovered controller is not a WidowXAI")
        self._firmware_version = str(result.firmware_version)

        driver = module.TrossenArmDriver()
        driver.configure(
            module.Model.wxai_v0,
            module.StandardEndEffector.wxai_v0_follower,
            self.robot_ip,
            False,
        )
        self._driver = driver
        return self.read_state()

    def enable_position_control(self) -> None:
        if self._driver is None:
            raise HardwareSafetyError("driver is not connected")
        self._driver.set_all_modes(self._module.Mode.position)
        self._enabled = True

    def read_state(self) -> HardwareState:
        if self._driver is None:
            raise HardwareSafetyError("driver is not connected")
        positions = np.asarray(self._driver.get_all_positions(), dtype=float).reshape(7)
        if self._joint_limits is None:
            limits_raw = self._driver.get_joint_limits()
            self._joint_limits = np.array(
                [[limit.position_min, limit.position_max] for limit in limits_raw],
                dtype=float,
            )
        limits = self._joint_limits
        self._last_positions = positions.copy()
        self._joint_limits = limits.copy()
        return HardwareState(positions[:6], positions[6], limits, self._firmware_version)

    def send_positions(self, q_command: np.ndarray, gripper_position_m: float) -> None:
        if self._driver is None or not self._enabled:
            raise HardwareSafetyError("position control is not enabled")
        positions = np.concatenate(
            [np.asarray(q_command, dtype=float).reshape(6), [float(gripper_position_m)]]
        )
        self._driver.set_all_positions(positions, 0.0, False)
        self._last_positions = positions.copy()

    def safe_hold(self) -> None:
        if self._driver is None or not self._enabled:
            return
        try:
            positions = np.asarray(self._driver.get_all_positions(), dtype=float).reshape(7)
            self._driver.set_all_modes(self._module.Mode.position)
            self._driver.set_all_positions(positions, 0.0, False)
            self._last_positions = positions.copy()
        except Exception:
            pass

    def close(self) -> None:
        if self._driver is None:
            return
        try:
            self._driver.cleanup()
        finally:
            self._driver = None
            self._enabled = False
