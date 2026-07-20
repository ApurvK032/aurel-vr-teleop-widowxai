from types import SimpleNamespace

import numpy as np
import pytest

from widowxai_quest_teleop.config import load_config
from widowxai_quest_teleop.hardware import (
    CommandGate,
    HardwareSafetyError,
    HardwareUnavailableError,
    TrossenArmBackend,
)


def make_gate() -> CommandGate:
    limits = np.vstack([np.tile([-1.0, 1.0], (6, 1)), [0.0, 0.044]])
    return CommandGate(
        np.zeros(6),
        0.044,
        limits,
        np.full(6, 0.02),
        joint_limit_margin_rad=0.05,
        gripper_limits_m=(0.022, 0.044),
        max_gripper_delta_m=0.005,
    )


def test_command_gate_rejects_instead_of_modifying_commands() -> None:
    gate = make_gate()
    accepted = np.full(6, 0.01)
    gate.validate(accepted, 0.041)
    np.testing.assert_array_equal(gate.previous_q, accepted)
    with pytest.raises(HardwareSafetyError, match="per-tick"):
        gate.validate(np.full(6, 0.04), 0.041)
    with pytest.raises(HardwareSafetyError, match="limit margin"):
        gate.validate(np.full(6, 0.96), 0.041)
    with pytest.raises(HardwareSafetyError, match="non-finite"):
        gate.validate(np.full(6, np.nan), 0.041)


def test_hardware_demo_is_half_article_gain_and_has_no_trajectory_stage() -> None:
    config = load_config("configs/hardware_demo.yaml")
    assert config["quest"]["translation_scale"] == 0.75
    assert config["quest"]["rotation_scale"] == 0.75
    assert config["control"] == {"loop_rate_hz": 200, "pose_filter_alpha": 0.8}
    assert "command_goal_time_s" not in config["control"]


def test_official_backend_fails_closed_on_windows() -> None:
    backend = TrossenArmBackend("192.168.1.2", driver_module=object(), platform_name="Windows")
    with pytest.raises(HardwareUnavailableError, match="not Windows"):
        backend.connect()


def test_official_backend_uses_immediate_nonblocking_driver_calls() -> None:
    class EnumValue:
        def __init__(self, value):
            self.value = value

    class Limit:
        def __init__(self, lower, upper):
            self.position_min = lower
            self.position_max = upper

    class FakeDriver:
        instance = None

        def __init__(self):
            FakeDriver.instance = self
            self.positions = np.array([0.0, 1.0, 0.7, 0.0, 0.0, 0.0, 0.044])
            self.calls = []

        @staticmethod
        def discover(**kwargs):
            assert kwargs == {"subnet": "192.168.1", "ip_start": 2, "ip_end": 2, "timeout": 0.05}
            return [SimpleNamespace(ip="192.168.1.2", model=EnumValue(1), error_state=EnumValue(0), firmware_version="1.11.1")]

        def configure(self, *args):
            self.calls.append(("configure", args))

        def get_all_positions(self):
            return self.positions.copy()

        def get_joint_limits(self):
            return [Limit(-1.0, 1.0) for _ in range(6)] + [Limit(0.0, 0.044)]

        def set_all_modes(self, mode):
            self.calls.append(("mode", mode))

        def set_all_positions(self, positions, goal_time, blocking):
            self.positions = np.asarray(positions, dtype=float).copy()
            self.calls.append(("positions", self.positions.copy(), goal_time, blocking))

        def cleanup(self):
            self.calls.append(("cleanup",))

    module = SimpleNamespace(
        TrossenArmDriver=FakeDriver,
        Model=SimpleNamespace(wxai_v0=EnumValue(1)),
        StandardEndEffector=SimpleNamespace(wxai_v0_follower=EnumValue(2)),
        Mode=SimpleNamespace(position=EnumValue(3)),
    )
    backend = TrossenArmBackend("192.168.1.2", driver_module=module, platform_name="Linux")
    state = backend.connect()
    assert state.firmware_version == "1.11.1"
    backend.enable_position_control()
    q = np.array([0.01, 0.99, 0.69, 0.02, -0.01, 0.03])
    backend.send_positions(q, 0.04)
    call = FakeDriver.instance.calls[-1]
    assert call[0] == "positions"
    np.testing.assert_array_equal(call[1], np.concatenate([q, [0.04]]))
    assert call[2:] == (0.0, False)
    backend.close()
    assert FakeDriver.instance.calls[-1] == ("cleanup",)
