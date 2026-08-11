from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest

import scripts.run_dual_hardware as dual_hardware
from widowxai_quest_teleop.safety import TimeAlignedCommandHistory


class FeedbackBackend:
    def __init__(self, q_arm: np.ndarray, gripper_position_m: float = 0.03) -> None:
        self.reads = 0
        self.feedback = SimpleNamespace(
            q_arm=np.asarray(q_arm, dtype=float),
            gripper_position_m=gripper_position_m,
        )

    def read_state(self):
        self.reads += 1
        return self.feedback


def channel(side: str, q_feedback: np.ndarray | None = None) -> dual_hardware.ArmChannel:
    q = np.zeros(6) if q_feedback is None else np.asarray(q_feedback, dtype=float)
    result = dual_hardware.ArmChannel(side, FeedbackBackend(q))
    result.command_history = TimeAlignedCommandHistory(np.zeros(6), 9.0, 0.0)
    result.q_feedback_reference = np.zeros(6)
    result.q_feedback_error = np.zeros(6)
    return result


def test_feedback_telemetry_uses_actual_encoder_read_time(monkeypatch) -> None:
    left = channel("left", np.full(6, 0.01))
    arm = SimpleNamespace(q_feedback=np.zeros(6), gripper_feedback_m=0.0)
    monkeypatch.setattr(dual_hardware.time, "perf_counter_ns", lambda: 10_000_000_000)

    fault = dual_hardware.read_and_validate_channel_feedback(
        left,
        arm,
        {"max_feedback_error_rad": 0.08},
        feedback_period=0.02,
    )

    assert fault is None
    assert left.feedback_read_monotonic_ns == 10_000_000_000
    assert left.feedback_sample_fresh is True
    assert left.next_feedback_s == pytest.approx(10.02)
    np.testing.assert_allclose(left.q_feedback_reference, np.zeros(6))
    np.testing.assert_allclose(left.q_feedback_error, np.full(6, 0.01))
    np.testing.assert_allclose(arm.q_feedback, np.full(6, 0.01))


def test_no_encoder_read_is_not_reported_as_fresh() -> None:
    channels = {side: channel(side) for side in ("left", "right")}
    for arm_channel in channels.values():
        arm_channel.feedback_read_monotonic_ns = 123
        arm_channel.feedback_sample_fresh = True

    dual_hardware.reset_feedback_telemetry(channels)
    record = dual_hardware.add_feedback_telemetry({}, channels)

    for side in ("left", "right"):
        assert record[f"{side}_feedback_read_monotonic_ns"] == 0
        assert record[f"{side}_feedback_sample_fresh"] is False


def test_feedback_scheduler_marks_only_the_arm_actually_read(monkeypatch) -> None:
    channels = {side: channel(side) for side in ("left", "right")}
    arms = {
        side: SimpleNamespace(q_feedback=np.zeros(6), gripper_feedback_m=0.0)
        for side in ("left", "right")
    }
    channels["left"].next_feedback_s = 11.0
    channels["right"].next_feedback_s = 9.0
    monkeypatch.setattr(dual_hardware.time, "perf_counter", lambda: 10.0)
    monkeypatch.setattr(dual_hardware.time, "perf_counter_ns", lambda: 10_000_000_000)

    dual_hardware.reset_feedback_telemetry(channels)
    fault = dual_hardware.read_due_feedback(
        channels,
        arms,
        {"max_feedback_error_rad": 0.08},
        feedback_period=0.02,
    )

    assert fault is None
    assert channels["left"].backend.reads == 0
    assert channels["left"].feedback_sample_fresh is False
    assert channels["right"].backend.reads == 1
    assert channels["right"].feedback_sample_fresh is True


def test_idle_row_does_not_reuse_previous_command_send_times() -> None:
    channels = {side: channel(side) for side in ("left", "right")}
    channels["left"].send_monotonic_ns = 101
    channels["left"].send_epoch_ns = 201
    channels["right"].send_monotonic_ns = 102
    channels["right"].send_epoch_ns = 202

    idle_monotonic, idle_epoch = dual_hardware.command_send_telemetry(
        channels, command_sent=False
    )
    sent_monotonic, sent_epoch = dual_hardware.command_send_telemetry(
        channels, command_sent=True
    )

    assert idle_monotonic == {"left": 0, "right": 0}
    assert idle_epoch == {"left": 0, "right": 0}
    assert sent_monotonic == {"left": 101, "right": 102}
    assert sent_epoch == {"left": 201, "right": 202}


def test_tracking_fault_sample_is_retained_for_the_stop_row(monkeypatch) -> None:
    measured = np.zeros(6)
    measured[2] = 0.101761
    channels = {
        "left": channel("left", measured),
        "right": channel("right"),
    }
    arm = SimpleNamespace(q_feedback=np.zeros(6), gripper_feedback_m=0.0)
    monkeypatch.setattr(dual_hardware.time, "perf_counter_ns", lambda: 10_000_000_000)

    fault = dual_hardware.read_and_validate_channel_feedback(
        channels["left"],
        arm,
        {"max_feedback_error_rad": 0.08},
        feedback_period=0.02,
    )
    record = dual_hardware.add_feedback_telemetry({}, channels, fault=fault)

    assert fault is not None
    assert "joint 2 time-aligned tracking error" in record["fault_reason"]
    assert record["left_feedback_sample_fresh"] is True
    assert record["left_feedback_read_monotonic_ns"] == 10_000_000_000
    np.testing.assert_allclose(record["left_q_feedback_reference"], np.zeros(6))
    np.testing.assert_allclose(record["left_q_feedback_error"], measured)
    # Validation stops at the first fault, so the untouched arm must not claim
    # that an encoder read happened in the same row.
    assert record["right_feedback_sample_fresh"] is False
    assert record["right_feedback_read_monotonic_ns"] == 0
