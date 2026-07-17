import time

import numpy as np

from widowxai_quest_teleop.command_shaper import JointCommandShaper
from widowxai_quest_teleop.sample_buffer import LatestValueMailbox
from widowxai_quest_teleop.safety import FreshSequenceWatchdog, wait_for_cycle_period
from widowxai_quest_teleop.transport import parse_pose_message
from widowxai_quest_teleop.wrist_pivot import estimate_wrist_pivot_offset


def test_latest_mailbox_overwrites_old_state() -> None:
    mailbox: LatestValueMailbox[int] = LatestValueMailbox()
    mailbox.publish(1)
    mailbox.publish(2)
    value, generation = mailbox.take_latest()
    assert value == 2
    assert generation == 2
    assert mailbox.overwrite_count == 1


def test_transport_parses_webxr_xyzw_as_internal_wxyz() -> None:
    sample = parse_pose_message(
        {
            "type": "pose",
            "sequence": 7,
            "capture_monotonic_ms": 10.0,
            "capture_epoch_ms": 20.0,
            "send_monotonic_ms": 11.0,
            "left": {
                "position": [1, 2, 3],
                "orientation_xyzw": [0, 0, 0, 1],
                "grip": 0.8,
                "trigger": 0.2,
            },
            "reconnect_generation": 3,
        }
    )
    np.testing.assert_allclose(sample.controller_pose.quaternion_wxyz, [1, 0, 0, 0])
    assert sample.sequence == 7
    assert sample.reconnect_generation == 3


def test_watchdog_requires_fresh_window_and_times_out() -> None:
    base = parse_pose_message(
        {
            "type": "pose",
            "sequence": 1,
            "capture_monotonic_ms": 0,
            "capture_epoch_ms": 0,
            "send_monotonic_ms": 0,
            "left": {"position": [0, 0, 0], "orientation_xyzw": [0, 0, 0, 1]},
            "reconnect_generation": 1,
        }
    )
    watchdog = FreshSequenceWatchdog(0.01, fresh_samples_to_recover=2)
    assert not watchdog.observe(base).fresh
    second = type(base)(**{**base.__dict__, "sequence": 2, "pc_arrival_monotonic_ns": base.pc_arrival_monotonic_ns + 1})
    assert watchdog.observe(second).fresh
    assert not watchdog.poll(second.pc_arrival_monotonic_ns + 20_000_000).fresh


def test_command_shaper_respects_all_hard_bounds() -> None:
    limits = np.tile(np.array([-1.0, 1.0]), (6, 1))
    shaper = JointCommandShaper(
        np.zeros(6),
        limits,
        lowpass_hz=8.0,
        max_velocity=np.ones(6),
        max_acceleration=np.full(6, 4.0),
        max_jerk=np.full(6, 40.0),
        max_step=np.full(6, 0.005),
    )
    previous = np.zeros(6)
    previous_acceleration = np.zeros(6)
    for target in [5.0] * 100 + [-5.0] * 200:
        command = shaper.step(np.full(6, target), 0.01)
        assert np.all(np.abs(command - previous) <= 0.005 + 1e-12)
        assert np.all(command <= 1.0)
        assert np.all(command >= -1.0)
        assert np.all(np.abs(shaper.velocity) <= 0.5 + 1e-10)
        assert np.all(np.abs(shaper.acceleration) <= 4.0 + 1e-10)
        assert np.all(np.abs((shaper.acceleration - previous_acceleration) / 0.01) <= 40.0 + 1e-8)
        previous = command
        previous_acceleration = shaper.acceleration.copy()


def test_command_shaper_converges_without_a_limit_cycle() -> None:
    shaper = JointCommandShaper(
        np.zeros(6),
        np.tile(np.array([-1.0, 1.0]), (6, 1)),
        lowpass_hz=8.0,
        max_velocity=np.ones(6),
        max_acceleration=np.full(6, 4.0),
        max_jerk=np.full(6, 40.0),
        max_step=np.full(6, 0.005),
    )
    tail = []
    for index in range(1000):
        command = shaper.step(np.full(6, 0.25), 0.01)
        if index >= 900:
            tail.append(command)
    np.testing.assert_allclose(command, 0.25, atol=1e-9)
    assert np.max(np.ptp(np.asarray(tail), axis=0)) < 1e-9


def test_command_shaper_hold_clears_motion_and_stays_fixed() -> None:
    shaper = JointCommandShaper(
        np.zeros(6),
        np.tile(np.array([-1.0, 1.0]), (6, 1)),
        lowpass_hz=8.0,
        max_velocity=np.ones(6),
        max_acceleration=np.full(6, 4.0),
        max_jerk=np.full(6, 40.0),
        max_step=np.full(6, 0.005),
    )
    for _ in range(30):
        shaper.step(np.full(6, 0.5), 0.01)
    held = shaper.hold()
    np.testing.assert_array_equal(shaper.velocity, np.zeros(6))
    np.testing.assert_array_equal(shaper.acceleration, np.zeros(6))
    for _ in range(100):
        np.testing.assert_allclose(shaper.step(held, 0.01), held, atol=1e-12)


def test_cycle_wait_never_attempts_to_catch_up() -> None:
    sleeps: list[float] = []
    now = [10.004]

    def sleeper(duration: float) -> None:
        sleeps.append(duration)
        now[0] += duration

    wait_for_cycle_period(10.0, 0.01, clock=lambda: now[0], sleeper=sleeper)
    np.testing.assert_allclose(sleeps, [0.006], atol=1e-12)
    wait_for_cycle_period(10.0, 0.01, clock=lambda: 10.015, sleeper=sleeper)
    assert len(sleeps) == 1


def test_wrist_pivot_calibration_recovers_controller_frame_offset() -> None:
    rng = np.random.default_rng(4)
    true_offset = np.array([-0.045, 0.012, -0.018])
    rotations = []
    positions = []
    fixed_pivot = np.array([0.4, 1.0, -0.2])
    for _ in range(100):
        axis = rng.normal(size=3)
        axis /= np.linalg.norm(axis)
        angle = rng.uniform(-0.8, 0.8)
        x, y, z = axis
        skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
        rotation = np.eye(3) + np.sin(angle) * skew + (1 - np.cos(angle)) * (skew @ skew)
        rotations.append(rotation)
        positions.append(fixed_pivot - rotation @ true_offset)
    estimated = estimate_wrist_pivot_offset(np.asarray(positions), np.asarray(rotations))
    np.testing.assert_allclose(estimated, true_offset, atol=1e-9)
