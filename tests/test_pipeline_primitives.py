import numpy as np

from widowxai_quest_teleop.pose_filter import pose_ema
from widowxai_quest_teleop.sample_buffer import LatestValueMailbox
from widowxai_quest_teleop.safety import FreshSequenceWatchdog
from widowxai_quest_teleop.transport import parse_pose_message
from widowxai_quest_teleop.types import Pose
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
            "head": {"orientation_xyzw": [0, 0, 0, 1]},
            "reconnect_generation": 3,
        }
    )
    np.testing.assert_allclose(sample.controller_pose.quaternion_wxyz, [1, 0, 0, 0])
    assert sample.sequence == 7
    assert sample.reconnect_generation == 3
    np.testing.assert_allclose(sample.head_quaternion_wxyz, [1, 0, 0, 0])


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


def test_pose_ema_matches_reference_position_and_quaternion_nlerp() -> None:
    previous = Pose(np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]))
    current = Pose(np.ones(3), np.array([-0.8, -0.6, 0.0, 0.0]))
    filtered = pose_ema(previous, current, 0.8)
    np.testing.assert_allclose(filtered.position, np.full(3, 0.8))
    np.testing.assert_allclose(filtered.quaternion_wxyz, [0.86824314, 0.49613894, 0.0, 0.0])


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
