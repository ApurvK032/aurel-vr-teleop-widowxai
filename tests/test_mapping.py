import numpy as np

from widowxai_quest_teleop.mapping import ClutchPoseMapper
from widowxai_quest_teleop.math3d import matrix_to_quat, quat_to_matrix, rotation_vector_to_matrix
from widowxai_quest_teleop.types import Pose


IDENTITY_POSE = Pose(np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]))


def test_confirmed_position_matrix_maps_column_basis_vectors() -> None:
    mapper = ClutchPoseMapper(
        "configs/calibrations/left_behind_full_pose_good_20260602.json",
        position_reach_limit_m=None,
        rotation_reach_limit_rad=None,
    )
    for axis in range(3):
        mapper.engage(IDENTITY_POSE, IDENTITY_POSE, np.zeros(3))
        delta = np.zeros(3)
        delta[axis] = 0.01
        target = mapper.update(Pose(delta, IDENTITY_POSE.quaternion_wxyz), IDENTITY_POSE)
        assert target is not None
        np.testing.assert_allclose(target.position, 0.01 * mapper.position_matrix[:, axis], atol=1e-12)
        mapper.release()


def test_reengage_has_no_target_jump() -> None:
    mapper = ClutchPoseMapper(
        "configs/calibrations/left_behind_full_pose_good_20260602.json",
        position_reach_limit_m=None,
        rotation_reach_limit_rad=None,
    )
    controller = Pose(np.array([1.0, 2.0, 3.0]), np.array([1.0, 0.0, 0.0, 0.0]))
    robot = Pose(np.array([0.2, -0.1, 0.4]), np.array([1.0, 0.0, 0.0, 0.0]))
    mapper.engage(controller, robot, np.array([0.1, -0.1, 0.4]))
    first = mapper.update(controller, robot)
    assert first is not None
    np.testing.assert_allclose(first.position, robot.position, atol=1e-12)
    mapper.release()
    mapper.engage(Pose(controller.position + 10.0, controller.quaternion_wxyz), robot, np.array([0.1, -0.1, 0.4]))
    second = mapper.update(Pose(controller.position + 10.0, controller.quaternion_wxyz), robot)
    assert second is not None
    np.testing.assert_allclose(second.position, robot.position, atol=1e-12)


def test_head_yaw_is_frozen_at_engage_and_keeps_operator_forward_aligned() -> None:
    mapper = ClutchPoseMapper(
        "configs/calibrations/left_behind_full_pose_good_20260602.json",
        position_reach_limit_m=None,
        rotation_reach_limit_rad=None,
    )
    yaw = 0.7
    heading = rotation_vector_to_matrix(np.array([0.0, yaw, 0.0]))
    head_quaternion = matrix_to_quat(heading)
    mapper.engage(
        IDENTITY_POSE,
        IDENTITY_POSE,
        np.zeros(3),
        head_quaternion_wxyz=head_quaternion,
    )
    operator_forward = heading @ np.array([0.0, 0.0, -0.05])
    target = mapper.update(Pose(operator_forward, IDENTITY_POSE.quaternion_wxyz), IDENTITY_POSE)
    assert target is not None
    expected = mapper.calibrated_position_matrix @ np.array([0.0, 0.0, -0.05])
    np.testing.assert_allclose(target.position, expected, atol=1e-12)
    np.testing.assert_allclose(mapper.rotation_matrix, mapper.calibrated_rotation_matrix @ heading.T, atol=1e-12)
    assert np.isclose(mapper.engage_head_yaw_rad, yaw)


def test_mirrored_position_mode_does_not_apply_unconfirmed_orientation() -> None:
    mapper = ClutchPoseMapper(
        "configs/calibrations/left_mirrored_position_good_20260602.json",
        position_reach_limit_m=None,
        rotation_reach_limit_rad=None,
    )
    mapper.engage(IDENTITY_POSE, IDENTITY_POSE, np.zeros(3))
    controller_rotation = matrix_to_quat(rotation_vector_to_matrix(np.array([0.4, 0.0, 0.0])))
    target = mapper.update(Pose(np.zeros(3), controller_rotation), IDENTITY_POSE)
    assert target is not None
    np.testing.assert_allclose(quat_to_matrix(target.quaternion_wxyz), np.eye(3), atol=1e-12)
