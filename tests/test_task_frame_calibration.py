import json

import numpy as np

from widowxai_quest_teleop.config import load_config, resolve_project_path
from widowxai_quest_teleop.math3d import matrix_to_quat, rotation_vector_to_matrix
from widowxai_quest_teleop.task_frame_calibration import (
    build_calibration_document,
    direction_map_quality,
    fit_direction_map,
    heading_correction,
)


def test_direction_fit_recovers_a_proper_rotation_from_repeated_axes() -> None:
    expected_mapping = rotation_vector_to_matrix(np.array([0.25, -0.4, 0.15]))
    observed = np.vstack([np.eye(3), np.eye(3)])
    desired = (expected_mapping @ observed.T).T

    fitted, quality = fit_direction_map(observed, desired)

    np.testing.assert_allclose(fitted, expected_mapping, atol=1e-12)
    assert np.isclose(np.linalg.det(fitted), 1.0)
    assert quality["max_axis_error_deg"] < 1e-5


def test_heading_correction_removes_captured_head_yaw() -> None:
    yaw = 0.6
    heading = rotation_vector_to_matrix(np.array([0.0, yaw, 0.0]))
    quaternion = matrix_to_quat(heading)
    operator_forward = np.array([0.0, 0.0, -1.0])
    world_forward = heading @ operator_forward

    np.testing.assert_allclose(
        heading_correction(quaternion) @ world_forward,
        operator_forward,
        atol=1e-12,
    )


def test_calibration_document_uses_independent_valid_position_and_rotation_maps() -> None:
    position_mapping = rotation_vector_to_matrix(np.array([0.1, 0.2, -0.3]))
    rotation_mapping = rotation_vector_to_matrix(np.array([-0.2, 0.15, 0.35]))
    observed = np.vstack([np.eye(3), np.eye(3)])
    document = build_calibration_document(
        name="guided",
        position_observed=observed,
        position_desired=(position_mapping @ observed.T).T,
        rotation_observed=observed,
        rotation_desired=(rotation_mapping @ observed.T).T,
        captures=[],
    )

    np.testing.assert_allclose(document["position_matrix"], position_mapping, atol=1e-12)
    np.testing.assert_allclose(document["rotation_matrix"], rotation_mapping, atol=1e-12)
    assert document["task_rotation_signs"] == [1.0, 1.0, 1.0]


def test_linked_calibration_uses_one_rigid_frame_for_translation_and_rotation() -> None:
    mapping = rotation_vector_to_matrix(np.array([0.1, -0.3, 0.2]))
    observed = np.vstack([np.eye(3), np.eye(3)])
    desired = (mapping @ observed.T).T
    document = build_calibration_document(
        name="linked",
        position_observed=observed,
        position_desired=desired,
        rotation_observed=observed,
        rotation_desired=desired,
        captures=[],
        link_rotation_to_position=True,
    )

    np.testing.assert_allclose(document["position_matrix"], mapping, atol=1e-12)
    np.testing.assert_allclose(document["rotation_matrix"], mapping, atol=1e-12)
    assert document["guided_calibration"]["rotation_frame_linked_to_position"]
    quality = direction_map_quality(mapping, observed, desired)
    assert quality["max_axis_error_deg"] < 1e-5


def test_guided_mujoco_profile_is_linked_unshaped_and_hardware_disabled() -> None:
    config = load_config("configs/guided_calibration_30pct_mujoco.yaml")
    calibration = json.loads(
        resolve_project_path(config["quest"]["calibration"]).read_text(encoding="utf-8")
    )
    position = np.asarray(calibration["position_matrix"], dtype=float)
    rotation = np.asarray(calibration["rotation_matrix"], dtype=float)

    np.testing.assert_allclose(position, rotation, atol=1e-12)
    np.testing.assert_allclose(position.T @ position, np.eye(3), atol=1e-12)
    assert np.isclose(np.linalg.det(position), 1.0)
    assert calibration["task_rotation_signs"] == [1.0, 1.0, 1.0]
    assert calibration["guided_calibration"]["rotation_frame_linked_to_position"]
    assert config["control"]["update_mode"] == "quest_synchronized"
    assert not config["control"]["joint_command_limits"]["enabled"]
    assert not config["control"]["gripper_command_limits"]["enabled"]
    assert not config["hardware"]["enabled"]
    assert not config["hardware"]["require_explicit_enable"]
