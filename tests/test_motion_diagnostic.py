import numpy as np
import pytest

from widowxai_quest_teleop.config import load_config
from widowxai_quest_teleop.decoupled_ik import DecoupledIK
from widowxai_quest_teleop.motion_diagnostic import (
    DIAGNOSTIC_MOTIONS,
    build_six_axis_plan,
    minimum_snap_fraction,
    validate_plan_dynamics,
)


def make_plan(model):
    config = load_config("configs/six_axis_arm_diagnostic.yaml")
    solver = DecoupledIK.from_config(model, config)
    limits = model.joint_limits.copy()
    limits[:, 0] += config["hardware"]["joint_limit_margin_rad"]
    limits[:, 1] -= config["hardware"]["joint_limit_margin_rad"]
    solver.set_joint_limits(limits)
    diagnostic = config["diagnostic"]
    plan = build_six_axis_plan(
        model,
        solver,
        np.asarray(config["model"]["simulation_start_q_rad"], dtype=float),
        gripper_position_m=config["hardware"]["gripper_open_m"],
        rate_hz=diagnostic["command_rate_hz"],
        segment_duration_s=diagnostic["segment_duration_s"],
        pause_duration_s=diagnostic["pause_duration_s"],
        translation_amplitude_m=diagnostic["translation_amplitude_m"],
        rotation_amplitude_rad=diagnostic["rotation_amplitude_rad"],
        maximum_endpoint_position_error_m=diagnostic[
            "maximum_endpoint_position_error_m"
        ],
        maximum_endpoint_orientation_error_rad=diagnostic[
            "maximum_endpoint_orientation_error_rad"
        ],
        maximum_ik_iterations=diagnostic["maximum_ik_iterations"],
    )
    return config, plan


def test_minimum_snap_fraction_has_stationary_endpoints() -> None:
    assert minimum_snap_fraction(0.0) == pytest.approx(0.0)
    assert minimum_snap_fraction(1.0) == pytest.approx(1.0)
    epsilon = 1e-4
    assert minimum_snap_fraction(epsilon) < 1e-12
    assert 1.0 - minimum_snap_fraction(1.0 - epsilon) < 1e-12
    with pytest.raises(ValueError, match="within"):
        minimum_snap_fraction(1.01)


def test_six_axis_plan_is_complete_smooth_and_collision_free(model) -> None:
    config, plan = make_plan(model)
    diagnostic = config["diagnostic"]
    assert {point.motion_key for point in plan.points} == {
        motion.key for motion in DIAGNOSTIC_MOTIONS
    }
    assert len(plan.endpoints) == 12
    np.testing.assert_allclose(plan.points[-1].q_command, plan.home_q, atol=1e-12)
    assert all(
        not model.in_self_collision(point.q_command, config["hardware"]["gripper_open_m"])
        for point in plan.points
    )

    dynamics = validate_plan_dynamics(
        plan,
        maximum_step_rad=np.asarray(diagnostic["maximum_joint_step_rad"]),
        maximum_velocity_rad_s=np.asarray(diagnostic["maximum_joint_velocity_rad_s"]),
        maximum_acceleration_rad_s2=np.asarray(
            diagnostic["maximum_joint_acceleration_rad_s2"]
        ),
        maximum_jerk_rad_s3=np.asarray(diagnostic["maximum_joint_jerk_rad_s3"]),
    )
    assert np.max(dynamics.max_velocity_rad_s) < 0.2
    assert np.max(dynamics.max_step_rad) < 0.003


def test_six_axis_plan_rejects_amplitudes_that_do_not_converge(model) -> None:
    config = load_config("configs/six_axis_arm_diagnostic.yaml")
    solver = DecoupledIK.from_config(model, config)
    with pytest.raises(ValueError, match="did not converge"):
        build_six_axis_plan(
            model,
            solver,
            np.asarray(config["model"]["simulation_start_q_rad"], dtype=float),
            gripper_position_m=config["hardware"]["gripper_open_m"],
            translation_amplitude_m=2.0,
            maximum_ik_iterations=2,
        )
