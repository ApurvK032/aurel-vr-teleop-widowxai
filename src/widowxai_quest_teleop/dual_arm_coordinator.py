from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import Any

import numpy as np

from .arm_runtime import ArmProposal, ArmRuntime
from .config import ArmConfig
from .dual_arm_model import CollisionReport, DualArmCollisionModel
from .types import BimanualQuestSample

DUAL_ARM_SIDES = ("left", "right")


@dataclass(frozen=True)
class CoordinatedTick:
    """The outcome of one coordinated control tick for both arms."""

    proposals: dict[str, ArmProposal]
    collision: CollisionReport
    accepted: bool
    stream_lost: bool
    fault_reason: str = ""
    held_sides: tuple[str, ...] = field(default_factory=tuple)
    control_state: str = "normal"

    @property
    def moving_sides(self) -> tuple[str, ...]:
        if not self.accepted:
            return ()
        return tuple(
            side for side, proposal in sorted(self.proposals.items()) if proposal.active
        )


class DualArmCoordinator:
    """Owns the one place where two arms become one coordinated action.

    The ordering is deliberate and is the whole reason a dual-arm runtime
    cannot be two single-arm processes:

    1. consume one Quest sequence exactly once;
    2. judge each controller's freshness independently;
    3. solve both arms against that one packet;
    4. screen the *combined* commanded state for cross-arm collision;
    5. only then commit and send.

    A combined state that fails step 4 is rejected for both arms, even when
    only one arm was moving. Neither arm may proceed from a state that was
    never sent.
    """

    def __init__(
        self,
        arms: dict[str, ArmRuntime],
        collision_model: DualArmCollisionModel,
        *,
        coordinated_fault_hold: bool = True,
        cross_arm_collision: bool = True,
        collision_recovery_max_joint_delta_rad: np.ndarray | None = None,
        collision_recovery_max_gripper_delta_m: float = 0.001,
    ) -> None:
        missing = [side for side in DUAL_ARM_SIDES if side not in arms]
        if missing:
            raise ValueError(f"dual-arm coordinator is missing arms: {missing}")
        hands = [arms[side].controller_hand for side in DUAL_ARM_SIDES]
        if len(set(hands)) != len(hands):
            raise ValueError("both arm runtimes claim the same controller hand")
        self.arms = dict(arms)
        self.collision_model = collision_model
        self.coordinated_fault_hold = bool(coordinated_fault_hold)
        self.cross_arm_collision = bool(cross_arm_collision)
        recovery_delta = (
            np.full(6, 0.010)
            if collision_recovery_max_joint_delta_rad is None
            else np.asarray(collision_recovery_max_joint_delta_rad, dtype=float).reshape(6)
        )
        if not np.all(np.isfinite(recovery_delta)) or np.any(recovery_delta <= 0.0):
            raise ValueError("collision recovery joint deltas must be finite and positive")
        recovery_gripper_delta = float(collision_recovery_max_gripper_delta_m)
        if not np.isfinite(recovery_gripper_delta) or recovery_gripper_delta <= 0.0:
            raise ValueError("collision recovery gripper delta must be finite and positive")
        self.collision_recovery_max_joint_delta_rad = recovery_delta.copy()
        self.collision_recovery_max_gripper_delta_m = recovery_gripper_delta
        self.collision_recovery_active = False
        self.rejected_ticks = 0
        self.stream_loss_ticks = 0
        self.last_reconnect_generation: int | None = None

    def observe_reconnect(self, sample: BimanualQuestSample | None) -> bool:
        """Force both arms to re-anchor across a relay reconnect.

        A reconnect means the operator's controllers and the arms may no longer
        agree on where "here" is. Re-anchoring both together avoids one arm
        keeping a stale clutch anchor while the other takes a new one.
        """

        if sample is None:
            return False
        changed = (
            self.last_reconnect_generation is not None
            and sample.reconnect_generation != self.last_reconnect_generation
        )
        self.last_reconnect_generation = sample.reconnect_generation
        if changed:
            for arm in self.arms.values():
                arm.force_reanchor()
        return changed

    def step(
        self,
        sample: BimanualQuestSample | None,
        *,
        limiter_dt: float,
        robot_q_source: dict[str, object] | None = None,
        external_fault: str = "",
    ) -> CoordinatedTick:
        self.observe_reconnect(sample)
        sources = robot_q_source or {}

        for arm in self.arms.values():
            arm.observe(sample)

        proposals: dict[str, ArmProposal] = {}
        for side in DUAL_ARM_SIDES:
            arm = self.arms[side]
            proposals[side] = arm.solve(
                sample,
                limiter_dt=limiter_dt,
                robot_q_source=sources.get(side),
            )

        # Loss of the shared WebXR/relay connection is distinguishable from one
        # controller leaving the volume: neither hand is fresh at the same time.
        stream_lost = not any(proposal.freshness.fresh for proposal in proposals.values())
        if stream_lost:
            self.stream_loss_ticks += 1

        held_sides = tuple(
            side
            for side in DUAL_ARM_SIDES
            if not proposals[side].tracked or not proposals[side].freshness.fresh
        )

        if external_fault:
            return self._reject(
                proposals,
                CollisionReport(colliding=False),
                stream_lost,
                external_fault,
                held_sides,
            )

        report = CollisionReport(colliding=False)
        if self.cross_arm_collision:
            report = self.collision_model.check(
                {side: proposals[side].q_command for side in DUAL_ARM_SIDES},
                {side: proposals[side].gripper_command_m for side in DUAL_ARM_SIDES},
            )
        if report.colliding:
            self.collision_recovery_active = True
            return self._reject(
                proposals,
                report,
                stream_lost,
                f"combined scene rejects the commanded state: {report.describe()}",
                held_sides,
                control_state="collision_hold",
            )

        control_state = "normal"
        bounded_override = False
        recovery_complete = True
        if self.collision_recovery_active:
            control_state = "collision_recovery"
            bounded: dict[str, ArmProposal] = {}
            for side in DUAL_ARM_SIDES:
                arm = self.arms[side]
                proposal = proposals[side]
                q_delta = np.clip(
                    proposal.q_command - arm.q_command,
                    -self.collision_recovery_max_joint_delta_rad,
                    self.collision_recovery_max_joint_delta_rad,
                )
                gripper_delta = float(
                    np.clip(
                        proposal.gripper_command_m - arm.gripper_command_m,
                        -self.collision_recovery_max_gripper_delta_m,
                        self.collision_recovery_max_gripper_delta_m,
                    )
                )
                q_command = arm.q_command + q_delta
                gripper_command = arm.gripper_command_m + gripper_delta
                side_complete = np.allclose(
                    q_command, proposal.q_command, rtol=0.0, atol=1e-12
                ) and np.isclose(
                    gripper_command,
                    proposal.gripper_command_m,
                    rtol=0.0,
                    atol=1e-12,
                )
                recovery_complete = recovery_complete and bool(side_complete)
                flags = list(proposal.limiter_flags)
                if not side_complete:
                    flags.append("collision_recovery")
                bounded[side] = replace(
                    proposal,
                    q_command=q_command,
                    gripper_command_m=gripper_command,
                    feedforward_velocity=np.zeros(6),
                    limiter_flags=flags,
                )
            proposals = bounded
            bounded_override = True
            report = self.collision_model.check(
                {side: proposals[side].q_command for side in DUAL_ARM_SIDES},
                {side: proposals[side].gripper_command_m for side in DUAL_ARM_SIDES},
            )
            if report.colliding:
                return self._reject(
                    proposals,
                    report,
                    stream_lost,
                    f"combined scene rejects collision recovery: {report.describe()}",
                    held_sides,
                    control_state="collision_hold",
                )

        for side in DUAL_ARM_SIDES:
            if bounded_override:
                self.arms[side].commit_bounded_override(proposals[side])
            else:
                self.arms[side].commit(proposals[side])
        if bounded_override and recovery_complete:
            self.collision_recovery_active = False
        return CoordinatedTick(
            proposals=proposals,
            collision=report,
            accepted=True,
            stream_lost=stream_lost,
            held_sides=held_sides,
            control_state=control_state,
        )

    def _reject(
        self,
        proposals: dict[str, ArmProposal],
        report: CollisionReport,
        stream_lost: bool,
        reason: str,
        held_sides: tuple[str, ...],
        *,
        control_state: str = "fault_hold",
    ) -> CoordinatedTick:
        self.rejected_ticks += 1
        for arm in self.arms.values():
            arm.reject()
        return CoordinatedTick(
            proposals=proposals,
            collision=report,
            accepted=False,
            stream_lost=stream_lost,
            fault_reason=reason,
            held_sides=tuple(DUAL_ARM_SIDES) if self.coordinated_fault_hold else held_sides,
            control_state=control_state,
        )

    def hold_all(self) -> None:
        for arm in self.arms.values():
            arm.hold()


def build_dual_arm_system(
    config: dict[str, Any],
    arms_config: dict[str, ArmConfig],
    *,
    initial_q: np.ndarray,
    initial_gripper_m: float,
    control_gripper: bool = True,
) -> tuple[dict[str, ArmRuntime], DualArmCollisionModel, DualArmCoordinator]:
    """Build both arm runtimes, the combined scene, and the coordinator.

    Shared by the simulation, preflight, and hardware launchers so all three
    exercise the same construction path rather than three near-copies.
    """

    safety = config.get("safety", {})
    simulation_environment = config.get("simulation_environment", {}) or {}
    if not isinstance(simulation_environment, dict):
        raise ValueError("simulation_environment must be a mapping")
    collision_model = DualArmCollisionModel(
        {side: arms_config[side].placement for side in DUAL_ARM_SIDES},
        xml_path=config["model"]["xml_path"],
        clearance_m=float(safety.get("cross_arm_clearance_m", 0.0)),
        tabletop=simulation_environment.get("tabletop"),
    )
    arms = {
        side: ArmRuntime(
            arms_config[side],
            config,
            initial_q=initial_q,
            initial_gripper_m=initial_gripper_m,
            control_gripper=control_gripper,
        )
        for side in DUAL_ARM_SIDES
    }
    coordinator = DualArmCoordinator(
        arms,
        collision_model,
        coordinated_fault_hold=bool(safety.get("coordinated_fault_hold", True)),
        cross_arm_collision=bool(safety.get("cross_arm_collision", True)),
        collision_recovery_max_joint_delta_rad=np.asarray(
            safety.get("collision_recovery_max_joint_delta_rad", [0.010] * 6),
            dtype=float,
        ),
        collision_recovery_max_gripper_delta_m=float(
            safety.get("collision_recovery_max_gripper_delta_m", 0.001)
        ),
    )
    return arms, collision_model, coordinator
