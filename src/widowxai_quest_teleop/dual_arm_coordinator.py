from __future__ import annotations

from dataclasses import dataclass, field
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
            return self._reject(proposals, CollisionReport(colliding=False), stream_lost, external_fault, held_sides)

        report = CollisionReport(colliding=False)
        if self.cross_arm_collision:
            report = self.collision_model.check(
                {side: proposals[side].q_command for side in DUAL_ARM_SIDES},
                {side: proposals[side].gripper_command_m for side in DUAL_ARM_SIDES},
            )
        if report.colliding:
            return self._reject(
                proposals,
                report,
                stream_lost,
                f"combined scene rejects the commanded state: {report.describe()}",
                held_sides,
            )

        for side in DUAL_ARM_SIDES:
            self.arms[side].commit(proposals[side])
        return CoordinatedTick(
            proposals=proposals,
            collision=report,
            accepted=True,
            stream_lost=stream_lost,
            held_sides=held_sides,
        )

    def _reject(
        self,
        proposals: dict[str, ArmProposal],
        report: CollisionReport,
        stream_lost: bool,
        reason: str,
        held_sides: tuple[str, ...],
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
    collision_model = DualArmCollisionModel(
        {side: arms_config[side].placement for side in DUAL_ARM_SIDES},
        xml_path=config["model"]["xml_path"],
        clearance_m=float(safety.get("cross_arm_clearance_m", 0.0)),
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
    )
    return arms, collision_model, coordinator
