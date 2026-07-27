from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

from .config import ArmPlacement, resolve_project_path
from .model import ARM_JOINT_NAMES, DEFAULT_MODEL_XML, JOINT3_ORIGIN_IN_LINK3

DUAL_ARM_SIDES = ("left", "right")


@dataclass(frozen=True)
class CollisionReport:
    """One collision verdict for the combined two-arm scene."""

    colliding: bool
    kind: str = "none"
    side: str | None = None
    bodies: tuple[str, str] | None = None
    separation_m: float = float("inf")

    def describe(self) -> str:
        if not self.colliding:
            return "no collision"
        bodies = "" if self.bodies is None else f" between {self.bodies[0]} and {self.bodies[1]}"
        side = "" if self.side is None else f" on the {self.side} arm"
        return f"{self.kind} collision{side}{bodies}"


class DualArmCollisionModel:
    """Both WidowXAI arms in one MuJoCo model at their measured base transforms.

    Two independent single-arm models physically cannot see each other, so the
    dual-arm safety gate needs one scene. Per-arm inverse kinematics still uses
    its own ``WidowXAIModel``: the solver mutates ``MjData`` as scratch between
    ``fk`` and ``jacobian``, so two solvers must never share one model.

    This class owns collision screening and the viewer only.
    """

    def __init__(
        self,
        placements: dict[str, ArmPlacement],
        *,
        xml_path: str | Path = DEFAULT_MODEL_XML,
        clearance_m: float = 0.0,
    ) -> None:
        missing = [side for side in DUAL_ARM_SIDES if side not in placements]
        if missing:
            raise ValueError(f"dual-arm model is missing base transforms for {missing}")
        self.xml_path = resolve_project_path(xml_path)
        if not self.xml_path.exists():
            raise FileNotFoundError(
                f"WidowXAI model not found: {self.xml_path}. "
                "Run git submodule update --init --recursive."
            )
        self.clearance_m = float(clearance_m)
        if not np.isfinite(self.clearance_m) or self.clearance_m < 0.0:
            raise ValueError("cross-arm clearance must be finite and nonnegative")

        spec = mujoco.MjSpec()
        spec.visual.headlight.diffuse = [0.6, 0.6, 0.6]
        spec.visual.headlight.ambient = [0.3, 0.3, 0.3]
        spec.visual.headlight.specular = [0.0, 0.0, 0.0]
        spec.worldbody.add_light(
            name="viewer_light",
            pos=[0.0, 0.0, 1.5],
            dir=[0.0, 0.0, -1.0],
            type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL,
        )
        spec.worldbody.add_geom(
            name="floor",
            type=mujoco.mjtGeom.mjGEOM_PLANE,
            size=[0.0, 0.0, 0.05],
            rgba=[0.16, 0.20, 0.24, 1.0],
            contype=0,
            conaffinity=0,
        )

        self.prefixes = {side: f"{side}_" for side in DUAL_ARM_SIDES}
        for side in DUAL_ARM_SIDES:
            placement = placements[side]
            arm = mujoco.MjSpec.from_file(str(self.xml_path))
            link3 = arm.body("link_3")
            if link3 is None:
                raise RuntimeError("official model is missing link_3")
            # Inject the decoupled-IK wrist pivot before attaching so the
            # prefix rename applies to it exactly as it does to ee_site.
            link3.add_site(
                name="wrist_anchor",
                pos=JOINT3_ORIGIN_IN_LINK3.tolist(),
                size=[0.008, 0.0, 0.0],
                rgba=[1.0, 0.45, 0.0, 1.0],
            )
            frame = spec.worldbody.add_frame(
                pos=placement.position_m.tolist(),
                quat=placement.quaternion_wxyz.tolist(),
            )
            spec.attach(arm, prefix=self.prefixes[side], frame=frame)

        self.model = spec.compile()
        self.data = mujoco.MjData(self.model)

        self.arm_qpos_indices: dict[str, np.ndarray] = {}
        self.gripper_qpos_indices: dict[str, np.ndarray] = {}
        self.ee_site_ids: dict[str, int] = {}
        self.wrist_site_ids: dict[str, int] = {}
        for side, prefix in self.prefixes.items():
            joint_ids = [self._joint_id(f"{prefix}{name}") for name in ARM_JOINT_NAMES]
            self.arm_qpos_indices[side] = np.array(
                [self.model.jnt_qposadr[index] for index in joint_ids], dtype=int
            )
            self.gripper_qpos_indices[side] = np.array(
                [
                    self.model.jnt_qposadr[self._joint_id(f"{prefix}right_carriage_joint")],
                    self.model.jnt_qposadr[self._joint_id(f"{prefix}left_carriage_joint")],
                ],
                dtype=int,
            )
            self.ee_site_ids[side] = self._site_id(f"{prefix}ee_site")
            self.wrist_site_ids[side] = self._site_id(f"{prefix}wrist_anchor")

        # The two carriage collision boxes intentionally meet when a valid
        # gripper closes. The whitelist must carry the attach prefix, otherwise
        # it silently stops matching and every closed gripper reads as a
        # self-collision.
        self._carriage_pairs = {
            frozenset({f"{prefix}carriage_right", f"{prefix}carriage_left"})
            for prefix in self.prefixes.values()
        }
        self._body_side = self._build_body_side_map()
        # Cache once. This is queried inside the 90 Hz control tick, and only
        # collidable geometry can ever produce a cross-arm hazard: the visual
        # meshes carry contype/conaffinity 0 and would just inflate the
        # pairwise distance loop.
        self._collidable_geoms = {
            side: self._collidable_geoms_for_side(side) for side in DUAL_ARM_SIDES
        }
        self._rbounds = {
            side: self.model.geom_rbound[self._collidable_geoms[side]].copy()
            for side in DUAL_ARM_SIDES
        }

    def _site_id(self, name: str) -> int:
        identifier = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, name)
        if identifier < 0:
            raise RuntimeError(f"dual-arm model site not found: {name}")
        return identifier

    def _joint_id(self, name: str) -> int:
        identifier = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if identifier < 0:
            raise RuntimeError(f"dual-arm model joint not found: {name}")
        return identifier

    def _build_body_side_map(self) -> dict[int, str | None]:
        mapping: dict[int, str | None] = {}
        for body_id in range(self.model.nbody):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, body_id) or ""
            side_for_body = None
            for side, prefix in self.prefixes.items():
                if name.startswith(prefix):
                    side_for_body = side
                    break
            mapping[body_id] = side_for_body
        return mapping

    def forward(
        self,
        q_by_side: dict[str, np.ndarray],
        gripper_by_side: dict[str, float],
    ) -> None:
        """Place both arms, then run one shared forward pass.

        Both arms are written every call. Zeroing the shared ``qpos`` and
        setting only one arm would silently evaluate the other arm at its zero
        pose, which is the exact failure that makes cross-arm screening useless.
        """

        missing = [side for side in DUAL_ARM_SIDES if side not in q_by_side]
        if missing:
            raise ValueError(f"dual-arm forward is missing joint commands for {missing}")
        self.data.qpos[:] = 0.0
        for side in DUAL_ARM_SIDES:
            q = np.asarray(q_by_side[side], dtype=float).reshape(6)
            if not np.all(np.isfinite(q)):
                raise ValueError(f"{side} arm joint command is non-finite")
            self.data.qpos[self.arm_qpos_indices[side]] = q
            gripper = float(gripper_by_side.get(side, 0.044))
            if not np.isfinite(gripper):
                raise ValueError(f"{side} gripper command is non-finite")
            self.data.qpos[self.gripper_qpos_indices[side]] = gripper
        mujoco.mj_forward(self.model, self.data)

    def check(
        self,
        q_by_side: dict[str, np.ndarray],
        gripper_by_side: dict[str, float],
    ) -> CollisionReport:
        """Classify the combined state as clear, self-colliding, or cross-arm.

        Cross-arm contacts are reported in preference to self-collisions so an
        operator sees the two-arm hazard first when both are present.
        """

        self.forward(q_by_side, gripper_by_side)
        self_report: CollisionReport | None = None
        for index in range(self.data.ncon):
            contact = self.data.contact[index]
            body_1 = int(self.model.geom_bodyid[contact.geom1])
            body_2 = int(self.model.geom_bodyid[contact.geom2])
            name_1 = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, body_1) or ""
            name_2 = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, body_2) or ""
            side_1 = self._body_side.get(body_1)
            side_2 = self._body_side.get(body_2)
            if side_1 is None or side_2 is None:
                continue
            if side_1 != side_2:
                return CollisionReport(
                    colliding=True,
                    kind="cross-arm",
                    side=None,
                    bodies=(name_1, name_2),
                    separation_m=float(contact.dist),
                )
            if frozenset({name_1, name_2}) in self._carriage_pairs:
                continue
            if self_report is None:
                self_report = CollisionReport(
                    colliding=True,
                    kind="self",
                    side=side_1,
                    bodies=(name_1, name_2),
                    separation_m=float(contact.dist),
                )
        if self_report is not None:
            return self_report
        if self.clearance_m <= 0.0:
            # No margin configured: contact above is the whole verdict. Skip
            # the pairwise distance sweep rather than pay for it every tick.
            return CollisionReport(colliding=False)
        separation = self.minimum_cross_arm_distance()
        if separation < self.clearance_m:
            return CollisionReport(
                colliding=True,
                kind="cross-arm-clearance",
                separation_m=separation,
            )
        return CollisionReport(colliding=False, separation_m=separation)

    def minimum_cross_arm_distance(self) -> float:
        """Smallest signed distance between any left-arm and right-arm geom.

        ``mj_geomDistance`` reports separation rather than only contact, so a
        configured clearance margin can reject a near miss before it becomes a
        touch. Requires ``forward`` to have been called.

        Returns the search cutoff when every pair is farther than that; the
        value is a bound, not an exact distance, which is all a margin test
        needs.

        Two guards wrap the MuJoCo call. ``mj_geomDistance`` in mujoco 3.8.1
        returns exactly 0.0 for some box-box pairs once ``distmax`` grows past
        the true separation — verified on this model with two 0.31 m-apart
        link boxes. A spurious zero would fail closed, but it would also make
        the margin unusable, so:

        * the search cutoff is kept just above the configured margin, which
          stays inside the regime where the primitive path behaves; and
        * every result is raised to the bounding-sphere lower bound, which is a
          provable minimum for the true distance. Because the true distance is
          never below that bound, raising to it can only under-report
          clearance, never over-report it.
        """

        cutoff = max(self.clearance_m * 2.0, 0.02)
        left = self._collidable_geoms["left"]
        right = self._collidable_geoms["right"]
        geom_xpos = self.data.geom_xpos

        # Vectorised bounding-sphere pre-filter. Only pairs whose provable
        # lower bound falls under the cutoff can affect the result, which
        # usually leaves a handful of exact tests instead of every pair.
        centre_gaps = np.linalg.norm(
            geom_xpos[left][:, None, :] - geom_xpos[right][None, :, :], axis=-1
        )
        lower_bounds = (
            centre_gaps
            - self._rbounds["left"][:, None]
            - self._rbounds["right"][None, :]
        )
        candidates = np.argwhere(lower_bounds < cutoff)
        if candidates.size == 0:
            return cutoff

        minimum = cutoff
        for left_index, right_index in candidates:
            lower_bound = float(lower_bounds[left_index, right_index])
            if lower_bound >= minimum:
                continue
            distance = float(
                mujoco.mj_geomDistance(
                    self.model,
                    self.data,
                    left[left_index],
                    right[right_index],
                    cutoff,
                    None,
                )
            )
            distance = max(distance, lower_bound)
            if distance < minimum:
                minimum = distance
        return minimum

    def _collidable_geoms_for_side(self, side: str) -> list[int]:
        return [
            geom_id
            for geom_id in range(self.model.ngeom)
            if self._body_side.get(int(self.model.geom_bodyid[geom_id])) == side
            and (
                int(self.model.geom_contype[geom_id]) != 0
                or int(self.model.geom_conaffinity[geom_id]) != 0
            )
        ]

    def first_collision_on_path(
        self,
        start_q: dict[str, np.ndarray],
        end_q: dict[str, np.ndarray],
        *,
        start_gripper: dict[str, float] | None = None,
        end_gripper: dict[str, float] | None = None,
        samples: int = 251,
    ) -> tuple[float, CollisionReport] | None:
        """Screen a straight-line joint path for both arms simultaneously.

        Both arms are interpolated together, so a path that is clear for each
        arm alone but collides when they move at the same time is rejected.
        """

        start_gripper = start_gripper or {}
        end_gripper = end_gripper or {}
        for alpha in np.linspace(0.0, 1.0, max(2, int(samples))):
            q_by_side = {}
            gripper_by_side = {}
            for side in DUAL_ARM_SIDES:
                start = np.asarray(start_q[side], dtype=float).reshape(6)
                end = np.asarray(end_q[side], dtype=float).reshape(6)
                q_by_side[side] = start + alpha * (end - start)
                start_g = float(start_gripper.get(side, 0.044))
                end_g = float(end_gripper.get(side, start_g))
                gripper_by_side[side] = start_g + alpha * (end_g - start_g)
            report = self.check(q_by_side, gripper_by_side)
            if report.colliding:
                return float(alpha), report
        return None

    def set_viewer_qpos(
        self,
        data: mujoco.MjData,
        q_by_side: dict[str, np.ndarray],
        gripper_by_side: dict[str, float],
    ) -> None:
        for side in DUAL_ARM_SIDES:
            data.qpos[self.arm_qpos_indices[side]] = np.asarray(
                q_by_side[side], dtype=float
            ).reshape(6)
            data.qpos[self.gripper_qpos_indices[side]] = float(
                gripper_by_side.get(side, 0.044)
            )
        mujoco.mj_forward(self.model, data)
