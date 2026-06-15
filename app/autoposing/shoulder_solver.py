from __future__ import annotations

from dataclasses import dataclass, field

from core.math import add, dot, length, normalize, scale, subtract
from core.skeleton.models import Vec3

from .anatomy_profile import HumanoidAnatomyProfile
from .constraints import PoseConstraintSet
from .limb_solver import SolvedLimb


@dataclass(frozen=True, slots=True)
class ShoulderGirdleResult:
    chest_target: Vec3
    clavicle_offsets: dict[str, Vec3] = field(default_factory=dict)
    shoulder_scores: dict[str, float] = field(default_factory=dict)


class ShoulderGirdleSolver:
    def solve(
        self,
        anatomy: HumanoidAnatomyProfile,
        constraints: PoseConstraintSet,
        limbs: dict[str, SolvedLimb],
        chest_target: Vec3,
    ) -> ShoulderGirdleResult:
        clavicle_offsets: dict[str, Vec3] = {}
        scores: dict[str, float] = {}
        chest_delta = (0.0, 0.0, 0.0)

        for side, limb_key in (("l", "left_arm"), ("r", "right_arm")):
            result = self._solve_side(anatomy, constraints, limbs, side, limb_key, chest_target)
            clavicle_offsets[f"clavicle_{side}"] = result.clavicle_offset
            scores[side] = result.activation
            chest_delta = add(chest_delta, result.chest_delta)

        if constraints.locked("chest_secondary") or constraints.engaged("chest_dir"):
            adjusted_chest = chest_target
        else:
            adjusted_chest = add(chest_target, self._cap(chest_delta, anatomy.torso_length * 0.075))

        return ShoulderGirdleResult(chest_target=adjusted_chest, clavicle_offsets=clavicle_offsets, shoulder_scores=scores)

    def _solve_side(
        self,
        anatomy: HumanoidAnatomyProfile,
        constraints: PoseConstraintSet,
        limbs: dict[str, SolvedLimb],
        side: str,
        limb_key: str,
        chest_target: Vec3,
    ) -> "_SideShoulderResult":
        wrist_id = f"wrist_{side}_main"
        clavicle_name = f"clavicle_{side}"
        hand_name = f"hand_{side}"
        upperarm_name = f"upperarm_{side}"
        default_clavicle = anatomy.offset("spine_05", clavicle_name, chest_target)
        constraint = constraints.positions.get(wrist_id)
        intent = constraints.pose_intent_weight(wrist_id)
        if constraint is None or intent <= 0.0:
            return _SideShoulderResult((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.0)

        arm_length = max(anatomy.arm_length(side), 0.001)
        wrist_delta = subtract(constraint.target_world_position, anatomy.rest(hand_name))
        delta_length = length(wrist_delta)
        limb = limbs.get(limb_key)
        reach_pressure = limb.reach.pressure if limb is not None else 0.0
        activation = max(_smoothstep(0.12 * arm_length, 0.65 * arm_length, delta_length), reach_pressure) * intent
        if activation <= 0.001:
            return _SideShoulderResult((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.0)

        shoulder_y = anatomy.rest(upperarm_name)[1]
        height_activation = _smoothstep(0.15 * arm_length, 0.75 * arm_length, constraint.target_world_position[1] - shoulder_y)
        forward_activation = max(0.0, dot(normalize(wrist_delta), anatomy.forward_axis))
        outward = self._outward_axis(anatomy, side, default_clavicle)

        clavicle_delta = add(
            add(
                scale(anatomy.up_axis, arm_length * 0.035 * activation * height_activation),
                scale(anatomy.forward_axis, arm_length * 0.025 * activation * forward_activation),
            ),
            scale(outward, arm_length * 0.020 * activation),
        )
        clavicle_delta = self._cap(clavicle_delta, arm_length * 0.08)
        chest_delta = self._cap(scale(wrist_delta, 0.06 * activation), anatomy.torso_length * 0.05)
        return _SideShoulderResult(clavicle_delta, chest_delta, activation)

    def _outward_axis(self, anatomy: HumanoidAnatomyProfile, side: str, default_clavicle: Vec3) -> Vec3:
        from_chest = normalize(subtract(default_clavicle, anatomy.rest("spine_05")))
        if length(from_chest) > 0.0:
            return from_chest
        return anatomy.outward_axis(side)

    @staticmethod
    def _cap(vector: Vec3, max_length: float) -> Vec3:
        vector_length = length(vector)
        if vector_length <= max_length or vector_length <= 0.0:
            return vector
        return scale(vector, max_length / vector_length)


@dataclass(frozen=True, slots=True)
class _SideShoulderResult:
    clavicle_offset: Vec3
    chest_delta: Vec3
    activation: float


def _smoothstep(edge0: float, edge1: float, value: float) -> float:
    if edge1 <= edge0:
        return 1.0 if value >= edge1 else 0.0
    t = max(0.0, min(1.0, (value - edge0) / (edge1 - edge0)))
    return t * t * (3.0 - 2.0 * t)
