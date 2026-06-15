from __future__ import annotations

from dataclasses import dataclass, field

from core.math import add, length, normalize, scale, subtract
from core.skeleton.models import Vec3

from .anatomy_profile import HumanoidAnatomyProfile
from .constraints import PoseConstraintSet
from .joint_rules.shoulder import ShoulderGirdleRule, ShoulderGirdleRuleResult
from .limb_solver import SolvedLimb


@dataclass(frozen=True, slots=True)
class ShoulderGirdleResult:
    chest_target: Vec3
    clavicle_offsets: dict[str, Vec3] = field(default_factory=dict)
    shoulder_scores: dict[str, float] = field(default_factory=dict)
    rule_results: dict[str, ShoulderGirdleRuleResult] = field(default_factory=dict)


class ShoulderGirdleSolver:
    def __init__(self) -> None:
        self._rule = ShoulderGirdleRule()

    def solve(
        self,
        anatomy: HumanoidAnatomyProfile,
        constraints: PoseConstraintSet,
        limbs: dict[str, SolvedLimb],
        chest_target: Vec3,
    ) -> ShoulderGirdleResult:
        clavicle_offsets: dict[str, Vec3] = {}
        scores: dict[str, float] = {}
        rule_results: dict[str, ShoulderGirdleRuleResult] = {}
        chest_delta = (0.0, 0.0, 0.0)

        for side, limb_key in (("l", "left_arm"), ("r", "right_arm")):
            rule_result = self._solve_side(anatomy, constraints, limbs, side, limb_key, chest_target)
            clavicle_offsets[f"clavicle_{side}"] = rule_result.clavicle_offset
            scores[side] = rule_result.activation
            rule_results[side] = rule_result
            chest_delta = add(chest_delta, rule_result.chest_delta)

        if constraints.locked("chest_secondary") or constraints.engaged("chest_dir"):
            adjusted_chest = chest_target
        else:
            adjusted_chest = add(chest_target, self._cap(chest_delta, anatomy.torso_length * 0.075))

        return ShoulderGirdleResult(
            chest_target=adjusted_chest,
            clavicle_offsets=clavicle_offsets,
            shoulder_scores=scores,
            rule_results=rule_results,
        )

    def _solve_side(
        self,
        anatomy: HumanoidAnatomyProfile,
        constraints: PoseConstraintSet,
        limbs: dict[str, SolvedLimb],
        side: str,
        limb_key: str,
        chest_target: Vec3,
    ) -> ShoulderGirdleRuleResult:
        wrist_id = f"wrist_{side}_main"
        shoulder_id = f"shoulder_{side}_lesser"
        clavicle_name = f"clavicle_{side}"
        hand_name = f"hand_{side}"
        upperarm_name = f"upperarm_{side}"
        default_clavicle = anatomy.offset("spine_05", clavicle_name, chest_target)
        default_upperarm = anatomy.offset(clavicle_name, upperarm_name, default_clavicle)
        constraint = constraints.positions.get(wrist_id)
        intent = constraints.pose_intent_weight(wrist_id)
        if constraint is None or intent <= 0.0:
            return ShoulderGirdleRuleResult(clavicle_offset=(0.0, 0.0, 0.0), chest_delta=(0.0, 0.0, 0.0))

        arm_length = max(anatomy.arm_length(side), 0.001)
        limb = limbs.get(limb_key)
        reach_pressure = limb.reach.pressure if limb is not None else 0.0
        outward = self._outward_axis(anatomy, side, default_clavicle)
        shoulder_constraint = constraints.positions.get(shoulder_id)
        shoulder_target = shoulder_constraint.target_world_position if shoulder_constraint is not None else None

        return self._rule.solve(
            wrist_target=constraint.target_world_position,
            rest_hand=anatomy.rest(hand_name),
            rest_upperarm=anatomy.rest(upperarm_name),
            default_upperarm=default_upperarm,
            arm_length=arm_length,
            torso_length=anatomy.torso_length,
            up_axis=anatomy.up_axis,
            forward_axis=anatomy.forward_axis,
            outward_axis=outward,
            reach_pressure=reach_pressure,
            intent=intent,
            shoulder_target=shoulder_target,
            shoulder_preservation=constraints.preservation_weight(shoulder_id),
            shoulder_pose_intent=constraints.pose_intent_weight(shoulder_id),
        )

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
