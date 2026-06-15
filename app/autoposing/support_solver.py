from __future__ import annotations

from dataclasses import dataclass

from core.math import distance
from core.skeleton.models import Vec3

from .anatomy_profile import HumanoidAnatomyProfile
from .constraints import PoseConstraintSet


@dataclass(frozen=True, slots=True)
class SupportState:
    floor_y: float
    support_targets: dict[str, Vec3]
    support_weights: dict[str, float]


class SupportSolver:
    def state_from_constraints(self, anatomy: HumanoidAnatomyProfile, constraints: PoseConstraintSet) -> SupportState:
        support_targets: dict[str, Vec3] = {}
        support_weights: dict[str, float] = {}
        for support in constraints.supports.values():
            support_targets[support.joint_name] = self.clamp_to_floor(support.target_world_position, support.floor_y)
            support_weights[support.joint_name] = max(support_weights.get(support.joint_name, 0.0), support.support_weight)
        for contact in constraints.foot_contacts.values():
            support_targets[contact.joint_name] = self.clamp_to_floor(contact.target_world_position, contact.floor_y)
            support_weights[contact.joint_name] = max(support_weights.get(contact.joint_name, 0.0), contact.support_weight)
        return SupportState(floor_y=anatomy.floor_y, support_targets=support_targets, support_weights=support_weights)

    def clamp_to_floor(self, position: Vec3, floor_y: float) -> Vec3:
        if position[1] >= floor_y:
            return position
        return (position[0], floor_y, position[2])

    def target_for(self, state: SupportState, joint_name: str, fallback: Vec3) -> Vec3:
        return state.support_targets.get(joint_name, self.clamp_to_floor(fallback, state.floor_y))

    def limit_pelvis_height(self, anatomy: HumanoidAnatomyProfile, pelvis: Vec3) -> Vec3:
        left_leg = anatomy.length("thigh_l", "calf_l") + anatomy.length("calf_l", "foot_l")
        right_leg = anatomy.length("thigh_r", "calf_r") + anatomy.length("calf_r", "foot_r")
        average_leg = (left_leg + right_leg) * 0.5
        if average_leg <= 0.002:
            return pelvis
        min_height = anatomy.floor_y + average_leg * 0.28
        if pelvis[1] >= min_height:
            return pelvis
        return (pelvis[0], min_height, pelvis[2])

    def support_alignment_error(self, state: SupportState, solved_positions: dict[str, Vec3]) -> float:
        max_error = 0.0
        for joint_name, target in state.support_targets.items():
            solved = solved_positions.get(joint_name)
            if solved is not None:
                max_error = max(max_error, distance(target, solved))
        return max_error

    def stable_support_targets(self, state: SupportState, minimum_weight: float = 0.7) -> dict[str, Vec3]:
        return {
            joint_name: target
            for joint_name, target in state.support_targets.items()
            if state.support_weights.get(joint_name, 0.0) >= minimum_weight
        }
