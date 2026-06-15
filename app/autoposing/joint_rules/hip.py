from __future__ import annotations

from dataclasses import dataclass, field

from core.math import EPSILON, Vec3, add, length, scale, subtract

from app.autoposing.anatomy_profile import HumanoidAnatomyProfile
from app.autoposing.constraints import PoseConstraintSet

from .common import radians


@dataclass(frozen=True, slots=True)
class HipOverreachSource:
    controller_id: str
    overreach_vector: Vec3
    base_weight: float


@dataclass(frozen=True, slots=True)
class HipPelvisRuleResult:
    compensation: Vec3
    pelvis_weight: float
    chest_weight: float
    head_weight: float
    pressure: float = 0.0
    pressure_state: str = "normal"
    debug_angles: dict[str, float] = field(default_factory=dict)


class HipPelvisRule:
    FLEXION_LIMIT = radians(130.0)
    EXTENSION_COMFORT = radians(18.0)
    EXTENSION_LIMIT = radians(30.0)
    ABDUCTION_LIMIT = radians(50.0)
    ADDUCTION_LIMIT = radians(30.0)
    ROTATION_LIMIT = radians(40.0)

    def solve_leg_overreach(
        self,
        *,
        sources: tuple[HipOverreachSource, ...],
        constraints: PoseConstraintSet,
        stable_support_count: int,
        body_height: float,
    ) -> HipPelvisRuleResult:
        stable_opposite_support = stable_support_count == 1
        vertical_weight = 0.09 if stable_opposite_support else 1.0
        max_length = body_height * (0.045 if stable_opposite_support else 0.20)
        compensation = (0.0, 0.0, 0.0)
        for source in sources:
            intent = constraints.pose_intent_weight(source.controller_id)
            if intent <= 0.0:
                continue
            overreach = source.overreach_vector
            if vertical_weight != 1.0:
                overreach = (overreach[0], overreach[1] * vertical_weight, overreach[2])
            compensation = add(compensation, scale(overreach, source.base_weight * intent))
        compensation = self._cap_vector(compensation, max_length)

        return HipPelvisRuleResult(
            compensation=compensation,
            pelvis_weight=0.38 if stable_opposite_support else 0.68,
            chest_weight=0.08 if stable_opposite_support else 0.16,
            head_weight=0.03 if stable_opposite_support else 0.05,
            pressure=min(1.0, length(compensation) / max(max_length, EPSILON)) if length(compensation) > 0.0 else 0.0,
            pressure_state="stretch" if length(compensation) >= max(max_length - EPSILON, 0.0) and max_length > 0.0 else "normal",
            debug_angles={
                "hip_flexion_limit_degrees": 130.0,
                "hip_extension_comfort_degrees": 18.0,
                "hip_extension_limit_degrees": 30.0,
                "hip_abduction_limit_degrees": 50.0,
                "hip_adduction_limit_degrees": 30.0,
                "hip_rotation_limit_degrees": 40.0,
                "stable_support_count": float(stable_support_count),
                "vertical_weight": vertical_weight,
                "compensation_length": length(compensation),
                "max_compensation_length": max_length,
            },
        )

    def chain_start_for_side(
        self,
        *,
        anatomy: HumanoidAnatomyProfile,
        constraints: PoseConstraintSet,
        pelvis_position: Vec3,
        side: str,
    ) -> Vec3:
        thigh_name = f"thigh_{side}"
        default_start = anatomy.offset("pelvis", thigh_name, pelvis_position)
        controller_id = f"hip_{side}_lesser"
        if not constraints.engaged(controller_id):
            return default_start

        target = constraints.position_for(controller_id, default_start)
        influence = max(
            constraints.preservation_weight(controller_id),
            constraints.pose_intent_weight(controller_id),
            0.75 if constraints.locked(controller_id) else 0.0,
        )
        if influence <= 0.0:
            return default_start

        max_offset = max(anatomy.length("pelvis", thigh_name), anatomy.body_height * 0.04)
        offset = self._cap_vector(subtract(target, default_start), max_offset)
        return add(default_start, scale(offset, min(influence, 1.0)))

    @staticmethod
    def _cap_vector(vector: Vec3, max_length: float) -> Vec3:
        vector_length = length(vector)
        if vector_length <= max_length or vector_length <= EPSILON:
            return vector
        return scale(vector, max_length / vector_length)
