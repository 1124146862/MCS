from __future__ import annotations

import math
from dataclasses import dataclass, field

from core.math import EPSILON, Vec3, add, clamp, dot, length, lerp, normalize, scale, subtract

from .common import degrees, pressure_for_angle, radians


@dataclass(frozen=True, slots=True)
class ShoulderGirdleRuleResult:
    clavicle_offset: Vec3
    chest_delta: Vec3
    activation: float = 0.0
    pressure: float = 0.0
    pressure_state: str = "normal"
    debug_angles: dict[str, float] = field(default_factory=dict)


class ShoulderGirdleRule:
    ELEVATION_RECRUIT_START = radians(45.0)
    ELEVATION_RECRUIT_FULL = radians(120.0)
    ELEVATION_PRESSURE_START = radians(155.0)
    ELEVATION_LIMIT = radians(170.0)
    CLAVICLE_CAP_RATIO = 0.12
    CHEST_CAP_RATIO = 0.030

    def solve(
        self,
        *,
        wrist_target: Vec3,
        rest_hand: Vec3,
        rest_upperarm: Vec3,
        default_upperarm: Vec3,
        arm_length: float,
        torso_length: float,
        up_axis: Vec3,
        forward_axis: Vec3,
        outward_axis: Vec3,
        reach_pressure: float,
        intent: float,
        shoulder_target: Vec3 | None = None,
        shoulder_preservation: float = 0.0,
        shoulder_pose_intent: float = 0.0,
    ) -> ShoulderGirdleRuleResult:
        if intent <= 0.0 or arm_length <= EPSILON:
            return self._empty()

        wrist_delta = subtract(wrist_target, rest_hand)
        activation = max(_smoothstep(0.12 * arm_length, 0.65 * arm_length, length(wrist_delta)), reach_pressure) * intent
        if activation <= 0.001:
            return self._empty()

        target_direction = normalize(subtract(wrist_target, default_upperarm))
        elevation = self._arm_elevation(target_direction, up_axis)
        rhythm = _smoothstep(self.ELEVATION_RECRUIT_START, self.ELEVATION_RECRUIT_FULL, elevation)
        vertical_delta = max(0.0, dot(subtract(wrist_target, rest_upperarm), normalize(up_axis)))
        height_activation = _smoothstep(0.15 * arm_length, 0.85 * arm_length, vertical_delta)
        wrist_direction = normalize(wrist_delta)
        forward_activation = max(0.0, dot(wrist_direction, normalize(forward_axis)))
        outward_activation = max(0.0, dot(wrist_direction, normalize(outward_axis)))

        scapular_share = 0.25 + 0.10 * rhythm
        clavicle_offset = add(
            add(
                scale(up_axis, arm_length * 0.16 * scapular_share * activation * height_activation),
                scale(forward_axis, arm_length * 0.10 * scapular_share * activation * forward_activation),
            ),
            scale(outward_axis, arm_length * 0.10 * scapular_share * activation * (0.35 + 0.65 * max(outward_activation, height_activation * 0.35))),
        )
        clavicle_offset = _cap(clavicle_offset, arm_length * self.CLAVICLE_CAP_RATIO)
        clavicle_offset = self._apply_shoulder_constraint(
            clavicle_offset,
            shoulder_target,
            default_upperarm,
            arm_length,
            shoulder_preservation,
            shoulder_pose_intent,
        )

        chest_delta = _cap(scale(wrist_delta, 0.028 * activation), torso_length * self.CHEST_CAP_RATIO)
        pressure, state = self._pressure(elevation)
        return ShoulderGirdleRuleResult(
            clavicle_offset=clavicle_offset,
            chest_delta=chest_delta,
            activation=activation,
            pressure=pressure,
            pressure_state=state,
            debug_angles={
                "shoulder_elevation_degrees": degrees(elevation),
                "scapular_rhythm": rhythm,
                "scapular_share": scapular_share,
                "height_activation": height_activation,
                "forward_activation": forward_activation,
                "outward_activation": outward_activation,
                "shoulder_preservation": shoulder_preservation,
            },
        )

    def _apply_shoulder_constraint(
        self,
        clavicle_offset: Vec3,
        shoulder_target: Vec3 | None,
        default_upperarm: Vec3,
        arm_length: float,
        shoulder_preservation: float,
        shoulder_pose_intent: float,
    ) -> Vec3:
        if shoulder_target is None:
            return clavicle_offset

        target_offset = _cap(subtract(shoulder_target, default_upperarm), arm_length * self.CLAVICLE_CAP_RATIO)
        if shoulder_preservation >= 0.99:
            return target_offset
        blend = clamp(max(shoulder_preservation * 0.70, shoulder_pose_intent * 0.45), 0.0, 0.85)
        return lerp(clavicle_offset, target_offset, blend)

    def _arm_elevation(self, target_direction: Vec3, up_axis: Vec3) -> float:
        if length(target_direction) <= EPSILON:
            return 0.0
        down_axis = scale(normalize(up_axis), -1.0)
        return math.acos(clamp(dot(normalize(target_direction), down_axis), -1.0, 1.0))

    def _pressure(self, elevation: float) -> tuple[float, str]:
        overflow = max(0.0, elevation - self.ELEVATION_PRESSURE_START)
        limit = max(self.ELEVATION_LIMIT - self.ELEVATION_PRESSURE_START, EPSILON)
        return pressure_for_angle(overflow, limit, comfort_ratio=0.0)

    @staticmethod
    def _empty() -> ShoulderGirdleRuleResult:
        return ShoulderGirdleRuleResult(clavicle_offset=(0.0, 0.0, 0.0), chest_delta=(0.0, 0.0, 0.0))


def _cap(vector: Vec3, max_length: float) -> Vec3:
    vector_length = length(vector)
    if vector_length <= max_length or vector_length <= 0.0:
        return vector
    return scale(vector, max_length / vector_length)


def _smoothstep(edge0: float, edge1: float, value: float) -> float:
    if edge1 <= edge0:
        return 1.0 if value >= edge1 else 0.0
    t = clamp((value - edge0) / (edge1 - edge0), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)
