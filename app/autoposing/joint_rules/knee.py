from __future__ import annotations

import math
from dataclasses import dataclass, field

from core.math import (
    EPSILON,
    Vec3,
    add,
    angle_between_on_axis,
    clamp,
    distance,
    length,
    normalize,
    project_on_plane,
    quaternion_angle_axis,
    rotate_vector,
    scale,
    subtract,
)

from .common import degrees, pressure_for_angle, radians


@dataclass(frozen=True, slots=True)
class KneeRuleResult:
    target_position: Vec3
    hint_position: Vec3
    pressure: float = 0.0
    pressure_state: str = "normal"
    target_clamped: bool = False
    debug_angles: dict[str, float] = field(default_factory=dict)


class KneeRule:
    FLEXION_COMFORT = radians(120.0)
    FLEXION_LIMIT = radians(140.0)
    HYPEREXTENSION_LIMIT = radians(5.0)
    BEND_PLANE_COMFORT = radians(70.0)
    BEND_PLANE_LIMIT = radians(110.0)
    VARUS_VALGUS_COMFORT = radians(8.0)
    VARUS_VALGUS_LIMIT = radians(15.0)

    def solve(
        self,
        *,
        start_position: Vec3,
        target_position: Vec3,
        raw_hint_position: Vec3,
        fallback_hint_position: Vec3,
        upper_length: float,
        lower_length: float,
    ) -> KneeRuleResult:
        requested_flexion = self._flexion_angle(start_position, target_position, upper_length, lower_length)
        adjusted_target, target_clamped = self._limit_target_flexion(
            start_position,
            target_position,
            fallback_hint_position,
            upper_length,
            lower_length,
        )
        applied_flexion = self._flexion_angle(start_position, adjusted_target, upper_length, lower_length)

        chain_axis = normalize(subtract(adjusted_target, start_position))
        if length(chain_axis) <= EPSILON:
            chain_axis = normalize(subtract(target_position, start_position))
        if length(chain_axis) <= EPSILON:
            chain_axis = (0.0, -1.0, 0.0)

        fallback_direction = self._bend_direction(start_position, fallback_hint_position, chain_axis, None)
        raw_direction = self._bend_direction(start_position, raw_hint_position, chain_axis, fallback_direction)
        requested_plane = angle_between_on_axis(fallback_direction, raw_direction, chain_axis)
        applied_plane = clamp(requested_plane, -self.BEND_PLANE_LIMIT, self.BEND_PLANE_LIMIT)
        applied_direction = normalize(rotate_vector(quaternion_angle_axis(applied_plane, chain_axis), fallback_direction))
        if length(applied_direction) <= EPSILON:
            applied_direction = fallback_direction

        hint_distance = max(upper_length, lower_length, distance(start_position, fallback_hint_position), 1.0)
        adjusted_hint = add(start_position, scale(applied_direction, hint_distance))

        flexion_pressure, flexion_state = self._flexion_pressure(requested_flexion)
        plane_pressure, plane_state = self._bend_plane_pressure(requested_plane)
        pressure = max(flexion_pressure, plane_pressure)
        pressure_state = "stretch" if "stretch" in {flexion_state, plane_state} else "normal"
        return KneeRuleResult(
            target_position=adjusted_target,
            hint_position=adjusted_hint,
            pressure=pressure,
            pressure_state=pressure_state,
            target_clamped=target_clamped,
            debug_angles={
                "knee_flexion_degrees": degrees(applied_flexion),
                "knee_requested_flexion_degrees": degrees(requested_flexion),
                "knee_hyperextension_degrees": 0.0,
                "bend_plane_requested_degrees": degrees(requested_plane),
                "bend_plane_applied_degrees": degrees(applied_plane),
                "bend_plane_overflow_degrees": degrees(requested_plane - applied_plane),
                "valgus_varus_degrees": min(abs(degrees(requested_plane)), degrees(self.VARUS_VALGUS_LIMIT)),
            },
        )

    def _limit_target_flexion(
        self,
        start_position: Vec3,
        target_position: Vec3,
        fallback_hint_position: Vec3,
        upper_length: float,
        lower_length: float,
    ) -> tuple[Vec3, bool]:
        minimum_distance = self._distance_for_flexion(self.FLEXION_LIMIT, upper_length, lower_length)
        current_distance = distance(start_position, target_position)
        if current_distance >= minimum_distance:
            return target_position, False

        direction = normalize(subtract(target_position, start_position))
        if length(direction) <= EPSILON:
            direction = normalize(subtract(fallback_hint_position, start_position))
        if length(direction) <= EPSILON:
            direction = (0.0, -1.0, 0.0)
        return add(start_position, scale(direction, minimum_distance)), True

    def _flexion_angle(
        self,
        start_position: Vec3,
        target_position: Vec3,
        upper_length: float,
        lower_length: float,
    ) -> float:
        if upper_length <= EPSILON or lower_length <= EPSILON:
            return 0.0
        current_distance = distance(start_position, target_position)
        cosine_inner = clamp(
            (upper_length * upper_length + lower_length * lower_length - current_distance * current_distance)
            / (2.0 * upper_length * lower_length),
            -1.0,
            1.0,
        )
        inner_angle = math.acos(cosine_inner)
        return max(0.0, math.pi - inner_angle)

    def _distance_for_flexion(self, flexion_angle: float, upper_length: float, lower_length: float) -> float:
        inner_angle = math.pi - flexion_angle
        squared = (
            upper_length * upper_length
            + lower_length * lower_length
            - 2.0 * upper_length * lower_length * math.cos(inner_angle)
        )
        return max(math.sqrt(max(squared, 0.0)), abs(upper_length - lower_length) + 0.001)

    def _bend_direction(
        self,
        start_position: Vec3,
        hint_position: Vec3,
        chain_axis: Vec3,
        fallback_direction: Vec3 | None,
    ) -> Vec3:
        projected = normalize(project_on_plane(subtract(hint_position, start_position), chain_axis))
        if length(projected) > EPSILON:
            return projected
        if fallback_direction is not None and length(fallback_direction) > EPSILON:
            return fallback_direction
        fallback = normalize(project_on_plane((0.0, 0.0, 1.0), chain_axis))
        if length(fallback) > EPSILON:
            return fallback
        fallback = normalize(project_on_plane((0.0, -1.0, 0.0), chain_axis))
        return fallback if length(fallback) > EPSILON else (1.0, 0.0, 0.0)

    def _flexion_pressure(self, requested_flexion: float) -> tuple[float, str]:
        overflow = max(0.0, requested_flexion - self.FLEXION_COMFORT)
        limit = max(self.FLEXION_LIMIT - self.FLEXION_COMFORT, EPSILON)
        return pressure_for_angle(overflow, limit, comfort_ratio=0.0)

    def _bend_plane_pressure(self, requested_plane: float) -> tuple[float, str]:
        overflow = max(0.0, abs(requested_plane) - self.BEND_PLANE_COMFORT)
        limit = max(self.BEND_PLANE_LIMIT - self.BEND_PLANE_COMFORT, EPSILON)
        return pressure_for_angle(overflow, limit, comfort_ratio=0.0)
