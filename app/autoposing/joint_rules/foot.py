from __future__ import annotations

import math
from dataclasses import dataclass

from core.math import (
    Vec3,
    dot,
    extract_twist_rotation,
    inverse_quaternion,
    length,
    normalize,
    quaternion_angle_axis,
    quaternion_multiply,
)

from app.autoposing.reach_constraints import ReachConstraintResult, clamp_target_to_reach

from .common import JointRuleResult, clamp_angle, degrees, pressure_for_angle, radians, signed_twist_angle


@dataclass(frozen=True, slots=True)
class BallRollPositionResult:
    solved_position: Vec3
    reach: ReachConstraintResult


class FootRollRule:
    INVERSION_COMFORT = radians(20.0)
    INVERSION_LIMIT = radians(30.0)
    EVERSION_COMFORT = radians(12.0)
    EVERSION_LIMIT = radians(20.0)
    TOE_ROLL_LIMIT = radians(30.0)

    def solve_hindfoot_roll(
        self,
        foot_local_delta,
        foot_axis: Vec3,
    ) -> JointRuleResult:
        twist = extract_twist_rotation(foot_local_delta, foot_axis)
        twist_angle = signed_twist_angle(twist, foot_axis)
        constrained = clamp_angle(twist_angle, self.EVERSION_LIMIT, self.INVERSION_LIMIT)
        pressure, state = self._roll_pressure(twist_angle)
        return JointRuleResult(
            rotation=quaternion_angle_axis(constrained, foot_axis),
            pressure=pressure,
            pressure_state=state,
            overflow_twist=twist_angle - constrained,
            debug_angles={
                "foot_roll_requested_degrees": degrees(twist_angle),
                "foot_roll_applied_degrees": degrees(constrained),
                "foot_inversion_degrees": degrees(max(constrained, 0.0)),
                "foot_eversion_degrees": degrees(max(-constrained, 0.0)),
                "foot_roll_overflow_degrees": degrees(twist_angle - constrained),
            },
        )

    def solve_toe_roll(
        self,
        ball_local_delta,
        toe_axis: Vec3 = (1.0, 0.0, 0.0),
    ) -> JointRuleResult:
        swing_axis, swing_angle = self._axis_angle(ball_local_delta)
        toe_axis = normalize(toe_axis)
        if length(toe_axis) <= 0.0:
            toe_axis = (1.0, 0.0, 0.0)
        sign = 1.0 if dot(swing_axis, toe_axis) >= 0.0 else -1.0
        signed_roll = swing_angle * sign
        constrained = clamp_angle(signed_roll, self.TOE_ROLL_LIMIT, self.TOE_ROLL_LIMIT)
        pressure, state = pressure_for_angle(signed_roll, self.TOE_ROLL_LIMIT)
        return JointRuleResult(
            rotation=quaternion_angle_axis(abs(constrained), swing_axis),
            pressure=pressure,
            pressure_state=state,
            overflow_twist=signed_roll - constrained,
            debug_angles={
                "toe_roll_requested_degrees": degrees(signed_roll),
                "toe_roll_applied_degrees": degrees(constrained),
                "toe_roll_overflow_degrees": degrees(signed_roll - constrained),
            },
        )

    def solve_ball_position(
        self,
        *,
        foot_position: Vec3,
        target_position: Vec3,
        floor_y: float,
        max_reach: float,
    ) -> BallRollPositionResult:
        reach = clamp_target_to_reach(foot_position, target_position, max_reach)
        solved = reach.clamped_target if reach.clamped_target[1] >= floor_y else (reach.clamped_target[0], floor_y, reach.clamped_target[2])
        return BallRollPositionResult(solved_position=solved, reach=reach)

    def _roll_pressure(self, twist_angle: float) -> tuple[float, str]:
        angle = abs(twist_angle)
        comfort = self.INVERSION_COMFORT if twist_angle >= 0.0 else self.EVERSION_COMFORT
        limit = self.INVERSION_LIMIT if twist_angle >= 0.0 else self.EVERSION_LIMIT
        overflow = max(0.0, angle - comfort)
        return pressure_for_angle(overflow, max(limit - comfort, 0.0001), comfort_ratio=0.0)

    @staticmethod
    def _axis_angle(rotation) -> tuple[Vec3, float]:
        w, x, y, z = rotation
        w = max(-1.0, min(1.0, w))
        angle = 2.0 * math.acos(w)
        if angle > math.pi:
            angle = math.tau - angle
            x, y, z = -x, -y, -z
        sine = math.sqrt(max(1.0 - w * w, 0.0))
        if sine <= 1e-6 or angle <= 1e-6:
            return (1.0, 0.0, 0.0), 0.0
        return normalize((x / sine, y / sine, z / sine)), angle
