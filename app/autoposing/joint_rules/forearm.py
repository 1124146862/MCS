from __future__ import annotations

from core.math import extract_twist_rotation, quaternion_angle_axis

from core.skeleton.models import Vec3

from .common import JointRuleResult, clamp_angle, degrees, pressure_for_angle, radians, signed_twist_angle


class ForearmRule:
    PRONATION_LIMIT = radians(80.0)
    SUPINATION_LIMIT = radians(85.0)

    def solve(self, hand_local_delta, forearm_axis: Vec3) -> JointRuleResult:
        twist = extract_twist_rotation(hand_local_delta, forearm_axis)
        twist_angle = signed_twist_angle(twist, forearm_axis)
        constrained_angle = clamp_angle(twist_angle, self.SUPINATION_LIMIT, self.PRONATION_LIMIT)
        overflow = twist_angle - constrained_angle
        limit = self.PRONATION_LIMIT if twist_angle >= 0.0 else self.SUPINATION_LIMIT
        pressure, state = pressure_for_angle(twist_angle, limit)
        return JointRuleResult(
            rotation=quaternion_angle_axis(constrained_angle, forearm_axis),
            pressure=pressure,
            pressure_state=state,
            overflow_twist=overflow,
            debug_angles={
                "forearm_twist_degrees": degrees(twist_angle),
                "forearm_applied_degrees": degrees(constrained_angle),
                "forearm_overflow_degrees": degrees(overflow),
            },
        )
