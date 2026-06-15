from __future__ import annotations

import math

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

from .common import JointRuleResult, clamp_angle, degrees, pressure_for_angle, radians, signed_twist_angle


class AnkleRule:
    DORSIFLEXION_COMFORT = radians(15.0)
    DORSIFLEXION_LIMIT = radians(20.0)
    PLANTARFLEXION_COMFORT = radians(45.0)
    PLANTARFLEXION_LIMIT = radians(55.0)
    AXIAL_TWIST_COMFORT = radians(8.0)
    AXIAL_TWIST_LIMIT = radians(12.0)

    def solve(
        self,
        foot_local_delta,
        foot_axis: Vec3,
        flexion_axis: Vec3 = (1.0, 0.0, 0.0),
    ) -> JointRuleResult:
        twist = extract_twist_rotation(foot_local_delta, foot_axis)
        swing_delta = quaternion_multiply(foot_local_delta, inverse_quaternion(twist))
        swing_axis, swing_angle = self._axis_angle(swing_delta)
        flexion_axis = normalize(flexion_axis)
        if length(flexion_axis) <= 0.0:
            flexion_axis = (1.0, 0.0, 0.0)
        swing_sign = 1.0 if dot(swing_axis, flexion_axis) >= 0.0 else -1.0
        signed_swing = swing_angle * swing_sign
        constrained_swing = clamp_angle(signed_swing, self.PLANTARFLEXION_LIMIT, self.DORSIFLEXION_LIMIT)
        swing_rotation = quaternion_angle_axis(abs(constrained_swing), swing_axis)

        twist_angle = signed_twist_angle(twist, foot_axis)
        constrained_twist = clamp_angle(twist_angle, self.AXIAL_TWIST_LIMIT, self.AXIAL_TWIST_LIMIT)
        twist_rotation = quaternion_angle_axis(constrained_twist, foot_axis)
        adjusted_delta = quaternion_multiply(swing_rotation, twist_rotation)

        swing_pressure, swing_state = self._swing_pressure(signed_swing)
        twist_pressure, twist_state = self._twist_pressure(twist_angle)
        pressure = max(swing_pressure, twist_pressure)
        state = "stretch" if "stretch" in {swing_state, twist_state} else "normal"
        return JointRuleResult(
            rotation=adjusted_delta,
            pressure=pressure,
            pressure_state=state,
            overflow_twist=twist_angle - constrained_twist,
            debug_angles={
                "ankle_requested_degrees": degrees(signed_swing),
                "ankle_applied_degrees": degrees(constrained_swing),
                "ankle_dorsiflexion_degrees": degrees(max(constrained_swing, 0.0)),
                "ankle_plantarflexion_degrees": degrees(max(-constrained_swing, 0.0)),
                "ankle_axial_twist_degrees": degrees(constrained_twist),
                "ankle_axial_requested_degrees": degrees(twist_angle),
                "ankle_axial_overflow_degrees": degrees(twist_angle - constrained_twist),
            },
        )

    def _swing_pressure(self, signed_swing: float) -> tuple[float, str]:
        angle = abs(signed_swing)
        comfort = self.DORSIFLEXION_COMFORT if signed_swing >= 0.0 else self.PLANTARFLEXION_COMFORT
        limit = self.DORSIFLEXION_LIMIT if signed_swing >= 0.0 else self.PLANTARFLEXION_LIMIT
        overflow = max(0.0, angle - comfort)
        return pressure_for_angle(overflow, max(limit - comfort, 0.0001), comfort_ratio=0.0)

    def _twist_pressure(self, twist_angle: float) -> tuple[float, str]:
        overflow = max(0.0, abs(twist_angle) - self.AXIAL_TWIST_COMFORT)
        return pressure_for_angle(overflow, self.AXIAL_TWIST_LIMIT - self.AXIAL_TWIST_COMFORT, comfort_ratio=0.0)

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
