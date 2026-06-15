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


class SpineRule:
    FLEXION_LIMIT = radians(90.0)
    EXTENSION_LIMIT = radians(30.0)
    LATERAL_BEND_LIMIT = radians(30.0)
    AXIAL_ROTATION_LIMIT = radians(30.0)

    def solve(
        self,
        local_delta,
        spine_axis: Vec3,
        *,
        flexion_axis: Vec3 = (1.0, 0.0, 0.0),
        lateral_axis: Vec3 = (0.0, 0.0, 1.0),
        label: str = "spine",
    ) -> JointRuleResult:
        spine_axis = _fallback_axis(spine_axis, (0.0, 1.0, 0.0))
        flexion_axis = _fallback_axis(flexion_axis, (1.0, 0.0, 0.0))
        lateral_axis = _fallback_axis(lateral_axis, (0.0, 0.0, 1.0))

        twist = extract_twist_rotation(local_delta, spine_axis)
        swing_delta = quaternion_multiply(local_delta, inverse_quaternion(twist))
        swing_axis, swing_angle = _axis_angle(swing_delta)
        constrained_swing, swing_kind = self._constrain_swing(swing_axis, swing_angle, flexion_axis, lateral_axis)
        twist_angle = signed_twist_angle(twist, spine_axis)
        constrained_twist = clamp_angle(twist_angle, self.AXIAL_ROTATION_LIMIT, self.AXIAL_ROTATION_LIMIT)

        swing_rotation = quaternion_angle_axis(abs(constrained_swing), swing_axis)
        twist_rotation = quaternion_angle_axis(constrained_twist, spine_axis)
        pressure, state = self._pressure(swing_kind, constrained_swing, swing_angle, twist_angle)
        return JointRuleResult(
            rotation=quaternion_multiply(swing_rotation, twist_rotation),
            pressure=pressure,
            pressure_state=state,
            overflow_twist=twist_angle - constrained_twist,
            debug_angles={
                f"{label}_swing_requested_degrees": degrees(swing_angle),
                f"{label}_swing_applied_degrees": degrees(abs(constrained_swing)),
                f"{label}_swing_kind": float({"flexion": 1, "extension": 2, "lateral": 3}.get(swing_kind, 0)),
                f"{label}_axial_requested_degrees": degrees(twist_angle),
                f"{label}_axial_applied_degrees": degrees(constrained_twist),
                f"{label}_axial_overflow_degrees": degrees(twist_angle - constrained_twist),
            },
        )

    def _constrain_swing(
        self,
        swing_axis: Vec3,
        swing_angle: float,
        flexion_axis: Vec3,
        lateral_axis: Vec3,
    ) -> tuple[float, str]:
        if swing_angle <= 1e-6:
            return 0.0, "neutral"
        flex_weight = abs(dot(swing_axis, flexion_axis))
        lateral_weight = abs(dot(swing_axis, lateral_axis))
        if flex_weight >= lateral_weight:
            sign = 1.0 if dot(swing_axis, flexion_axis) >= 0.0 else -1.0
            signed = swing_angle * sign
            return clamp_angle(signed, self.EXTENSION_LIMIT, self.FLEXION_LIMIT), "flexion" if signed >= 0.0 else "extension"
        sign = 1.0 if dot(swing_axis, lateral_axis) >= 0.0 else -1.0
        signed = swing_angle * sign
        return clamp_angle(signed, self.LATERAL_BEND_LIMIT, self.LATERAL_BEND_LIMIT), "lateral"

    def _pressure(self, swing_kind: str, constrained_swing: float, requested_swing: float, twist_angle: float) -> tuple[float, str]:
        if swing_kind == "flexion":
            swing_limit = self.FLEXION_LIMIT
        elif swing_kind == "extension":
            swing_limit = self.EXTENSION_LIMIT
        elif swing_kind == "lateral":
            swing_limit = self.LATERAL_BEND_LIMIT
        else:
            swing_limit = self.FLEXION_LIMIT
        swing_pressure, swing_state = pressure_for_angle(requested_swing, swing_limit)
        twist_pressure, twist_state = pressure_for_angle(twist_angle, self.AXIAL_ROTATION_LIMIT)
        pressure = max(swing_pressure, twist_pressure)
        state = "stretch" if "stretch" in {swing_state, twist_state} else "normal"
        if abs(constrained_swing) < requested_swing - 1e-6:
            state = "stretch"
            pressure = max(pressure, 0.5)
        return pressure, state


def _fallback_axis(axis: Vec3, fallback: Vec3) -> Vec3:
    normalized = normalize(axis)
    return normalized if length(normalized) > 1e-6 else fallback


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
