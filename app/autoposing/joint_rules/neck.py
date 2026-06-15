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


class NeckHeadRule:
    CERVICAL_FLEXION_LIMIT = radians(45.0)
    CERVICAL_EXTENSION_LIMIT = radians(45.0)
    CERVICAL_LATERAL_LIMIT = radians(45.0)
    CERVICAL_ROTATION_LIMIT = radians(80.0)
    HEAD_ROLL_UPRIGHT_LIMIT = radians(20.0)
    HEAD_ROLL_DIRECTED_LIMIT = radians(45.0)

    def solve_neck(
        self,
        local_delta,
        neck_axis: Vec3,
        *,
        flexion_axis: Vec3 = (1.0, 0.0, 0.0),
        lateral_axis: Vec3 = (0.0, 0.0, 1.0),
    ) -> JointRuleResult:
        return self._solve(
            local_delta,
            neck_axis,
            flexion_axis=flexion_axis,
            lateral_axis=lateral_axis,
            axial_limit=self.CERVICAL_ROTATION_LIMIT,
            roll_limit=None,
            label="neck",
        )

    def solve_head(
        self,
        local_delta,
        head_axis: Vec3,
        *,
        flexion_axis: Vec3 = (1.0, 0.0, 0.0),
        lateral_axis: Vec3 = (0.0, 0.0, 1.0),
        directed: bool,
    ) -> JointRuleResult:
        return self._solve(
            local_delta,
            head_axis,
            flexion_axis=flexion_axis,
            lateral_axis=lateral_axis,
            axial_limit=self.CERVICAL_ROTATION_LIMIT,
            roll_limit=self.HEAD_ROLL_DIRECTED_LIMIT if directed else self.HEAD_ROLL_UPRIGHT_LIMIT,
            label="head",
        )

    def _solve(
        self,
        local_delta,
        axis: Vec3,
        *,
        flexion_axis: Vec3,
        lateral_axis: Vec3,
        axial_limit: float,
        roll_limit: float | None,
        label: str,
    ) -> JointRuleResult:
        axis = _fallback_axis(axis, (0.0, 1.0, 0.0))
        flexion_axis = _fallback_axis(flexion_axis, (1.0, 0.0, 0.0))
        lateral_axis = _fallback_axis(lateral_axis, (0.0, 0.0, 1.0))

        twist = extract_twist_rotation(local_delta, axis)
        swing_delta = quaternion_multiply(local_delta, inverse_quaternion(twist))
        swing_axis, swing_angle = _axis_angle(swing_delta)
        constrained_swing, swing_kind = self._constrain_swing(swing_axis, swing_angle, flexion_axis, lateral_axis)
        twist_angle = signed_twist_angle(twist, axis)
        twist_limit = roll_limit if roll_limit is not None else axial_limit
        constrained_twist = clamp_angle(twist_angle, twist_limit, twist_limit)

        swing_rotation = quaternion_angle_axis(abs(constrained_swing), swing_axis)
        twist_rotation = quaternion_angle_axis(constrained_twist, axis)
        swing_pressure, swing_state = self._swing_pressure(swing_kind, swing_angle)
        twist_pressure, twist_state = pressure_for_angle(twist_angle, twist_limit)
        pressure = max(swing_pressure, twist_pressure)
        state = "stretch" if "stretch" in {swing_state, twist_state} or abs(constrained_twist) < abs(twist_angle) - 1e-6 else "normal"
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
            return clamp_angle(signed, self.CERVICAL_EXTENSION_LIMIT, self.CERVICAL_FLEXION_LIMIT), "flexion" if signed >= 0.0 else "extension"
        sign = 1.0 if dot(swing_axis, lateral_axis) >= 0.0 else -1.0
        signed = swing_angle * sign
        return clamp_angle(signed, self.CERVICAL_LATERAL_LIMIT, self.CERVICAL_LATERAL_LIMIT), "lateral"

    def _swing_pressure(self, swing_kind: str, swing_angle: float) -> tuple[float, str]:
        if swing_kind == "extension":
            limit = self.CERVICAL_EXTENSION_LIMIT
        elif swing_kind == "lateral":
            limit = self.CERVICAL_LATERAL_LIMIT
        else:
            limit = self.CERVICAL_FLEXION_LIMIT
        return pressure_for_angle(swing_angle, limit)


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
