from __future__ import annotations

import math

from core.math import extract_twist_rotation, inverse_quaternion, quaternion_angle_axis, quaternion_multiply

from core.skeleton.models import Vec3

from .common import JointRuleResult, degrees, pressure_for_angle, radians, signed_twist_angle


class WristRule:
    EXTENSION_LIMIT = radians(70.0)
    FLEXION_LIMIT = radians(75.0)
    RADIAL_DEVIATION_LIMIT = radians(20.0)
    ULNAR_DEVIATION_LIMIT = radians(35.0)
    AXIAL_TWIST_COMFORT = radians(10.0)
    AXIAL_TWIST_LIMIT = radians(15.0)

    def solve(self, hand_local_delta, forearm_axis: Vec3, forearm_overflow_twist: float) -> JointRuleResult:
        original_twist = extract_twist_rotation(hand_local_delta, forearm_axis)
        swing_delta = quaternion_multiply(hand_local_delta, inverse_quaternion(original_twist))
        wrist_twist = max(-self.AXIAL_TWIST_LIMIT, min(self.AXIAL_TWIST_LIMIT, forearm_overflow_twist))
        adjusted_delta = quaternion_multiply(swing_delta, quaternion_angle_axis(wrist_twist, forearm_axis))

        axial_pressure, axial_state = pressure_for_angle(forearm_overflow_twist, self.AXIAL_TWIST_LIMIT, comfort_ratio=2.0 / 3.0)
        flexion_pressure, flexion_state = self._swing_pressure(swing_delta, forearm_axis)
        pressure = max(axial_pressure, flexion_pressure)
        state = "stretch" if "stretch" in {axial_state, flexion_state} else "normal"
        return JointRuleResult(
            rotation=adjusted_delta,
            pressure=pressure,
            pressure_state=state,
            overflow_twist=forearm_overflow_twist - wrist_twist,
            debug_angles={
                "wrist_axial_degrees": degrees(wrist_twist),
                "wrist_axial_requested_degrees": degrees(forearm_overflow_twist),
                "wrist_axial_overflow_degrees": degrees(forearm_overflow_twist - wrist_twist),
                "wrist_swing_pressure": flexion_pressure,
            },
        )

    def _swing_pressure(self, swing_delta, forearm_axis: Vec3) -> tuple[float, str]:
        # First pass: approximate wrist flexion/deviation stress from swing magnitude.
        # Axis-specific labels can be made more exact once anatomical hand frames are added.
        twist = extract_twist_rotation(swing_delta, forearm_axis)
        pure_swing = quaternion_multiply(swing_delta, inverse_quaternion(twist))
        swing_angle = abs(2.0 * math.acos(max(-1.0, min(1.0, pure_swing[0]))))
        return pressure_for_angle(swing_angle, max(self.EXTENSION_LIMIT, self.FLEXION_LIMIT, self.ULNAR_DEVIATION_LIMIT))
