from __future__ import annotations

import math
from dataclasses import dataclass, field

from core.math import Quat, Vec3, clamp, dot, normalize, normalize_quaternion


@dataclass(frozen=True, slots=True)
class JointRuleResult:
    rotation: Quat
    pressure: float = 0.0
    pressure_state: str = "normal"
    overflow_twist: float = 0.0
    debug_angles: dict[str, float] = field(default_factory=dict)


def signed_twist_angle(rotation: Quat, axis: Vec3) -> float:
    w, x, y, z = normalize_quaternion(rotation)
    normalized_axis = normalize(axis)
    if normalized_axis == (0.0, 0.0, 0.0):
        return 0.0
    vector_dot = dot((x, y, z), normalized_axis)
    angle = 2.0 * math.atan2(abs(vector_dot), w)
    if angle > math.pi:
        angle -= math.tau
    return angle if vector_dot >= 0.0 else -angle


def pressure_for_angle(angle: float, soft_limit: float, comfort_ratio: float = 0.75) -> tuple[float, str]:
    if soft_limit <= 0.0:
        return 0.0, "normal"
    ratio = abs(angle) / soft_limit
    if ratio <= comfort_ratio:
        return 0.0, "normal"
    if ratio <= 1.0:
        t = (ratio - comfort_ratio) / (1.0 - comfort_ratio)
        pressure = t * t * (3.0 - 2.0 * t) * 0.5
        return pressure, "normal"
    pressure = 0.5 + min(0.5, (ratio - 1.0) * 0.75)
    return pressure, "stretch"


def degrees(value_radians: float) -> float:
    return math.degrees(value_radians)


def radians(value_degrees: float) -> float:
    return math.radians(value_degrees)


def clamp_angle(value: float, negative_limit: float, positive_limit: float) -> float:
    return clamp(value, -abs(negative_limit), abs(positive_limit))
