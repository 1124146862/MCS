from dataclasses import dataclass

from core.math import EPSILON, add, clamp, distance, normalize, scale, subtract
from core.skeleton.models import Vec3


@dataclass(slots=True)
class ReachConstraintResult:
    target: Vec3
    clamped_target: Vec3
    overreach_vector: Vec3
    pressure: float
    pressure_state: str
    clamped: bool


def clamp_target_to_reach(start_position: Vec3, target_position: Vec3, max_reach: float) -> ReachConstraintResult:
    safe_reach = max(max_reach, EPSILON)
    delta = subtract(target_position, start_position)
    raw_distance = distance(start_position, target_position)
    clamped = raw_distance > safe_reach

    if raw_distance <= EPSILON:
        return ReachConstraintResult(
            target=target_position,
            clamped_target=target_position,
            overreach_vector=(0.0, 0.0, 0.0),
            pressure=0.0,
            pressure_state="normal",
            clamped=False,
        )

    direction = normalize(delta)
    clamped_distance = clamp(raw_distance, 0.0, safe_reach)
    clamped_target = add(start_position, scale(direction, clamped_distance))
    overreach_vector = subtract(target_position, clamped_target)
    normalized_distance = raw_distance / safe_reach

    if clamped:
        pressure_state = "stretch"
        pressure = max(0.0, normalized_distance - 1.0)
    elif normalized_distance < 0.55:
        pressure_state = "squeeze"
        pressure = 1.0 - normalized_distance / 0.55
    else:
        pressure_state = "normal"
        pressure = 0.0

    return ReachConstraintResult(
        target=target_position,
        clamped_target=clamped_target,
        overreach_vector=overreach_vector,
        pressure=pressure,
        pressure_state=pressure_state,
        clamped=clamped,
    )
