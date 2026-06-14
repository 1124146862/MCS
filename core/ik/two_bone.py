from dataclasses import dataclass
import math


EPSILON = 1e-6


@dataclass(slots=True)
class TwoBoneIkSolution:
    start_position: tuple[float, float, float]
    mid_position: tuple[float, float, float]
    end_position: tuple[float, float, float]


def add(
    left: tuple[float, float, float],
    right: tuple[float, float, float],
) -> tuple[float, float, float]:
    return (
        left[0] + right[0],
        left[1] + right[1],
        left[2] + right[2],
    )


def subtract(
    left: tuple[float, float, float],
    right: tuple[float, float, float],
) -> tuple[float, float, float]:
    return (
        left[0] - right[0],
        left[1] - right[1],
        left[2] - right[2],
    )


def scale(
    vector: tuple[float, float, float],
    factor: float,
) -> tuple[float, float, float]:
    return (
        vector[0] * factor,
        vector[1] * factor,
        vector[2] * factor,
    )


def dot(
    left: tuple[float, float, float],
    right: tuple[float, float, float],
) -> float:
    return left[0] * right[0] + left[1] * right[1] + left[2] * right[2]


def cross(
    left: tuple[float, float, float],
    right: tuple[float, float, float],
) -> tuple[float, float, float]:
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


def length(vector: tuple[float, float, float]) -> float:
    return math.sqrt(dot(vector, vector))


def distance(
    left: tuple[float, float, float],
    right: tuple[float, float, float],
) -> float:
    return length(subtract(left, right))


def normalize(vector: tuple[float, float, float]) -> tuple[float, float, float]:
    vector_length = length(vector)
    if vector_length <= EPSILON:
        return (0.0, 0.0, 0.0)
    return (
        vector[0] / vector_length,
        vector[1] / vector_length,
        vector[2] / vector_length,
    )


def lerp(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    factor: float,
) -> tuple[float, float, float]:
    return (
        start[0] + (end[0] - start[0]) * factor,
        start[1] + (end[1] - start[1]) * factor,
        start[2] + (end[2] - start[2]) * factor,
    )


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def safe_perpendicular(
    direction: tuple[float, float, float],
) -> tuple[float, float, float]:
    up = (0.0, 1.0, 0.0)
    perpendicular = cross(direction, up)
    if length(perpendicular) <= EPSILON:
        perpendicular = cross(direction, (1.0, 0.0, 0.0))
    return normalize(perpendicular)


def solve_two_bone_ik(
    start_position: tuple[float, float, float],
    target_position: tuple[float, float, float],
    upper_length: float,
    lower_length: float,
    bend_hint_position: tuple[float, float, float],
) -> TwoBoneIkSolution:
    to_target = subtract(target_position, start_position)
    raw_distance = length(to_target)

    direction = normalize(to_target)
    if length(direction) <= EPSILON:
        direction = normalize(subtract(bend_hint_position, start_position))
    if length(direction) <= EPSILON:
        direction = (1.0, 0.0, 0.0)

    min_reach = max(abs(upper_length - lower_length) + 0.001, 0.001)
    max_reach = max(upper_length + lower_length - 0.001, min_reach)
    solved_distance = clamp(raw_distance, min_reach, max_reach)

    bend_hint = subtract(bend_hint_position, start_position)
    plane_normal = cross(direction, bend_hint)
    if length(plane_normal) <= EPSILON:
        plane_normal = safe_perpendicular(direction)
    else:
        plane_normal = normalize(plane_normal)

    bend_direction = cross(plane_normal, direction)
    if length(bend_direction) <= EPSILON:
        bend_direction = safe_perpendicular(direction)
    else:
        bend_direction = normalize(bend_direction)

    projected_distance = (upper_length * upper_length - lower_length * lower_length + solved_distance * solved_distance) / (
        2.0 * solved_distance
    )
    bend_height = math.sqrt(max(upper_length * upper_length - projected_distance * projected_distance, 0.0))

    solved_end = add(start_position, scale(direction, solved_distance))
    solved_mid = add(
        add(start_position, scale(direction, projected_distance)),
        scale(bend_direction, bend_height),
    )

    if min_reach <= raw_distance <= max_reach:
        solved_end = target_position

    return TwoBoneIkSolution(
        start_position=start_position,
        mid_position=solved_mid,
        end_position=solved_end,
    )
