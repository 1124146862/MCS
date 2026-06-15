from __future__ import annotations

from core.math import add, distance, normalize, scale, subtract
from core.skeleton.models import Vec3

from .anatomy_profile import HumanoidAnatomyProfile


class SpineSolver:
    def solve_spine_and_head(
        self,
        anatomy: HumanoidAnatomyProfile,
        pelvis: Vec3,
        chest_target: Vec3,
        head_target: Vec3,
    ) -> dict[str, Vec3]:
        lower_names = anatomy.spine_chain
        upper_names = anatomy.neck_chain
        if not anatomy.chain_available(lower_names) or not anatomy.chain_available(upper_names):
            return {
                "pelvis": pelvis,
                "spine_05": chest_target,
                "head": head_target,
            }

        lower_lengths = [anatomy.length(*pair) for pair in zip(lower_names[:-1], lower_names[1:], strict=True)]
        upper_lengths = [anatomy.length(*pair) for pair in zip(upper_names[:-1], upper_names[1:], strict=True)]
        lower_rest_positions = [anatomy.rest(name) for name in lower_names]
        upper_rest_positions = [anatomy.rest(name) for name in upper_names]
        lower_chain = self.solve_linear_chain(lower_rest_positions, pelvis, chest_target, lower_lengths)
        chest = lower_chain[-1]
        upper_chain = self.solve_linear_chain(upper_rest_positions, chest, head_target, upper_lengths)
        positions = {name: lower_chain[index] for index, name in enumerate(lower_names)}
        positions.update({name: upper_chain[index] for index, name in enumerate(upper_names)})
        return positions

    def solve_linear_chain(
        self,
        rest_positions: list[Vec3],
        start: Vec3,
        end: Vec3,
        segment_lengths: list[float],
    ) -> list[Vec3]:
        total_length = sum(segment_lengths)
        if not rest_positions:
            return [start, end]

        start_delta = subtract(start, rest_positions[0])
        joints = [add(position, start_delta) for position in rest_positions]
        joints[0] = start

        straight_distance = distance(start, end)
        if straight_distance >= total_length - 1e-6:
            direction = normalize(subtract(end, start))
            if direction == (0.0, 0.0, 0.0):
                direction = (0.0, 1.0, 0.0)
            current = start
            stretched = [start]
            for segment_length in segment_lengths:
                current = add(current, scale(direction, segment_length))
                stretched.append(current)
            return stretched

        tolerance = 0.001
        for _ in range(12):
            joints[-1] = end
            for index in range(len(segment_lengths) - 1, -1, -1):
                link_distance = max(distance(joints[index + 1], joints[index]), 1e-6)
                factor = segment_lengths[index] / link_distance
                joints[index] = add(
                    scale(joints[index + 1], 1.0 - factor),
                    scale(joints[index], factor),
                )

            joints[0] = start
            for index, segment_length in enumerate(segment_lengths):
                link_distance = max(distance(joints[index + 1], joints[index]), 1e-6)
                factor = segment_length / link_distance
                joints[index + 1] = add(
                    scale(joints[index], 1.0 - factor),
                    scale(joints[index + 1], factor),
                )

            if distance(joints[-1], end) <= tolerance:
                break

        return joints
