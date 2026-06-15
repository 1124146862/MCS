from __future__ import annotations

from dataclasses import dataclass

from core.math import add, distance, length, lerp, normalize, rotation_between_vectors, scale, subtract
from core.skeleton.models import Vec3

from .anatomy_profile import HumanoidAnatomyProfile
from .constraints import PoseConstraintSet
from .joint_rules import JointRuleResult, NeckHeadRule, SpineRule


@dataclass(frozen=True, slots=True)
class SpineSolveResult:
    positions: dict[str, Vec3]
    rule_results: dict[str, JointRuleResult]


class SpineSolver:
    def __init__(self) -> None:
        self._spine_rule = SpineRule()
        self._neck_rule = NeckHeadRule()

    def solve_spine_and_head(
        self,
        anatomy: HumanoidAnatomyProfile,
        constraints: PoseConstraintSet,
        pelvis: Vec3,
        chest_target: Vec3,
        head_target: Vec3,
    ) -> SpineSolveResult:
        lower_names = anatomy.spine_chain
        upper_names = anatomy.neck_chain
        pelvis = constraints.position_for("pelvis_main", pelvis)
        chest_target = constraints.position_for("chest_secondary", chest_target)
        head_target = constraints.position_for("head_main", head_target)

        if not anatomy.chain_available(lower_names) or not anatomy.chain_available(upper_names):
            positions = {"pelvis": pelvis, "spine_05": chest_target, "head": head_target}
            return SpineSolveResult(positions=positions, rule_results={})

        lower_lengths = [anatomy.length(*pair) for pair in zip(lower_names[:-1], lower_names[1:], strict=True)]
        upper_lengths = [anatomy.length(*pair) for pair in zip(upper_names[:-1], upper_names[1:], strict=True)]
        lower_rest_positions = [anatomy.rest(name) for name in lower_names]
        upper_rest_positions = [anatomy.rest(name) for name in upper_names]
        lower_chain = self.solve_linear_chain(
            lower_rest_positions,
            pelvis,
            chest_target,
            lower_lengths,
            self._lower_guides(lower_names, constraints, anatomy),
        )
        chest = lower_chain[-1]
        upper_chain = self.solve_linear_chain(
            upper_rest_positions,
            chest,
            head_target,
            upper_lengths,
            self._upper_guides(upper_names, constraints, anatomy),
        )
        positions = {name: lower_chain[index] for index, name in enumerate(lower_names)}
        positions.update({name: upper_chain[index] for index, name in enumerate(upper_names)})
        return SpineSolveResult(positions=positions, rule_results=self._rule_results(anatomy, positions))

    def solve_linear_chain(
        self,
        rest_positions: list[Vec3],
        start: Vec3,
        end: Vec3,
        segment_lengths: list[float],
        guides: dict[int, tuple[Vec3, float]] | None = None,
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

        guides = guides or {}
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

            self._apply_guides(joints, guides)

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

        return joints

    def _lower_guides(
        self,
        lower_names: tuple[str, ...],
        constraints: PoseConstraintSet,
        anatomy: HumanoidAnatomyProfile,
    ) -> dict[int, tuple[Vec3, float]]:
        guides: dict[int, tuple[Vec3, float]] = {}
        if constraints.engaged("stomach_secondary"):
            index = self._guide_index(lower_names, constraints.by_controller_id["stomach_secondary"].joint_name, fallback=len(lower_names) // 2)
            guides[index] = (constraints.position_for("stomach_secondary", anatomy.rest(lower_names[index])), self._guide_weight(constraints, "stomach_secondary"))
        return guides

    def _upper_guides(
        self,
        upper_names: tuple[str, ...],
        constraints: PoseConstraintSet,
        anatomy: HumanoidAnatomyProfile,
    ) -> dict[int, tuple[Vec3, float]]:
        guides: dict[int, tuple[Vec3, float]] = {}
        if constraints.engaged("neck_lesser"):
            index = self._guide_index(upper_names, constraints.by_controller_id["neck_lesser"].joint_name, fallback=1)
            guides[index] = (constraints.position_for("neck_lesser", anatomy.rest(upper_names[index])), self._guide_weight(constraints, "neck_lesser"))
        return guides

    @staticmethod
    def _guide_index(names: tuple[str, ...], joint_name: str, *, fallback: int) -> int:
        if joint_name in names:
            return names.index(joint_name)
        return min(max(fallback, 1), len(names) - 2)

    @staticmethod
    def _guide_weight(constraints: PoseConstraintSet, controller_id: str) -> float:
        if constraints.fixed(controller_id):
            return 0.98
        if constraints.locked(controller_id):
            return 0.86
        return min(max(constraints.pose_intent_weight(controller_id), constraints.preservation_weight(controller_id), 0.35), 0.75)

    @staticmethod
    def _apply_guides(joints: list[Vec3], guides: dict[int, tuple[Vec3, float]]) -> None:
        for index, (target, weight) in guides.items():
            if index <= 0 or index >= len(joints) - 1:
                continue
            joints[index] = lerp(joints[index], target, max(0.0, min(1.0, weight)))

    def _rule_results(self, anatomy: HumanoidAnatomyProfile, positions: dict[str, Vec3]) -> dict[str, JointRuleResult]:
        results: dict[str, JointRuleResult] = {}
        if "pelvis" in positions and "spine_05" in positions:
            rest_axis = subtract(anatomy.rest("spine_05"), anatomy.rest("pelvis"))
            solved_axis = subtract(positions["spine_05"], positions["pelvis"])
            if length(rest_axis) > 1e-6 and length(solved_axis) > 1e-6:
                results["spine"] = self._spine_rule.solve(
                    rotation_between_vectors(rest_axis, solved_axis),
                    anatomy.up_axis,
                    flexion_axis=anatomy.side_axis,
                    lateral_axis=anatomy.forward_axis,
                )
        if "spine_05" in positions and "head" in positions:
            rest_axis = subtract(anatomy.rest("head"), anatomy.rest("spine_05"))
            solved_axis = subtract(positions["head"], positions["spine_05"])
            if length(rest_axis) > 1e-6 and length(solved_axis) > 1e-6:
                results["neck"] = self._neck_rule.solve_neck(
                    rotation_between_vectors(rest_axis, solved_axis),
                    anatomy.up_axis,
                    flexion_axis=anatomy.side_axis,
                    lateral_axis=anatomy.forward_axis,
                )
        return results
