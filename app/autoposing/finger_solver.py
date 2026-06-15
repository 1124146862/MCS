from __future__ import annotations

from dataclasses import dataclass

from core.ik.two_bone import solve_two_bone_ik
from core.math import add, distance, length, normalize, scale, subtract
from core.skeleton.models import Vec3

from .constraints import PoseConstraintSet
from .finger_profile import FingerAnatomyProfile, FingerChainProfile
from .reach_constraints import ReachConstraintResult, clamp_target_to_reach


@dataclass(frozen=True, slots=True)
class SolvedFingerChain:
    chain: FingerChainProfile
    joint_positions: dict[str, Vec3]
    tip_position: Vec3
    pre_tip_position: Vec3
    reach: ReachConstraintResult


class FingerPoseSolver:
    def solve(
        self,
        profile: FingerAnatomyProfile,
        constraints: PoseConstraintSet,
        solved_body_positions: dict[str, Vec3],
    ) -> dict[str, SolvedFingerChain]:
        solved: dict[str, SolvedFingerChain] = {}
        for chain in profile.chains.values():
            solved[chain.key] = self._solve_chain(profile, constraints, chain, solved_body_positions)
        return solved

    def _solve_chain(
        self,
        profile: FingerAnatomyProfile,
        constraints: PoseConstraintSet,
        chain: FingerChainProfile,
        solved_body_positions: dict[str, Vec3],
    ) -> SolvedFingerChain:
        rest_points = [profile.rest(name) for name in chain.joint_names]
        if len(rest_points) < 2:
            reach = clamp_target_to_reach(chain.tip_world_position, chain.tip_world_position, 0.001)
            return SolvedFingerChain(chain, {}, chain.tip_world_position, chain.pre_tip_world_position, reach)

        hand_position = solved_body_positions.get(chain.hand_name, profile.rest(chain.hand_name))
        root_position = add(hand_position, subtract(rest_points[0], profile.rest(chain.hand_name)))
        default_tip = add(hand_position, subtract(chain.tip_world_position, profile.rest(chain.hand_name)))
        default_pre_tip = add(hand_position, subtract(chain.pre_tip_world_position, profile.rest(chain.hand_name)))

        tip_constraint = constraints.finger_tip_for(chain.tip_controller_id)
        bend_constraint = constraints.finger_bend_hint_for(chain.pre_tip_controller_id)
        target = tip_constraint.target_world_position if tip_constraint is not None else default_tip
        hint = bend_constraint.target_world_position if bend_constraint is not None else default_pre_tip
        if chain.opposable:
            hint = add(hint, scale(chain.opposition_axis, 3.5 if bend_constraint is not None else 1.5))

        segment_lengths = self._segment_lengths(profile, chain)
        max_reach = sum(segment_lengths) - 0.001
        if chain.opposable and tip_constraint is not None:
            max_reach = max(max_reach * 0.88, 0.001)
        reach = clamp_target_to_reach(root_position, target, max_reach)
        joint_positions = self._solve_positions(chain, root_position, reach.clamped_target, hint, segment_lengths, rest_points)
        pre_tip_position = bend_constraint.target_world_position if bend_constraint is not None else default_pre_tip
        return SolvedFingerChain(
            chain=chain,
            joint_positions=joint_positions,
            tip_position=reach.clamped_target,
            pre_tip_position=pre_tip_position,
            reach=reach,
        )

    def _segment_lengths(self, profile: FingerAnatomyProfile, chain: FingerChainProfile) -> list[float]:
        lengths = [
            max(distance(profile.rest(start), profile.rest(end)), 0.001)
            for start, end in zip(chain.joint_names, chain.joint_names[1:], strict=False)
        ]
        distal = profile.rest(chain.joint_names[-1])
        previous = profile.rest(chain.joint_names[-2])
        lengths.append(max(distance(distal, chain.tip_world_position), distance(previous, distal) * 0.55, 0.001))
        return lengths

    def _solve_positions(
        self,
        chain: FingerChainProfile,
        root_position: Vec3,
        tip_target: Vec3,
        hint_position: Vec3,
        segment_lengths: list[float],
        rest_points: list[Vec3],
    ) -> dict[str, Vec3]:
        if len(chain.joint_names) == 2:
            solution = solve_two_bone_ik(root_position, tip_target, segment_lengths[0], segment_lengths[1], hint_position)
            return {
                chain.joint_names[0]: root_position,
                chain.joint_names[1]: solution.mid_position,
            }

        upper_length = segment_lengths[0]
        lower_length = max(sum(segment_lengths[1:]), 0.001)
        solution = solve_two_bone_ik(root_position, tip_target, upper_length, lower_length, hint_position)
        axis = normalize(subtract(tip_target, solution.mid_position))
        if length(axis) <= 0.0:
            axis = normalize(subtract(rest_points[-1], rest_points[-2]))
        if length(axis) <= 0.0:
            axis = (0.0, -1.0, 0.0)

        positions: dict[str, Vec3] = {
            chain.joint_names[0]: root_position,
            chain.joint_names[1]: solution.mid_position,
        }
        current = solution.mid_position
        for joint_name, segment_length in zip(chain.joint_names[2:], segment_lengths[1:-1], strict=False):
            current = add(current, scale(axis, segment_length))
            positions[joint_name] = current
        return positions
