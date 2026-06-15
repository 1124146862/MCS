from __future__ import annotations

from dataclasses import dataclass

from core.ik.two_bone import TwoBoneIkSolution, solve_two_bone_ik
from core.skeleton.models import Vec3

from .anatomy_profile import HumanoidAnatomyProfile, LimbChainProfile
from .constraints import PoseConstraintSet
from .reach_constraints import ReachConstraintResult, clamp_target_to_reach


@dataclass(slots=True)
class SolvedLimb:
    chain: LimbChainProfile
    solution: TwoBoneIkSolution
    reach: ReachConstraintResult


class LimbSolver:
    def solve(
        self,
        anatomy: HumanoidAnatomyProfile,
        constraints: PoseConstraintSet,
        chain: LimbChainProfile,
        *,
        chain_start: Vec3,
        end_target: Vec3,
        fallback_hint: Vec3,
    ) -> SolvedLimb:
        upper_length = anatomy.length(chain.start_name, chain.mid_name)
        lower_length = anatomy.length(chain.mid_name, chain.end_name)
        max_reach = upper_length + lower_length - 0.001
        reach = clamp_target_to_reach(chain_start, end_target, max_reach)
        hint = constraints.bend_hint_for(chain.hint_controller_id, fallback_hint)
        solution = solve_two_bone_ik(chain_start, reach.clamped_target, upper_length, lower_length, hint)
        return SolvedLimb(chain=chain, solution=solution, reach=reach)
