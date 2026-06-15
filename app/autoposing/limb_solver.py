from __future__ import annotations

from dataclasses import dataclass

from core.ik.two_bone import TwoBoneIkSolution, solve_two_bone_ik
from core.math import subtract
from core.skeleton.models import Vec3

from .anatomy_profile import HumanoidAnatomyProfile, LimbChainProfile
from .constraints import PoseConstraintSet
from .joint_rules.elbow import ElbowRule, ElbowRuleResult
from .joint_rules.knee import KneeRule, KneeRuleResult
from .reach_constraints import ReachConstraintResult, clamp_target_to_reach


@dataclass(slots=True)
class SolvedLimb:
    chain: LimbChainProfile
    solution: TwoBoneIkSolution
    reach: ReachConstraintResult
    elbow_rule: ElbowRuleResult | None = None
    knee_rule: KneeRuleResult | None = None


class LimbSolver:
    def __init__(self) -> None:
        self._elbow_rule = ElbowRule()
        self._knee_rule = KneeRule()

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
        target = reach.clamped_target
        hint = constraints.bend_hint_for(chain.hint_controller_id, fallback_hint)
        elbow_rule = None
        knee_rule = None
        if chain.name.endswith("_arm"):
            elbow_rule = self._elbow_rule.solve(
                start_position=chain_start,
                target_position=target,
                raw_hint_position=hint,
                fallback_hint_position=fallback_hint,
                upper_length=upper_length,
                lower_length=lower_length,
            )
            target = elbow_rule.target_position
            hint = elbow_rule.hint_position
            reach = self._merge_elbow_target_limit(reach, elbow_rule)
        elif chain.name.endswith("_leg"):
            knee_rule = self._knee_rule.solve(
                start_position=chain_start,
                target_position=target,
                raw_hint_position=hint,
                fallback_hint_position=fallback_hint,
                upper_length=upper_length,
                lower_length=lower_length,
            )
            target = knee_rule.target_position
            hint = knee_rule.hint_position
            reach = self._merge_knee_target_limit(reach, knee_rule)

        solution = solve_two_bone_ik(chain_start, target, upper_length, lower_length, hint)
        return SolvedLimb(chain=chain, solution=solution, reach=reach, elbow_rule=elbow_rule, knee_rule=knee_rule)

    def _merge_elbow_target_limit(
        self,
        reach: ReachConstraintResult,
        elbow_rule: ElbowRuleResult,
    ) -> ReachConstraintResult:
        if not elbow_rule.target_clamped:
            return reach

        pressure = max(reach.pressure, elbow_rule.pressure)
        pressure_state = "stretch" if "stretch" in {reach.pressure_state, elbow_rule.pressure_state} else reach.pressure_state
        return ReachConstraintResult(
            target=reach.target,
            clamped_target=elbow_rule.target_position,
            overreach_vector=subtract(reach.target, elbow_rule.target_position),
            pressure=pressure,
            pressure_state=pressure_state,
            clamped=True,
        )

    def _merge_knee_target_limit(
        self,
        reach: ReachConstraintResult,
        knee_rule: KneeRuleResult,
    ) -> ReachConstraintResult:
        if not knee_rule.target_clamped:
            return reach

        pressure = max(reach.pressure, knee_rule.pressure)
        pressure_state = "stretch" if "stretch" in {reach.pressure_state, knee_rule.pressure_state} else reach.pressure_state
        return ReachConstraintResult(
            target=reach.target,
            clamped_target=knee_rule.target_position,
            overreach_vector=subtract(reach.target, knee_rule.target_position),
            pressure=pressure,
            pressure_state=pressure_state,
            clamped=True,
        )
