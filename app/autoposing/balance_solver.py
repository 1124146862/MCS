from __future__ import annotations

from dataclasses import dataclass

from core.math import add, distance, length, scale, subtract
from core.skeleton.models import Vec3

from .anatomy_profile import HumanoidAnatomyProfile
from .constraints import PoseConstraintSet
from .support_solver import SupportState


@dataclass(frozen=True, slots=True)
class BalanceCorrection:
    pelvis: Vec3
    chest: Vec3
    head: Vec3
    com_error: float = 0.0
    applied: Vec3 = (0.0, 0.0, 0.0)


class SupportBalanceSolver:
    def apply(
        self,
        anatomy: HumanoidAnatomyProfile,
        constraints: PoseConstraintSet,
        state: SupportState,
        solved_positions: dict[str, Vec3],
        pelvis: Vec3,
        chest: Vec3,
        head: Vec3,
    ) -> BalanceCorrection:
        stable_support = {
            name: target
            for name, target in state.support_targets.items()
            if state.support_weights.get(name, 0.0) >= 0.7
        }
        if not stable_support:
            return BalanceCorrection(pelvis, chest, head)

        com = self.estimate_com(anatomy, solved_positions)
        support_center = self._support_center(stable_support)
        error = (support_center[0] - com[0], 0.0, support_center[2] - com[2])
        error_length = length(error)
        dead_zone = anatomy.body_height * (0.06 if len(stable_support) >= 3 else 0.035)
        if error_length <= dead_zone:
            return BalanceCorrection(pelvis, chest, head, com_error=0.0)

        correction = self._cap(scale(error, 0.28), anatomy.body_height * 0.035)
        adjusted_pelvis = pelvis if constraints.locked("pelvis_main") else add(pelvis, scale(correction, 0.55))
        adjusted_chest = chest if constraints.locked("chest_secondary") else add(chest, scale(correction, 0.25))
        adjusted_head = head if constraints.locked("head_main") else add(head, scale(correction, 0.10))
        return BalanceCorrection(adjusted_pelvis, adjusted_chest, adjusted_head, com_error=error_length, applied=correction)

    def estimate_com(self, anatomy: HumanoidAnatomyProfile, solved_positions: dict[str, Vec3]) -> Vec3:
        weighted = (0.0, 0.0, 0.0)
        total = 0.0

        def add_segment(name: str, start: str, end: str, mass: float, ratio: float) -> None:
            nonlocal weighted, total
            start_position = solved_positions.get(start, anatomy.rest(start))
            end_position = solved_positions.get(end, anatomy.rest(end))
            point = add(start_position, scale(subtract(end_position, start_position), ratio))
            weighted = add(weighted, scale(point, mass))
            total += mass

        pelvis = solved_positions.get("pelvis", anatomy.rest("pelvis"))
        chest = solved_positions.get("spine_05", anatomy.rest("spine_05"))
        head = solved_positions.get("head", anatomy.rest("head"))
        trunk = add(add(scale(pelvis, 0.45), scale(chest, 0.45)), scale(head, 0.10))
        weighted = add(weighted, scale(trunk, 0.43))
        total += 0.43
        weighted = add(weighted, scale(head, 0.08))
        total += 0.08

        for side in ("l", "r"):
            add_segment(f"upperarm_{side}", f"upperarm_{side}", f"lowerarm_{side}", 0.028, 0.47)
            add_segment(f"forearm_{side}", f"lowerarm_{side}", f"hand_{side}", 0.018, 0.41)
            hand = solved_positions.get(f"hand_{side}", anatomy.rest(f"hand_{side}"))
            weighted = add(weighted, scale(hand, 0.007))
            total += 0.007
            add_segment(f"thigh_{side}", f"thigh_{side}", f"calf_{side}", 0.10, 0.39)
            add_segment(f"calf_{side}", f"calf_{side}", f"foot_{side}", 0.046, 0.41)
            add_segment(f"foot_{side}", f"foot_{side}", f"ball_{side}", 0.014, 0.50)

        if total <= 0.0:
            return pelvis
        return scale(weighted, 1.0 / total)

    def _support_center(self, support_targets: dict[str, Vec3]) -> Vec3:
        total = (0.0, 0.0, 0.0)
        for target in support_targets.values():
            total = add(total, target)
        return scale(total, 1.0 / max(len(support_targets), 1))

    @staticmethod
    def _cap(vector: Vec3, max_length: float) -> Vec3:
        vector_length = distance((0.0, 0.0, 0.0), vector)
        if vector_length <= max_length or vector_length <= 0.0:
            return vector
        return scale(vector, max_length / vector_length)
