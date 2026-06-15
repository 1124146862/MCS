from __future__ import annotations

from core.math import add, scale, subtract
from core.rig.models import JointModel, RigModel
from core.skeleton.models import BoneModel, SkeletonModel, Vec3

from .anatomy_profile import HumanoidAnatomyProfile
from .constraints import PoseConstraintSet
from .support_solver import SupportSolver


class PosePriorEstimator:
    def __init__(self) -> None:
        self._support_solver = SupportSolver()

    def estimate(
        self,
        skeleton: SkeletonModel,
        anatomy: HumanoidAnatomyProfile,
        constraints: PoseConstraintSet,
    ) -> RigModel:
        support_state = self._support_solver.state_from_constraints(anatomy, constraints)
        positions = {bone.name: bone.rest_world_position for bone in skeleton.bones}

        pelvis = constraints.position_for("pelvis_main", anatomy.rest("pelvis"))
        chest = constraints.position_for("chest_secondary", anatomy.rest("spine_05"))
        head = constraints.position_for("head_main", anatomy.rest("head"))
        left_ankle = self._support_solver.target_for(
            support_state,
            "foot_l",
            constraints.position_for("ankle_l_main", anatomy.rest("foot_l")),
        )
        right_ankle = self._support_solver.target_for(
            support_state,
            "foot_r",
            constraints.position_for("ankle_r_main", anatomy.rest("foot_r")),
        )

        if not constraints.locked("pelvis_main"):
            pelvis = self._apply_leg_average_offset(anatomy, left_ankle, right_ankle, pelvis)
            pelvis = self._apply_wrist_posture_bias(anatomy, constraints, pelvis, weight=0.025)
            pelvis = self._support_solver.limit_pelvis_height(anatomy, pelvis)
        if not constraints.locked("chest_secondary"):
            chest = self._apply_wrist_posture_bias(anatomy, constraints, chest, weight=0.08)
            chest = add(chest, scale(subtract(pelvis, anatomy.rest("pelvis")), 0.25))
        if not constraints.locked("head_main"):
            head = add(head, scale(subtract(chest, anatomy.rest("spine_05")), 0.2))

        positions["pelvis"] = pelvis
        positions["spine_05"] = chest
        positions["head"] = head
        positions["foot_l"] = left_ankle
        positions["foot_r"] = right_ankle
        positions["ball_l"] = self._support_solver.clamp_to_floor(anatomy.offset("foot_l", "ball_l", left_ankle), anatomy.floor_y)
        positions["ball_r"] = self._support_solver.clamp_to_floor(anatomy.offset("foot_r", "ball_r", right_ankle), anatomy.floor_y)

        joints = [
            self._joint_model(anatomy.rest_by_name, name, parent_name, positions.get(name, anatomy.rest(name)))
            for name, parent_name in anatomy.primary_layout
            if name in anatomy.rest_by_name or name in positions
        ]
        return RigModel(name="AutoPosing PriorRig", joints=joints)

    def _apply_leg_average_offset(self, anatomy: HumanoidAnatomyProfile, left_ankle: Vec3, right_ankle: Vec3, pelvis: Vec3) -> Vec3:
        rest_feet = (anatomy.rest("foot_l"), anatomy.rest("foot_r"))
        target_feet = (left_ankle, right_ankle)
        delta = (0.0, 0.0, 0.0)
        for rest_point, current_point in zip(rest_feet, target_feet, strict=True):
            delta = add(delta, subtract(current_point, rest_point))
        delta = scale(delta, 0.5)
        return add(pelvis, scale(delta, 0.75))

    def _apply_wrist_posture_bias(
        self,
        anatomy: HumanoidAnatomyProfile,
        constraints: PoseConstraintSet,
        point: Vec3,
        *,
        weight: float,
    ) -> Vec3:
        biased = point
        for controller_id, rest_name in (("wrist_l_main", "hand_l"), ("wrist_r_main", "hand_r")):
            constraint = constraints.positions.get(controller_id)
            if constraint is None:
                continue
            wrist_delta = subtract(constraint.target_world_position, anatomy.rest(rest_name))
            biased = add(biased, scale(wrist_delta, weight * constraint.strength))
        return biased

    @staticmethod
    def _joint_model(rest_by_name: dict[str, BoneModel], name: str, parent_name: str | None, world_position: Vec3) -> JointModel:
        rest_bone = rest_by_name.get(name)
        if rest_bone is None:
            return JointModel(name=name, parent_name=parent_name, world_position=world_position, rest_world_position=world_position)
        return JointModel(
            name=name,
            parent_name=parent_name,
            local_position=rest_bone.local_position,
            local_rotation=rest_bone.local_rotation,
            world_position=world_position,
            world_rotation=rest_bone.world_rotation,
            rest_local_position=rest_bone.rest_local_position,
            rest_local_rotation=rest_bone.rest_local_rotation,
            rest_world_position=rest_bone.rest_world_position,
            rest_world_rotation=rest_bone.rest_world_rotation,
            bone_length=rest_bone.bone_length,
            primary_axis=rest_bone.primary_axis,
        )
