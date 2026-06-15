from __future__ import annotations

from core.autoposing.models import AutoPosingRigModel, SolvedEffectorModel, SolvedPoseModel
from core.math import add, scale, subtract
from core.rig.models import JointModel, RigModel
from core.skeleton.models import BoneModel, SkeletonModel, Vec3

from .anatomy_profile import HumanoidAnatomyProfile
from .body_compensation import apply_capped_compensation
from .constraints import PoseConstraintSet
from .limb_solver import LimbSolver, SolvedLimb
from .reach_constraints import ReachConstraintResult
from .spine_solver import SpineSolver
from .support_solver import SupportSolver


class WholeBodyRelaxationSolver:
    def __init__(self) -> None:
        self._spine_solver = SpineSolver()
        self._limb_solver = LimbSolver()
        self._support_solver = SupportSolver()

    def solve(
        self,
        rig: AutoPosingRigModel,
        skeleton: SkeletonModel,
        anatomy: HumanoidAnatomyProfile,
        constraints: PoseConstraintSet,
        initial_pose: RigModel,
    ) -> SolvedPoseModel:
        rest_by_name = anatomy.rest_by_name
        prior_positions = {joint.name: joint.world_position for joint in initial_pose.joints}
        support_state = self._support_solver.state_from_constraints(anatomy, constraints)

        pelvis = prior_positions.get("pelvis", anatomy.rest("pelvis"))
        chest_target = prior_positions.get("spine_05", anatomy.rest("spine_05"))
        head_target = prior_positions.get("head", anatomy.rest("head"))
        left_ankle_target = self._support_solver.target_for(
            support_state,
            "foot_l",
            constraints.position_for("ankle_l_main", prior_positions.get("foot_l", anatomy.rest("foot_l"))),
        )
        right_ankle_target = self._support_solver.target_for(
            support_state,
            "foot_r",
            constraints.position_for("ankle_r_main", prior_positions.get("foot_r", anatomy.rest("foot_r"))),
        )

        spine_positions, limbs = self._solve_body(
            anatomy,
            constraints,
            pelvis,
            chest_target,
            head_target,
            left_ankle_target,
            right_ankle_target,
        )

        arm_compensation = add(
            scale(limbs["left_arm"].reach.overreach_vector, 0.22),
            scale(limbs["right_arm"].reach.overreach_vector, 0.22),
        )
        leg_compensation = add(
            scale(limbs["left_leg"].reach.overreach_vector, 0.65),
            scale(limbs["right_leg"].reach.overreach_vector, 0.65),
        )
        pelvis, chest_target, head_target = apply_capped_compensation(
            pelvis,
            chest_target,
            head_target,
            arm_compensation,
            pelvis_weight=0.18,
            chest_weight=0.48,
            head_weight=0.14,
            pelvis_locked=constraints.locked("pelvis_main"),
            chest_locked=constraints.locked("chest_secondary"),
            head_locked=constraints.locked("head_main"),
        )
        pelvis, chest_target, head_target = apply_capped_compensation(
            pelvis,
            chest_target,
            head_target,
            leg_compensation,
            pelvis_weight=0.6,
            chest_weight=0.2,
            head_weight=0.08,
            pelvis_locked=constraints.locked("pelvis_main"),
            chest_locked=constraints.locked("chest_secondary"),
            head_locked=constraints.locked("head_main"),
        )
        pelvis = self._support_solver.limit_pelvis_height(anatomy, pelvis)

        spine_positions, limbs = self._solve_body(
            anatomy,
            constraints,
            pelvis,
            chest_target,
            head_target,
            left_ankle_target,
            right_ankle_target,
        )
        solved_positions = self._solved_positions(anatomy, spine_positions, limbs)
        limb_pressure = self._limb_pressure(limbs)
        rig_model = RigModel(
            name="AutoPosing SolverRig",
            joints=[
                self._joint_model(rest_by_name, name, parent_name, solved_positions.get(name, anatomy.rest(name)), limb_pressure.get(name))
                for name, parent_name in anatomy.primary_layout
                if name in rest_by_name or name in solved_positions
            ],
        )
        effectors = self._build_effectors(rig, anatomy, constraints, solved_positions, limbs)
        status_parts = []
        if any(effector.clamped for effector in effectors):
            status_parts.append("Target unreachable - clamped")
        support_error = self._support_solver.support_alignment_error(support_state, solved_positions)
        if support_error > 2.0:
            status_parts.append("Support adjusted")
        return SolvedPoseModel(rig=rig_model, effectors=effectors, status_message=" | ".join(status_parts))

    def _solve_body(
        self,
        anatomy: HumanoidAnatomyProfile,
        constraints: PoseConstraintSet,
        pelvis: Vec3,
        chest_target: Vec3,
        head_target: Vec3,
        left_ankle_target: Vec3,
        right_ankle_target: Vec3,
    ) -> tuple[dict[str, Vec3], dict[str, SolvedLimb]]:
        spine_positions = self._spine_solver.solve_spine_and_head(anatomy, pelvis, chest_target, head_target)
        clavicle_l = anatomy.offset("spine_05", "clavicle_l", spine_positions.get("spine_05", chest_target))
        clavicle_r = anatomy.offset("spine_05", "clavicle_r", spine_positions.get("spine_05", chest_target))
        upperarm_l_start = anatomy.offset("clavicle_l", "upperarm_l", clavicle_l)
        upperarm_r_start = anatomy.offset("clavicle_r", "upperarm_r", clavicle_r)
        thigh_l_start = anatomy.offset("pelvis", "thigh_l", pelvis)
        thigh_r_start = anatomy.offset("pelvis", "thigh_r", pelvis)

        left_arm_chain = anatomy.limb_chains["left_arm"]
        right_arm_chain = anatomy.limb_chains["right_arm"]
        left_leg_chain = anatomy.limb_chains["left_leg"]
        right_leg_chain = anatomy.limb_chains["right_leg"]
        limbs = {
            "left_arm": self._limb_solver.solve(
                anatomy,
                constraints,
                left_arm_chain,
                chain_start=upperarm_l_start,
                end_target=constraints.position_for(left_arm_chain.end_controller_id, anatomy.rest(left_arm_chain.end_name)),
                fallback_hint=anatomy.offset(left_arm_chain.start_name, left_arm_chain.mid_name, upperarm_l_start),
            ),
            "right_arm": self._limb_solver.solve(
                anatomy,
                constraints,
                right_arm_chain,
                chain_start=upperarm_r_start,
                end_target=constraints.position_for(right_arm_chain.end_controller_id, anatomy.rest(right_arm_chain.end_name)),
                fallback_hint=anatomy.offset(right_arm_chain.start_name, right_arm_chain.mid_name, upperarm_r_start),
            ),
            "left_leg": self._limb_solver.solve(
                anatomy,
                constraints,
                left_leg_chain,
                chain_start=thigh_l_start,
                end_target=left_ankle_target,
                fallback_hint=anatomy.offset(left_leg_chain.start_name, left_leg_chain.mid_name, thigh_l_start),
            ),
            "right_leg": self._limb_solver.solve(
                anatomy,
                constraints,
                right_leg_chain,
                chain_start=thigh_r_start,
                end_target=right_ankle_target,
                fallback_hint=anatomy.offset(right_leg_chain.start_name, right_leg_chain.mid_name, thigh_r_start),
            ),
        }
        return spine_positions, limbs

    def _solved_positions(
        self,
        anatomy: HumanoidAnatomyProfile,
        spine_positions: dict[str, Vec3],
        limbs: dict[str, SolvedLimb],
    ) -> dict[str, Vec3]:
        left_arm = limbs["left_arm"]
        right_arm = limbs["right_arm"]
        left_leg = limbs["left_leg"]
        right_leg = limbs["right_leg"]
        clavicle_l = anatomy.offset("spine_05", "clavicle_l", spine_positions.get("spine_05", anatomy.rest("spine_05")))
        clavicle_r = anatomy.offset("spine_05", "clavicle_r", spine_positions.get("spine_05", anatomy.rest("spine_05")))
        ball_l = self._support_solver.clamp_to_floor(anatomy.offset("foot_l", "ball_l", left_leg.solution.end_position), anatomy.floor_y)
        ball_r = self._support_solver.clamp_to_floor(anatomy.offset("foot_r", "ball_r", right_leg.solution.end_position), anatomy.floor_y)
        return {
            **spine_positions,
            "clavicle_l": clavicle_l,
            "upperarm_l": left_arm.solution.start_position,
            "lowerarm_l": left_arm.solution.mid_position,
            "hand_l": left_arm.solution.end_position,
            "clavicle_r": clavicle_r,
            "upperarm_r": right_arm.solution.start_position,
            "lowerarm_r": right_arm.solution.mid_position,
            "hand_r": right_arm.solution.end_position,
            "thigh_l": left_leg.solution.start_position,
            "calf_l": left_leg.solution.mid_position,
            "foot_l": self._support_solver.clamp_to_floor(left_leg.solution.end_position, anatomy.floor_y),
            "ball_l": ball_l,
            "thigh_r": right_leg.solution.start_position,
            "calf_r": right_leg.solution.mid_position,
            "foot_r": self._support_solver.clamp_to_floor(right_leg.solution.end_position, anatomy.floor_y),
            "ball_r": ball_r,
        }

    def _limb_pressure(self, limbs: dict[str, SolvedLimb]) -> dict[str, ReachConstraintResult]:
        left_arm = limbs["left_arm"]
        right_arm = limbs["right_arm"]
        left_leg = limbs["left_leg"]
        right_leg = limbs["right_leg"]
        return {
            "upperarm_l": left_arm.reach,
            "lowerarm_l": left_arm.reach,
            "hand_l": left_arm.reach,
            "upperarm_r": right_arm.reach,
            "lowerarm_r": right_arm.reach,
            "hand_r": right_arm.reach,
            "thigh_l": left_leg.reach,
            "calf_l": left_leg.reach,
            "foot_l": left_leg.reach,
            "ball_l": left_leg.reach,
            "thigh_r": right_leg.reach,
            "calf_r": right_leg.reach,
            "foot_r": right_leg.reach,
            "ball_r": right_leg.reach,
        }

    def _joint_model(
        self,
        rest_by_name: dict[str, BoneModel],
        name: str,
        parent_name: str | None,
        solved_world_position: Vec3,
        reach: ReachConstraintResult | None,
    ) -> JointModel:
        rest_bone = rest_by_name.get(name)
        if rest_bone is None:
            return JointModel(name=name, parent_name=parent_name, world_position=solved_world_position, rest_world_position=solved_world_position)
        return JointModel(
            name=name,
            parent_name=parent_name,
            local_position=rest_bone.local_position,
            local_rotation=rest_bone.local_rotation,
            world_position=solved_world_position,
            world_rotation=rest_bone.world_rotation,
            rest_local_position=rest_bone.rest_local_position,
            rest_local_rotation=rest_bone.rest_local_rotation,
            rest_world_position=rest_bone.rest_world_position,
            rest_world_rotation=rest_bone.rest_world_rotation,
            bone_length=rest_bone.bone_length,
            primary_axis=rest_bone.primary_axis,
            pressure=reach.pressure if reach is not None else 0.0,
            pressure_state=reach.pressure_state if reach is not None else "normal",
        )

    def _build_effectors(
        self,
        rig: AutoPosingRigModel,
        anatomy: HumanoidAnatomyProfile,
        constraints: PoseConstraintSet,
        solved_positions: dict[str, Vec3],
        limbs: dict[str, SolvedLimb],
    ) -> list[SolvedEffectorModel]:
        by_controller = {
            "wrist_l_main": limbs["left_arm"],
            "elbow_l_secondary": limbs["left_arm"],
            "wrist_r_main": limbs["right_arm"],
            "elbow_r_secondary": limbs["right_arm"],
            "ankle_l_main": limbs["left_leg"],
            "knee_l_secondary": limbs["left_leg"],
            "ankle_r_main": limbs["right_leg"],
            "knee_r_secondary": limbs["right_leg"],
        }
        effectors: list[SolvedEffectorModel] = []
        for controller in rig.controllers:
            target = controller.target
            if controller.identifier in by_controller:
                limb = by_controller[controller.identifier]
                solved_position = {
                    "wrist_l_main": limb.solution.end_position,
                    "elbow_l_secondary": limb.solution.mid_position,
                    "wrist_r_main": limb.solution.end_position,
                    "elbow_r_secondary": limb.solution.mid_position,
                    "ankle_l_main": self._support_solver.clamp_to_floor(limb.solution.end_position, anatomy.floor_y),
                    "knee_l_secondary": limb.solution.mid_position,
                    "ankle_r_main": self._support_solver.clamp_to_floor(limb.solution.end_position, anatomy.floor_y),
                    "knee_r_secondary": limb.solution.mid_position,
                }[controller.identifier]
                clamped_position = limb.reach.clamped_target if controller.controller_type == "main" else solved_position
                pressure = limb.reach.pressure
                pressure_state = limb.reach.pressure_state
                clamped = (
                    constraints.engaged(controller.identifier)
                    and limb.reach.clamped
                    and limb.reach.pressure > 0.01
                    and controller.controller_type == "main"
                )
            else:
                solved_position = solved_positions.get(controller.joint_name, target.world_position)
                clamped_position = solved_position
                pressure = 0.0
                pressure_state = "normal"
                clamped = False

            if controller.controller_type == "direction" and not constraints.engaged(controller.identifier):
                solved_position = add(
                    solved_positions.get(controller.joint_name, target.world_position),
                    anatomy.direction_offsets.get(controller.identifier, (0.0, 0.0, 20.0)),
                )
                clamped_position = solved_position

            effectors.append(
                SolvedEffectorModel(
                    controller_id=controller.identifier,
                    joint_name=controller.joint_name,
                    target_world_position=target.world_position,
                    solved_world_position=solved_position,
                    clamped_world_position=clamped_position,
                    pressure=pressure,
                    pressure_state=pressure_state,
                    clamped=clamped,
                )
            )
        return effectors
