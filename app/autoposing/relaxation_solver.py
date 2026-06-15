from __future__ import annotations

from core.autoposing.models import AutoPosingRigModel, SolvedEffectorModel, SolvedPoseModel
from core.math import add, length, scale, subtract
from core.rig.models import JointModel, RigModel
from core.skeleton.models import BoneModel, SkeletonModel, Vec3

from .balance_solver import SupportBalanceSolver
from .anatomy_profile import HumanoidAnatomyProfile
from .body_compensation import apply_capped_compensation
from .constraints import PoseConstraintSet
from .finger_profile import FingerAnatomyProfile, build_finger_anatomy_profile, is_finger_controller_id
from .finger_solver import FingerPoseSolver, SolvedFingerChain
from .joint_rules.foot import FootRollRule
from .joint_rules.hip import HipOverreachSource, HipPelvisRule
from .limb_solver import LimbSolver, SolvedLimb
from .reach_constraints import ReachConstraintResult
from .shoulder_solver import ShoulderGirdleResult, ShoulderGirdleSolver
from .spine_solver import SpineSolver
from .support_solver import SupportSolver


class WholeBodyRelaxationSolver:
    def __init__(self) -> None:
        self._spine_solver = SpineSolver()
        self._limb_solver = LimbSolver()
        self._support_solver = SupportSolver()
        self._finger_solver = FingerPoseSolver()
        self._shoulder_solver = ShoulderGirdleSolver()
        self._balance_solver = SupportBalanceSolver()
        self._hip_rule = HipPelvisRule()
        self._foot_rule = FootRollRule()

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
        stable_support_count = len(self._support_solver.stable_support_targets(support_state))

        spine_positions, limbs = self._solve_body(
            anatomy,
            constraints,
            pelvis,
            chest_target,
            head_target,
            left_ankle_target,
            right_ankle_target,
        )

        shoulder_result = self._shoulder_solver.solve(anatomy, constraints, limbs, chest_target)
        chest_target = shoulder_result.chest_target

        arm_compensation = self._intent_weighted_overreach(
            (
                ("wrist_l_main", limbs["left_arm"], 0.22),
                ("wrist_r_main", limbs["right_arm"], 0.22),
            ),
            constraints,
            max_length=anatomy.body_height * 0.12,
        )
        leg_compensation = self._hip_rule.solve_leg_overreach(
            sources=(
                HipOverreachSource("ankle_l_main", limbs["left_leg"].reach.overreach_vector, 0.72),
                HipOverreachSource("ankle_r_main", limbs["right_leg"].reach.overreach_vector, 0.72),
            ),
            constraints=constraints,
            stable_support_count=stable_support_count,
            body_height=anatomy.body_height,
        )
        pelvis, chest_target, head_target = apply_capped_compensation(
            pelvis,
            chest_target,
            head_target,
            arm_compensation,
            pelvis_weight=0.10,
            chest_weight=0.42,
            head_weight=0.08,
            pelvis_locked=constraints.locked("pelvis_main"),
            chest_locked=constraints.locked("chest_secondary"),
            head_locked=constraints.locked("head_main"),
        )
        pelvis, chest_target, head_target = apply_capped_compensation(
            pelvis,
            chest_target,
            head_target,
            leg_compensation.compensation,
            pelvis_weight=leg_compensation.pelvis_weight,
            chest_weight=leg_compensation.chest_weight,
            head_weight=leg_compensation.head_weight,
            pelvis_locked=constraints.locked("pelvis_main"),
            chest_locked=constraints.locked("chest_secondary"),
            head_locked=constraints.locked("head_main"),
        )
        pelvis = self._support_solver.limit_pelvis_height(anatomy, pelvis)

        shoulder_result = self._shoulder_solver.solve(anatomy, constraints, limbs, chest_target)
        chest_target = shoulder_result.chest_target
        spine_positions, limbs = self._solve_body(
            anatomy,
            constraints,
            pelvis,
            chest_target,
            head_target,
            left_ankle_target,
            right_ankle_target,
            shoulder_result=shoulder_result,
        )
        solved_positions = self._solved_positions(anatomy, constraints, spine_positions, limbs, shoulder_result)
        balance = self._balance_solver.apply(anatomy, constraints, support_state, solved_positions, pelvis, chest_target, head_target)
        if balance.applied != (0.0, 0.0, 0.0):
            pelvis, chest_target, head_target = balance.pelvis, balance.chest, balance.head
            shoulder_result = self._shoulder_solver.solve(anatomy, constraints, limbs, chest_target)
            spine_positions, limbs = self._solve_body(
                anatomy,
                constraints,
                pelvis,
                chest_target,
                head_target,
                left_ankle_target,
                right_ankle_target,
                shoulder_result=shoulder_result,
            )
            solved_positions = self._solved_positions(anatomy, constraints, spine_positions, limbs, shoulder_result)
        limb_pressure = self._limb_pressure(limbs)
        finger_profile = build_finger_anatomy_profile(skeleton) if rig.fingers_enabled else FingerAnatomyProfile({}, {})
        finger_results = self._finger_solver.solve(finger_profile, constraints, solved_positions) if finger_profile.has_chains() else {}
        for result in finger_results.values():
            solved_positions.update(result.joint_positions)
        rig_model = RigModel(
            name="AutoPosing SolverRig",
            joints=self._rig_joints(rest_by_name, anatomy, solved_positions, limb_pressure, finger_profile),
        )
        effectors = self._build_effectors(rig, anatomy, constraints, solved_positions, limbs, finger_results)
        status_parts = []
        if any(effector.clamped for effector in effectors):
            status_parts.append("Target unreachable - clamped")
        support_error = self._support_solver.support_alignment_error(support_state, solved_positions)
        if support_error > 2.0:
            status_parts.append("Support adjusted")
        return SolvedPoseModel(rig=rig_model, effectors=effectors, status_message=" | ".join(status_parts))

    def _intent_weighted_overreach(
        self,
        sources: tuple[tuple[str, SolvedLimb, float], ...],
        constraints: PoseConstraintSet,
        *,
        max_length: float,
        vertical_weight: float = 1.0,
    ) -> Vec3:
        compensation = (0.0, 0.0, 0.0)
        for controller_id, limb, base_weight in sources:
            intent = constraints.pose_intent_weight(controller_id)
            if intent <= 0.0:
                continue
            overreach = limb.reach.overreach_vector
            if vertical_weight != 1.0:
                overreach = (overreach[0], overreach[1] * vertical_weight, overreach[2])
            compensation = add(compensation, scale(overreach, base_weight * intent))
        return self._cap_vector(compensation, max_length)

    @staticmethod
    def _cap_vector(vector: Vec3, max_length: float) -> Vec3:
        vector_length = length(vector)
        if vector_length <= max_length or vector_length <= 0.0:
            return vector
        return scale(vector, max_length / vector_length)

    def _rig_joints(
        self,
        rest_by_name: dict[str, BoneModel],
        anatomy: HumanoidAnatomyProfile,
        solved_positions: dict[str, Vec3],
        limb_pressure: dict[str, ReachConstraintResult],
        finger_profile: FingerAnatomyProfile,
    ) -> list[JointModel]:
        joints = [
            self._joint_model(rest_by_name, name, parent_name, solved_positions.get(name, anatomy.rest(name)), limb_pressure.get(name))
            for name, parent_name in anatomy.primary_layout
            if name in rest_by_name or name in solved_positions
        ]
        included = {joint.name for joint in joints}
        for chain in finger_profile.chains.values():
            for name in chain.joint_names:
                if name in included:
                    continue
                rest_bone = rest_by_name.get(name)
                parent_name = rest_bone.parent_name if rest_bone is not None else None
                joints.append(
                    self._joint_model(rest_by_name, name, parent_name, solved_positions.get(name, finger_profile.rest(name)), None)
                )
                included.add(name)
        return joints

    def _solve_body(
        self,
        anatomy: HumanoidAnatomyProfile,
        constraints: PoseConstraintSet,
        pelvis: Vec3,
        chest_target: Vec3,
        head_target: Vec3,
        left_ankle_target: Vec3,
        right_ankle_target: Vec3,
        shoulder_result: ShoulderGirdleResult | None = None,
    ) -> tuple[dict[str, Vec3], dict[str, SolvedLimb]]:
        spine_positions = self._spine_solver.solve_spine_and_head(anatomy, pelvis, chest_target, head_target)
        clavicle_l = self._clavicle_position(anatomy, shoulder_result, "l", spine_positions.get("spine_05", chest_target))
        clavicle_r = self._clavicle_position(anatomy, shoulder_result, "r", spine_positions.get("spine_05", chest_target))
        upperarm_l_start = anatomy.offset("clavicle_l", "upperarm_l", clavicle_l)
        upperarm_r_start = anatomy.offset("clavicle_r", "upperarm_r", clavicle_r)
        thigh_l_start = self._hip_rule.chain_start_for_side(
            anatomy=anatomy,
            constraints=constraints,
            pelvis_position=pelvis,
            side="l",
        )
        thigh_r_start = self._hip_rule.chain_start_for_side(
            anatomy=anatomy,
            constraints=constraints,
            pelvis_position=pelvis,
            side="r",
        )

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
        constraints: PoseConstraintSet,
        spine_positions: dict[str, Vec3],
        limbs: dict[str, SolvedLimb],
        shoulder_result: ShoulderGirdleResult | None = None,
    ) -> dict[str, Vec3]:
        left_arm = limbs["left_arm"]
        right_arm = limbs["right_arm"]
        left_leg = limbs["left_leg"]
        right_leg = limbs["right_leg"]
        clavicle_l = self._clavicle_position(anatomy, shoulder_result, "l", spine_positions.get("spine_05", anatomy.rest("spine_05")))
        clavicle_r = self._clavicle_position(anatomy, shoulder_result, "r", spine_positions.get("spine_05", anatomy.rest("spine_05")))
        foot_l = self._support_solver.clamp_to_floor(left_leg.solution.end_position, anatomy.floor_y)
        foot_r = self._support_solver.clamp_to_floor(right_leg.solution.end_position, anatomy.floor_y)
        ball_l = self._ball_roll_position(anatomy, constraints, "l", foot_l)
        ball_r = self._ball_roll_position(anatomy, constraints, "r", foot_r)
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
            "foot_l": foot_l,
            "ball_l": ball_l,
            "thigh_r": right_leg.solution.start_position,
            "calf_r": right_leg.solution.mid_position,
            "foot_r": foot_r,
            "ball_r": ball_r,
        }

    def _clavicle_position(
        self,
        anatomy: HumanoidAnatomyProfile,
        shoulder_result: ShoulderGirdleResult | None,
        side: str,
        chest_position: Vec3,
    ) -> Vec3:
        clavicle_name = f"clavicle_{side}"
        default_clavicle = anatomy.offset("spine_05", clavicle_name, chest_position)
        if shoulder_result is not None and clavicle_name in shoulder_result.clavicle_offsets:
            return add(default_clavicle, shoulder_result.clavicle_offsets[clavicle_name])
        return default_clavicle

    def _ball_roll_position(
        self,
        anatomy: HumanoidAnatomyProfile,
        constraints: PoseConstraintSet,
        side: str,
        foot_position: Vec3,
    ) -> Vec3:
        ball_name = f"ball_{side}"
        controller_id = f"ball_{side}_lesser"
        default_ball = self._support_solver.clamp_to_floor(anatomy.offset(f"foot_{side}", ball_name, foot_position), anatomy.floor_y)
        constraint = constraints.foot_contacts.get(controller_id) or constraints.positions.get(controller_id)
        if constraint is None:
            return default_ball
        max_reach = max(anatomy.length(f"foot_{side}", ball_name) * 1.15, 0.001)
        result = self._foot_rule.solve_ball_position(
            foot_position=foot_position,
            target_position=constraint.target_world_position,
            floor_y=anatomy.floor_y,
            max_reach=max_reach,
        )
        return result.solved_position

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
        finger_results: dict[str, SolvedFingerChain],
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
            finger_result = self._finger_result_for_controller(controller.identifier, finger_results)
            if finger_result is not None:
                if controller.identifier == finger_result.chain.tip_controller_id:
                    solved_position = finger_result.tip_position
                    clamped_position = finger_result.reach.clamped_target
                    pressure = finger_result.reach.pressure
                    pressure_state = finger_result.reach.pressure_state
                    clamped = constraints.engaged(controller.identifier) and finger_result.reach.clamped
                else:
                    solved_position = finger_result.pre_tip_position
                    clamped_position = solved_position
                    pressure = 0.0
                    pressure_state = "normal"
                    clamped = False
            elif controller.identifier in by_controller:
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
                if controller.identifier.startswith("elbow_") and limb.elbow_rule is not None:
                    pressure = max(pressure, limb.elbow_rule.pressure)
                    if "stretch" in {pressure_state, limb.elbow_rule.pressure_state}:
                        pressure_state = "stretch"
                if controller.identifier.startswith("knee_") and limb.knee_rule is not None:
                    pressure = max(pressure, limb.knee_rule.pressure)
                    if "stretch" in {pressure_state, limb.knee_rule.pressure_state}:
                        pressure_state = "stretch"
                clamped = (
                    constraints.engaged(controller.identifier)
                    and limb.reach.clamped
                    and limb.reach.pressure > 0.01
                    and controller.controller_type == "main"
                )
            else:
                solved_position = solved_positions.get(controller.joint_name, target.world_position)
                rest_bone = anatomy.rest_by_name.get(controller.joint_name)
                if controller.identifier in {"ball_l_lesser", "ball_r_lesser"} and constraints.engaged(controller.identifier):
                    side = controller.identifier.split("_")[1]
                    reach = self._ball_reach(anatomy, constraints, side, solved_positions, target.world_position)
                    solved_position = self._support_solver.clamp_to_floor(reach.clamped_target, anatomy.floor_y)
                    clamped_position = solved_position
                    pressure = reach.pressure
                    pressure_state = reach.pressure_state
                    clamped = reach.clamped
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
                    continue
                if controller.identifier in constraints.orientation_hints:
                    solved_position = target.world_position
                elif controller.controller_type != "direction" and rest_bone is not None:
                    solved_position = add(
                        solved_position,
                        subtract(target.default_position, rest_bone.rest_world_position),
                    )
                foot_contact = constraints.foot_contacts.get(controller.identifier)
                if foot_contact is not None:
                    solved_position = self._support_solver.clamp_to_floor(solved_position, foot_contact.floor_y)
                clamped_position = solved_position
                pressure = 0.0
                pressure_state = "normal"
                clamped = False

            if is_finger_controller_id(controller.identifier) and not rig.fingers_enabled:
                solved_position = target.world_position
                clamped_position = target.world_position
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

    def _ball_reach(
        self,
        anatomy: HumanoidAnatomyProfile,
        constraints: PoseConstraintSet,
        side: str,
        solved_positions: dict[str, Vec3],
        target_position: Vec3,
    ) -> ReachConstraintResult:
        foot_name = f"foot_{side}"
        ball_name = f"ball_{side}"
        foot_position = solved_positions.get(foot_name, anatomy.rest(foot_name))
        max_reach = max(anatomy.length(foot_name, ball_name) * 1.15, 0.001)
        return self._foot_rule.solve_ball_position(
            foot_position=foot_position,
            target_position=target_position,
            floor_y=anatomy.floor_y,
            max_reach=max_reach,
        ).reach

    @staticmethod
    def _finger_result_for_controller(
        controller_id: str,
        finger_results: dict[str, SolvedFingerChain],
    ) -> SolvedFingerChain | None:
        for result in finger_results.values():
            if controller_id in {result.chain.tip_controller_id, result.chain.pre_tip_controller_id}:
                return result
        return None
