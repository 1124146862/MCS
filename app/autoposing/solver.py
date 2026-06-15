from core.autoposing.models import (
    AutoPosingControllerModel,
    AutoPosingRigModel,
    SolvedEffectorModel,
    SolvedPoseModel,
)
from core.ik.two_bone import TwoBoneIkSolution, solve_two_bone_ik
from core.math import add, distance, lerp, normalize, scale, subtract
from core.rig.models import JointModel, RigModel
from core.skeleton.models import BoneModel, SkeletonModel, Vec3

from .body_compensation import apply_capped_compensation
from .reach_constraints import ReachConstraintResult, clamp_target_to_reach


class AutoPosingSolver:
    _PRIMARY_LAYOUT = (
        ("pelvis", None),
        ("spine_01", "pelvis"),
        ("spine_02", "spine_01"),
        ("spine_03", "spine_02"),
        ("spine_04", "spine_03"),
        ("spine_05", "spine_04"),
        ("neck_01", "spine_05"),
        ("neck_02", "neck_01"),
        ("head", "neck_02"),
        ("clavicle_l", "spine_05"),
        ("upperarm_l", "clavicle_l"),
        ("lowerarm_l", "upperarm_l"),
        ("hand_l", "lowerarm_l"),
        ("clavicle_r", "spine_05"),
        ("upperarm_r", "clavicle_r"),
        ("lowerarm_r", "upperarm_r"),
        ("hand_r", "lowerarm_r"),
        ("thigh_l", "pelvis"),
        ("calf_l", "thigh_l"),
        ("foot_l", "calf_l"),
        ("ball_l", "foot_l"),
        ("thigh_r", "pelvis"),
        ("calf_r", "thigh_r"),
        ("foot_r", "calf_r"),
        ("ball_r", "foot_r"),
    )

    _FORWARD_OFFSET = {
        "pelvis_dir": (0.0, 0.0, 24.0),
        "chest_dir": (0.0, 6.0, 26.0),
        "head_dir": (0.0, 0.0, 22.0),
    }

    def solve(self, rig: AutoPosingRigModel, skeleton: SkeletonModel) -> SolvedPoseModel:
        if not skeleton.bones:
            return SolvedPoseModel(status_message="No skeleton available.")

        rest_by_name = {bone.name: bone for bone in skeleton.bones}
        controller_by_id = {controller.identifier: controller for controller in rig.controllers}

        pelvis = self._controller_position(controller_by_id, "pelvis_main", self._rest(rest_by_name, "pelvis"))
        chest_target = self._controller_position(controller_by_id, "chest_secondary", self._rest(rest_by_name, "spine_05"))
        head_target = self._controller_position(controller_by_id, "head_main", self._rest(rest_by_name, "head"))

        left_ankle_target = self._controller_position(controller_by_id, "ankle_l_main", self._rest(rest_by_name, "foot_l"))
        right_ankle_target = self._controller_position(controller_by_id, "ankle_r_main", self._rest(rest_by_name, "foot_r"))
        pelvis = self._apply_leg_average_offset(rest_by_name, left_ankle_target, right_ankle_target, pelvis)

        chest_target = add(
            chest_target,
            scale(subtract(pelvis, self._rest(rest_by_name, "pelvis")), 0.25),
        )
        head_target = add(
            head_target,
            scale(subtract(chest_target, self._rest(rest_by_name, "spine_05")), 0.2),
        )

        spine_positions = self._solve_spine_and_head(rest_by_name, pelvis, chest_target, head_target)

        clavicle_l = self._offset(rest_by_name, "spine_05", "clavicle_l", spine_positions["spine_05"])
        clavicle_r = self._offset(rest_by_name, "spine_05", "clavicle_r", spine_positions["spine_05"])
        upperarm_l_start = self._offset(rest_by_name, "clavicle_l", "upperarm_l", clavicle_l)
        upperarm_r_start = self._offset(rest_by_name, "clavicle_r", "upperarm_r", clavicle_r)
        thigh_l_start = self._offset(rest_by_name, "pelvis", "thigh_l", pelvis)
        thigh_r_start = self._offset(rest_by_name, "pelvis", "thigh_r", pelvis)

        left_arm = self._solve_limb_chain(
            rig,
            rest_by_name,
            chain_start=upperarm_l_start,
            end_target=self._controller_position(controller_by_id, "wrist_l_main", self._rest(rest_by_name, "hand_l")),
            start_name="upperarm_l",
            mid_name="lowerarm_l",
            end_name="hand_l",
            hint_controller_id="elbow_l_secondary",
        )
        right_arm = self._solve_limb_chain(
            rig,
            rest_by_name,
            chain_start=upperarm_r_start,
            end_target=self._controller_position(controller_by_id, "wrist_r_main", self._rest(rest_by_name, "hand_r")),
            start_name="upperarm_r",
            mid_name="lowerarm_r",
            end_name="hand_r",
            hint_controller_id="elbow_r_secondary",
        )
        left_leg = self._solve_limb_chain(
            rig,
            rest_by_name,
            chain_start=thigh_l_start,
            end_target=left_ankle_target,
            start_name="thigh_l",
            mid_name="calf_l",
            end_name="foot_l",
            hint_controller_id="knee_l_secondary",
        )
        right_leg = self._solve_limb_chain(
            rig,
            rest_by_name,
            chain_start=thigh_r_start,
            end_target=right_ankle_target,
            start_name="thigh_r",
            mid_name="calf_r",
            end_name="foot_r",
            hint_controller_id="knee_r_secondary",
        )

        arm_compensation = add(scale(left_arm.reach.overreach_vector, 0.22), scale(right_arm.reach.overreach_vector, 0.22))
        leg_compensation = add(scale(left_leg.reach.overreach_vector, 0.65), scale(right_leg.reach.overreach_vector, 0.65))
        pelvis, chest_target, head_target = apply_capped_compensation(
            pelvis,
            chest_target,
            head_target,
            arm_compensation,
            pelvis_weight=0.18,
            chest_weight=0.48,
            head_weight=0.14,
            pelvis_locked=self._controller_locked(controller_by_id, "pelvis_main"),
            chest_locked=self._controller_locked(controller_by_id, "chest_secondary"),
            head_locked=self._controller_locked(controller_by_id, "head_main"),
        )
        pelvis, chest_target, head_target = apply_capped_compensation(
            pelvis,
            chest_target,
            head_target,
            leg_compensation,
            pelvis_weight=0.6,
            chest_weight=0.2,
            head_weight=0.08,
            pelvis_locked=self._controller_locked(controller_by_id, "pelvis_main"),
            chest_locked=self._controller_locked(controller_by_id, "chest_secondary"),
            head_locked=self._controller_locked(controller_by_id, "head_main"),
        )

        spine_positions = self._solve_spine_and_head(rest_by_name, pelvis, chest_target, head_target)
        clavicle_l = self._offset(rest_by_name, "spine_05", "clavicle_l", spine_positions["spine_05"])
        clavicle_r = self._offset(rest_by_name, "spine_05", "clavicle_r", spine_positions["spine_05"])
        upperarm_l_start = self._offset(rest_by_name, "clavicle_l", "upperarm_l", clavicle_l)
        upperarm_r_start = self._offset(rest_by_name, "clavicle_r", "upperarm_r", clavicle_r)
        thigh_l_start = self._offset(rest_by_name, "pelvis", "thigh_l", pelvis)
        thigh_r_start = self._offset(rest_by_name, "pelvis", "thigh_r", pelvis)

        left_arm = self._solve_limb_chain(
            rig,
            rest_by_name,
            chain_start=upperarm_l_start,
            end_target=self._controller_position(controller_by_id, "wrist_l_main", self._rest(rest_by_name, "hand_l")),
            start_name="upperarm_l",
            mid_name="lowerarm_l",
            end_name="hand_l",
            hint_controller_id="elbow_l_secondary",
        )
        right_arm = self._solve_limb_chain(
            rig,
            rest_by_name,
            chain_start=upperarm_r_start,
            end_target=self._controller_position(controller_by_id, "wrist_r_main", self._rest(rest_by_name, "hand_r")),
            start_name="upperarm_r",
            mid_name="lowerarm_r",
            end_name="hand_r",
            hint_controller_id="elbow_r_secondary",
        )
        left_leg = self._solve_limb_chain(
            rig,
            rest_by_name,
            chain_start=thigh_l_start,
            end_target=left_ankle_target,
            start_name="thigh_l",
            mid_name="calf_l",
            end_name="foot_l",
            hint_controller_id="knee_l_secondary",
        )
        right_leg = self._solve_limb_chain(
            rig,
            rest_by_name,
            chain_start=thigh_r_start,
            end_target=right_ankle_target,
            start_name="thigh_r",
            mid_name="calf_r",
            end_name="foot_r",
            hint_controller_id="knee_r_secondary",
        )

        ball_l = self._offset(rest_by_name, "foot_l", "ball_l", left_leg.solution.end_position)
        ball_r = self._offset(rest_by_name, "foot_r", "ball_r", right_leg.solution.end_position)
        solved_positions = {
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
            "foot_l": left_leg.solution.end_position,
            "ball_l": ball_l,
            "thigh_r": right_leg.solution.start_position,
            "calf_r": right_leg.solution.mid_position,
            "foot_r": right_leg.solution.end_position,
            "ball_r": ball_r,
        }

        limb_pressure = {
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
        rig_model = RigModel(
            name="AutoPosing SolverRig",
            joints=[
                self._joint_model(rest_by_name, name, parent_name, solved_positions.get(name, self._rest(rest_by_name, name)), limb_pressure.get(name))
                for name, parent_name in self._PRIMARY_LAYOUT
                if name in rest_by_name or name in solved_positions
            ],
        )
        effectors = self._build_effectors(rig, solved_positions, left_arm, right_arm, left_leg, right_leg)
        status_parts = []
        if any(effector.clamped for effector in effectors):
            status_parts.append("Target unreachable - clamped")
        return SolvedPoseModel(rig=rig_model, effectors=effectors, status_message=" | ".join(status_parts))

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
        solved_positions: dict[str, Vec3],
        left_arm,
        right_arm,
        left_leg,
        right_leg,
    ) -> list[SolvedEffectorModel]:
        by_joint = {
            "wrist_l_main": left_arm,
            "elbow_l_secondary": left_arm,
            "wrist_r_main": right_arm,
            "elbow_r_secondary": right_arm,
            "ankle_l_main": left_leg,
            "knee_l_secondary": left_leg,
            "ankle_r_main": right_leg,
            "knee_r_secondary": right_leg,
        }
        effectors: list[SolvedEffectorModel] = []
        for controller in rig.controllers:
            target = controller.target
            if controller.identifier in by_joint:
                limb = by_joint[controller.identifier]
                solved_position = {
                    "wrist_l_main": limb.solution.end_position,
                    "elbow_l_secondary": limb.solution.mid_position,
                    "wrist_r_main": limb.solution.end_position,
                    "elbow_r_secondary": limb.solution.mid_position,
                    "ankle_l_main": limb.solution.end_position,
                    "knee_l_secondary": limb.solution.mid_position,
                    "ankle_r_main": limb.solution.end_position,
                    "knee_r_secondary": limb.solution.mid_position,
                }[controller.identifier]
                clamped_position = limb.reach.clamped_target if controller.controller_type == "main" else solved_position
                pressure = limb.reach.pressure
                pressure_state = limb.reach.pressure_state
                clamped = limb.reach.clamped and controller.controller_type == "main"
            else:
                solved_position = solved_positions.get(controller.joint_name, target.world_position)
                clamped_position = solved_position
                pressure = 0.0
                pressure_state = "normal"
                clamped = False

            if controller.controller_type == "direction" and not (controller.active or controller.fixed or controller.always_active):
                solved_position = add(
                    solved_positions.get(controller.joint_name, target.world_position),
                    self._FORWARD_OFFSET.get(controller.identifier, (0.0, 0.0, 20.0)),
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

    def _solve_spine_and_head(
        self,
        rest_by_name: dict[str, BoneModel],
        pelvis: Vec3,
        chest_target: Vec3,
        head_target: Vec3,
    ) -> dict[str, Vec3]:
        lower_names = ("pelvis", "spine_01", "spine_02", "spine_03", "spine_04", "spine_05")
        upper_names = ("spine_05", "neck_01", "neck_02", "head")
        lower_lengths = [self._length(rest_by_name, *pair) for pair in zip(lower_names[:-1], lower_names[1:], strict=True)]
        upper_lengths = [self._length(rest_by_name, *pair) for pair in zip(upper_names[:-1], upper_names[1:], strict=True)]
        lower_rest_positions = [self._rest(rest_by_name, name) for name in lower_names]
        upper_rest_positions = [self._rest(rest_by_name, name) for name in upper_names]
        lower_chain = self._solve_linear_chain(lower_rest_positions, pelvis, chest_target, lower_lengths)
        chest = lower_chain[-1]
        upper_chain = self._solve_linear_chain(upper_rest_positions, chest, head_target, upper_lengths)
        return {
            "pelvis": lower_chain[0],
            "spine_01": lower_chain[1],
            "spine_02": lower_chain[2],
            "spine_03": lower_chain[3],
            "spine_04": lower_chain[4],
            "spine_05": lower_chain[5],
            "neck_01": upper_chain[1],
            "neck_02": upper_chain[2],
            "head": upper_chain[3],
        }

    def _solve_linear_chain(
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

    def _solve_limb_chain(
        self,
        rig: AutoPosingRigModel,
        rest_by_name: dict[str, BoneModel],
        *,
        chain_start: Vec3,
        end_target: Vec3,
        start_name: str,
        mid_name: str,
        end_name: str,
        hint_controller_id: str,
    ):
        controller_by_id = {controller.identifier: controller for controller in rig.controllers}
        upper_length = self._length(rest_by_name, start_name, mid_name)
        lower_length = self._length(rest_by_name, mid_name, end_name)
        max_reach = upper_length + lower_length - 0.001
        reach = clamp_target_to_reach(chain_start, end_target, max_reach)
        bend_hint = self._controller_position(
            controller_by_id,
            hint_controller_id,
            self._offset(rest_by_name, start_name, mid_name, chain_start),
        )
        solution = solve_two_bone_ik(chain_start, reach.clamped_target, upper_length, lower_length, bend_hint)
        return type("SolvedLimb", (), {"solution": solution, "reach": reach})

    def _apply_leg_average_offset(self, rest_by_name: dict[str, BoneModel], left_ankle_target: Vec3, right_ankle_target: Vec3, pelvis: Vec3) -> Vec3:
        rest_feet = (self._rest(rest_by_name, "foot_l"), self._rest(rest_by_name, "foot_r"))
        target_feet = (left_ankle_target, right_ankle_target)
        delta = (0.0, 0.0, 0.0)
        for rest_point, current_point in zip(rest_feet, target_feet, strict=True):
            delta = add(delta, subtract(current_point, rest_point))
        delta = scale(delta, 0.5)
        return add(pelvis, scale(delta, 0.75))

    @staticmethod
    def _rest(by_name: dict[str, BoneModel], bone_name: str) -> Vec3:
        bone = by_name.get(bone_name)
        return bone.rest_world_position if bone is not None else (0.0, 0.0, 0.0)

    def _offset(self, by_name: dict[str, BoneModel], reference_name: str, target_name: str, reference_world_position: Vec3) -> Vec3:
        reference = by_name.get(reference_name)
        target = by_name.get(target_name)
        if reference is None or target is None:
            return reference_world_position
        return add(reference_world_position, subtract(target.rest_world_position, reference.rest_world_position))

    def _length(self, by_name: dict[str, BoneModel], start_name: str, end_name: str) -> float:
        return max(distance(self._rest(by_name, start_name), self._rest(by_name, end_name)), 0.001)

    def _controller_position(
        self,
        by_id: dict[str, AutoPosingControllerModel],
        controller_id: str,
        fallback: Vec3,
    ) -> Vec3:
        controller = by_id.get(controller_id)
        if controller is None:
            return fallback
        if controller.active or controller.fixed or controller.always_active:
            return controller.target.world_position
        return fallback

    def _controller_locked(self, by_id: dict[str, AutoPosingControllerModel], controller_id: str) -> bool:
        controller = by_id.get(controller_id)
        return controller is not None and (controller.fixed or controller.selected)
