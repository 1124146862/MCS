from core.autoposing.models import AutoPosingRigModel, RetargetJointPoseModel, RetargetPoseModel, SolvedPoseModel
from core.math import (
    add,
    angle_between_on_axis,
    distance,
    extract_twist_rotation,
    inverse_quaternion,
    length,
    normalize,
    normalize_quaternion,
    project_on_plane,
    quaternion_angle_axis,
    quaternion_multiply,
    quaternion_slerp,
    rotate_vector,
    rotation_between_vectors,
    scale,
    subtract,
)
from core.skeleton.models import BoneModel, SkeletonModel, Vec3

from .finger_profile import build_finger_anatomy_profile
from .joint_rules import AnkleRule, FootRollRule, ForearmRule, JointRuleResult, NeckHeadRule, SpineRule, WristRule


class RuntimeRetargeter:
    _MAIN_ORDER = (
        "pelvis",
        "spine_01",
        "spine_02",
        "spine_03",
        "spine_04",
        "spine_05",
        "neck_01",
        "neck_02",
        "head",
        "clavicle_l",
        "upperarm_l",
        "lowerarm_l",
        "hand_l",
        "clavicle_r",
        "upperarm_r",
        "lowerarm_r",
        "hand_r",
        "thigh_l",
        "calf_l",
        "foot_l",
        "ball_l",
        "thigh_r",
        "calf_r",
        "foot_r",
        "ball_r",
    )
    _AIM_CHILD = {
        "pelvis": "spine_01",
        "spine_01": "spine_02",
        "spine_02": "spine_03",
        "spine_03": "spine_04",
        "spine_04": "spine_05",
        "spine_05": "neck_01",
        "neck_01": "neck_02",
        "neck_02": "head",
        "clavicle_l": "upperarm_l",
        "upperarm_l": "lowerarm_l",
        "lowerarm_l": "hand_l",
        "clavicle_r": "upperarm_r",
        "upperarm_r": "lowerarm_r",
        "lowerarm_r": "hand_r",
        "thigh_l": "calf_l",
        "calf_l": "foot_l",
        "foot_l": "ball_l",
        "thigh_r": "calf_r",
        "calf_r": "foot_r",
        "foot_r": "ball_r",
    }
    _TWIST_GROUPS = {
        "upperarm_l": ("lowerarm_l", ("upperarm_twist_01_l", "upperarm_twist_02_l")),
        "lowerarm_l": ("hand_l", ("lowerarm_twist_01_l", "lowerarm_twist_02_l")),
        "upperarm_r": ("lowerarm_r", ("upperarm_twist_01_r", "upperarm_twist_02_r")),
        "lowerarm_r": ("hand_r", ("lowerarm_twist_01_r", "lowerarm_twist_02_r")),
        "thigh_l": ("calf_l", ("thigh_twist_01_l", "thigh_twist_02_l")),
        "calf_l": ("foot_l", ("calf_twist_01_l", "calf_twist_02_l")),
        "thigh_r": ("calf_r", ("thigh_twist_01_r", "thigh_twist_02_r")),
        "calf_r": ("foot_r", ("calf_twist_01_r", "calf_twist_02_r")),
    }

    def __init__(self) -> None:
        self._forearm_rule = ForearmRule()
        self._wrist_rule = WristRule()
        self._ankle_rule = AnkleRule()
        self._foot_rule = FootRollRule()
        self._spine_rule = SpineRule()
        self._neck_rule = NeckHeadRule()
        self.last_joint_rule_results: dict[str, JointRuleResult] = {}

    def retarget(self, rig: AutoPosingRigModel, solved_pose: SolvedPoseModel, skeleton: SkeletonModel) -> RetargetPoseModel:
        self.last_joint_rule_results = {}
        rest_by_name = {bone.name: bone for bone in skeleton.bones}
        solved_by_name = {joint.name: joint for joint in solved_pose.rig.joints}
        controllers_by_id = {controller.identifier: controller for controller in rig.controllers}
        effectors_by_id = {effector.controller_id: effector for effector in solved_pose.effectors}
        pose_map = {
            bone.name: RetargetJointPoseModel(
                name=bone.name,
                local_position=bone.rest_local_position,
                local_rotation=bone.rest_local_rotation,
            )
            for bone in skeleton.bones
        }
        desired_world_rotations = {
            bone.name: bone.rest_world_rotation
            for bone in skeleton.bones
        }

        for name in self._MAIN_ORDER:
            bone = rest_by_name.get(name)
            if bone is None:
                continue

            parent_rotation = self._parent_world_rotation(bone, desired_world_rotations, rest_by_name)
            desired_world_rotation = self._solve_world_rotation(
                bone,
                rest_by_name,
                solved_by_name,
                controllers_by_id,
                effectors_by_id,
            )
            desired_world_rotations[name] = desired_world_rotation
            local_rotation = quaternion_multiply(inverse_quaternion(parent_rotation), desired_world_rotation)
            trunk_rule = self._trunk_rule_result(bone, local_rotation, controllers_by_id)
            if trunk_rule is not None:
                local_rotation = normalize_quaternion(quaternion_multiply(trunk_rule.rotation, bone.rest_local_rotation))
                desired_world_rotation = normalize_quaternion(quaternion_multiply(parent_rotation, local_rotation))
                desired_world_rotations[name] = desired_world_rotation
                self._record_trunk_rule_result(name, trunk_rule)
            world_position = solved_by_name[name].world_position if name == "pelvis" and name in solved_by_name else None
            pose_map[name] = RetargetJointPoseModel(
                name=name,
                local_position=bone.rest_local_position,
                local_rotation=normalize_quaternion(local_rotation),
                world_position=world_position,
            )

        self._apply_wrist_forearm_rules(
            rest_by_name,
            solved_by_name,
            controllers_by_id,
            effectors_by_id,
            desired_world_rotations,
            pose_map,
        )
        self._apply_ankle_foot_rules(
            rest_by_name,
            solved_by_name,
            controllers_by_id,
            effectors_by_id,
            desired_world_rotations,
            pose_map,
        )

        for parent_name, (_, twist_names) in self._TWIST_GROUPS.items():
            parent_bone = rest_by_name.get(parent_name)
            if parent_bone is None:
                continue
            child_name = self._AIM_CHILD.get(parent_name)
            parent_pose = pose_map.get(parent_name)
            parent_binding_rotation = parent_pose.local_rotation if parent_pose is not None else parent_bone.rest_local_rotation
            local_delta = quaternion_multiply(parent_binding_rotation, inverse_quaternion(parent_bone.rest_local_rotation))
            twist_delta = extract_twist_rotation(local_delta, parent_bone.primary_axis)
            for twist_name in twist_names:
                twist_bone = rest_by_name.get(twist_name)
                if twist_bone is None:
                    continue
                ratio = self._twist_ratio(parent_bone, twist_bone, rest_by_name.get(child_name))
                blended_delta = quaternion_slerp((1.0, 0.0, 0.0, 0.0), twist_delta, ratio)
                local_rotation = normalize_quaternion(quaternion_multiply(blended_delta, twist_bone.rest_local_rotation))
                parent_rotation = desired_world_rotations.get(twist_bone.parent_name, twist_bone.rest_world_rotation)
                desired_world_rotation = normalize_quaternion(quaternion_multiply(parent_rotation, local_rotation))
                desired_world_rotations[twist_name] = desired_world_rotation
                pose_map[twist_name] = RetargetJointPoseModel(
                    name=twist_name,
                    local_position=twist_bone.rest_local_position,
                    local_rotation=local_rotation,
                )

        for ball_name, foot_name in (("ball_l", "foot_l"), ("ball_r", "foot_r")):
            ball_bone = rest_by_name.get(ball_name)
            foot_rotation = desired_world_rotations.get(foot_name)
            if ball_bone is None or foot_rotation is None:
                continue
            delta_rotation = quaternion_multiply(foot_rotation, inverse_quaternion(rest_by_name[foot_name].rest_world_rotation))
            desired_world_rotation = quaternion_multiply(delta_rotation, ball_bone.rest_world_rotation)
            ball_hint = self._ball_controller_roll_hint(ball_name, ball_bone, controllers_by_id)
            if ball_hint is None:
                ball_hint = self._engaged_effector_vector(
                    (f"{ball_name}_lesser", f"foot_{ball_name[-1]}_additional"),
                    ball_name,
                    solved_by_name,
                    controllers_by_id,
                    effectors_by_id,
                )
            if ball_hint is not None:
                current_hint = normalize(rotate_vector(desired_world_rotation, (0.0, 0.0, 1.0)))
                desired_hint = normalize(ball_hint)
                if current_hint != (0.0, 0.0, 0.0) and desired_hint != (0.0, 0.0, 0.0):
                    desired_world_rotation = normalize_quaternion(
                        quaternion_multiply(rotation_between_vectors(current_hint, desired_hint), desired_world_rotation)
                    )
                parent_rotation = self._parent_world_rotation(ball_bone, desired_world_rotations, rest_by_name)
                ball_delta = self._local_delta_from_world(ball_bone, parent_rotation, desired_world_rotation)
                toe_axis = self._foot_flexion_axis(ball_bone)
                ball_result = self._foot_rule.solve_toe_roll(ball_delta, toe_axis)
                desired_world_rotation = normalize_quaternion(
                    quaternion_multiply(
                        parent_rotation,
                        quaternion_multiply(ball_result.rotation, ball_bone.rest_local_rotation),
                    )
                )
                self.last_joint_rule_results[ball_name] = ball_result
            desired_world_rotations[ball_name] = desired_world_rotation
            parent_rotation = self._parent_world_rotation(ball_bone, desired_world_rotations, rest_by_name)
            pose_map[ball_name] = RetargetJointPoseModel(
                name=ball_name,
                local_position=ball_bone.rest_local_position,
                local_rotation=normalize_quaternion(
                    quaternion_multiply(inverse_quaternion(parent_rotation), desired_world_rotation)
                ),
            )

        self._apply_finger_rotations(
            rig,
            rest_by_name,
            solved_by_name,
            controllers_by_id,
            effectors_by_id,
            desired_world_rotations,
            pose_map,
            skeleton,
        )

        return RetargetPoseModel(joints=list(pose_map.values()), status_message=solved_pose.status_message)

    def _trunk_rule_result(
        self,
        bone: BoneModel,
        local_rotation,
        controllers_by_id: dict[str, object],
    ) -> JointRuleResult | None:
        if bone.name not in {"pelvis", "spine_01", "spine_02", "spine_03", "spine_04", "spine_05", "neck_01", "neck_02", "head"}:
            return None
        local_delta = quaternion_multiply(local_rotation, inverse_quaternion(bone.rest_local_rotation))
        flexion_axis = self._local_flexion_axis(bone)
        lateral_axis = self._local_lateral_axis(bone, flexion_axis)
        if bone.name in {"pelvis", "spine_01", "spine_02", "spine_03", "spine_04", "spine_05"}:
            return self._spine_rule.solve(
                local_delta,
                bone.primary_axis,
                flexion_axis=flexion_axis,
                lateral_axis=lateral_axis,
                label=bone.name,
            )
        if bone.name in {"neck_01", "neck_02"}:
            return self._neck_rule.solve_neck(
                local_delta,
                bone.primary_axis,
                flexion_axis=flexion_axis,
                lateral_axis=lateral_axis,
            )
        directed = self._head_direction_engaged(controllers_by_id)
        if not directed:
            local_delta = (1.0, 0.0, 0.0, 0.0)
        return self._neck_rule.solve_head(
            local_delta,
            bone.primary_axis,
            flexion_axis=flexion_axis,
            lateral_axis=lateral_axis,
            directed=directed,
        )

    def _record_trunk_rule_result(self, bone_name: str, result: JointRuleResult) -> None:
        if bone_name == "pelvis":
            self._store_rule_result("pelvis", result)
        elif bone_name.startswith("spine_"):
            self._store_rule_result("spine", result)
        elif bone_name.startswith("neck_"):
            self._store_rule_result("neck", result)
        elif bone_name == "head":
            self._store_rule_result("head", result)

    def _store_rule_result(self, key: str, result: JointRuleResult) -> None:
        previous = self.last_joint_rule_results.get(key)
        if previous is None or result.pressure >= previous.pressure:
            self.last_joint_rule_results[key] = result

    @staticmethod
    def _head_direction_engaged(controllers_by_id: dict[str, object]) -> bool:
        return RuntimeRetargeter._controller_engaged(controllers_by_id.get("head_dir")) or RuntimeRetargeter._controller_engaged(
            controllers_by_id.get("head_additional")
        )

    def _solve_world_rotation(
        self,
        bone: BoneModel,
        rest_by_name: dict[str, BoneModel],
        solved_by_name: dict[str, object],
        controllers_by_id: dict[str, object],
        effectors_by_id: dict[str, object],
    ):
        terminal_rotation = self._terminal_world_rotation(
            bone,
            rest_by_name,
            solved_by_name,
            controllers_by_id,
            effectors_by_id,
        )
        if terminal_rotation is not None:
            return terminal_rotation

        child_name = self._AIM_CHILD.get(bone.name)
        if child_name is None or child_name not in rest_by_name or child_name not in solved_by_name or bone.name not in solved_by_name:
            return bone.rest_world_rotation

        rest_aim = subtract(rest_by_name[child_name].rest_world_position, bone.rest_world_position)
        desired_aim = subtract(solved_by_name[child_name].world_position, solved_by_name[bone.name].world_position)
        if distance((0.0, 0.0, 0.0), rest_aim) <= 1e-6 or distance((0.0, 0.0, 0.0), desired_aim) <= 1e-6:
            return bone.rest_world_rotation

        rest_secondary = self._rest_secondary_vector(bone, rest_by_name)
        desired_secondary = self._desired_secondary_vector(
            bone.name,
            solved_by_name,
            controllers_by_id,
            effectors_by_id,
            rest_by_name,
        )

        swing = rotation_between_vectors(rest_aim, desired_aim)
        rotated_secondary = rotate_vector(swing, rest_secondary)
        desired_aim_axis = normalize(desired_aim)
        projected_rotated = normalize(project_on_plane(rotated_secondary, desired_aim_axis))
        projected_desired = normalize(project_on_plane(desired_secondary, desired_aim_axis))
        if projected_rotated == (0.0, 0.0, 0.0) or projected_desired == (0.0, 0.0, 0.0):
            delta_rotation = swing
        else:
            twist_angle = angle_between_on_axis(projected_rotated, projected_desired, desired_aim_axis)
            twist = quaternion_angle_axis(twist_angle, desired_aim_axis)
            delta_rotation = quaternion_multiply(twist, swing)
        return normalize_quaternion(quaternion_multiply(delta_rotation, bone.rest_world_rotation))

    def _terminal_world_rotation(
        self,
        bone: BoneModel,
        rest_by_name: dict[str, BoneModel],
        solved_by_name: dict[str, object],
        controllers_by_id: dict[str, object],
        effectors_by_id: dict[str, object],
    ):
        if bone.name not in solved_by_name:
            return None
        if bone.name == "hand_l":
            return None
        if bone.name == "hand_r":
            return None
        if bone.name == "head":
            direction_rotation = self._terminal_rotation_from_direction_controller(
                bone,
                solved_by_name,
                controllers_by_id,
                "head_dir",
            )
            if direction_rotation is not None:
                return direction_rotation
            return self._terminal_rotation_from_controller(
                bone,
                solved_by_name,
                controllers_by_id,
                effectors_by_id,
                ("head_additional",),
            )
        return None

    def _terminal_rotation_from_direction_controller(
        self,
        bone: BoneModel,
        solved_by_name: dict[str, object],
        controllers_by_id: dict[str, object],
        controller_id: str,
    ):
        controller = controllers_by_id.get(controller_id)
        origin = solved_by_name.get(bone.name)
        if not self._controller_engaged(controller) or origin is None:
            return None
        desired_secondary = normalize(subtract(controller.target.world_position, origin.world_position))
        rest_secondary = normalize(rotate_vector(bone.rest_world_rotation, (0.0, 0.0, 1.0)))
        if rest_secondary == (0.0, 0.0, 0.0) or desired_secondary == (0.0, 0.0, 0.0):
            return None
        delta_rotation = rotation_between_vectors(rest_secondary, desired_secondary)
        return normalize_quaternion(quaternion_multiply(delta_rotation, bone.rest_world_rotation))

    def _apply_wrist_forearm_rules(
        self,
        rest_by_name: dict[str, BoneModel],
        solved_by_name: dict[str, object],
        controllers_by_id: dict[str, object],
        effectors_by_id: dict[str, object],
        desired_world_rotations: dict[str, tuple[float, float, float, float]],
        pose_map: dict[str, RetargetJointPoseModel],
    ) -> None:
        for side in ("l", "r"):
            hand_name = f"hand_{side}"
            lowerarm_name = f"lowerarm_{side}"
            hand_bone = rest_by_name.get(hand_name)
            lowerarm_bone = rest_by_name.get(lowerarm_name)
            lowerarm_pose = pose_map.get(lowerarm_name)
            if hand_bone is None or lowerarm_bone is None or lowerarm_pose is None:
                continue

            desired_hand_world = self._terminal_rotation_from_controller(
                hand_bone,
                solved_by_name,
                controllers_by_id,
                effectors_by_id,
                (f"hand_{side}_additional", f"hand_{side}_secondary"),
            )
            desired_hand_world = self._blend_finger_palm_spread(
                hand_bone,
                side,
                desired_hand_world,
                controllers_by_id,
                effectors_by_id,
            )
            if desired_hand_world is None:
                continue

            parent_world = self._parent_world_rotation(lowerarm_bone, desired_world_rotations, rest_by_name)
            lowerarm_world = desired_world_rotations.get(lowerarm_name, lowerarm_bone.rest_world_rotation)
            desired_hand_local = quaternion_multiply(inverse_quaternion(lowerarm_world), desired_hand_world)
            hand_local_delta = quaternion_multiply(desired_hand_local, inverse_quaternion(hand_bone.rest_local_rotation))

            forearm_result = self._forearm_rule.solve(hand_local_delta, lowerarm_bone.primary_axis)
            wrist_result = self._wrist_rule.solve(hand_local_delta, lowerarm_bone.primary_axis, forearm_result.overflow_twist)

            lowerarm_local = normalize_quaternion(
                quaternion_multiply(forearm_result.rotation, lowerarm_pose.local_rotation)
            )
            lowerarm_world = normalize_quaternion(quaternion_multiply(parent_world, lowerarm_local))
            desired_world_rotations[lowerarm_name] = lowerarm_world
            pose_map[lowerarm_name] = RetargetJointPoseModel(
                name=lowerarm_name,
                local_position=lowerarm_bone.rest_local_position,
                local_rotation=lowerarm_local,
            )

            hand_local = normalize_quaternion(quaternion_multiply(wrist_result.rotation, hand_bone.rest_local_rotation))
            hand_world = normalize_quaternion(quaternion_multiply(lowerarm_world, hand_local))
            desired_world_rotations[hand_name] = hand_world
            pose_map[hand_name] = RetargetJointPoseModel(
                name=hand_name,
                local_position=hand_bone.rest_local_position,
                local_rotation=hand_local,
            )
            self.last_joint_rule_results[f"forearm_{side}"] = forearm_result
            self.last_joint_rule_results[f"wrist_{side}"] = wrist_result

    def _blend_finger_palm_spread(
        self,
        bone: BoneModel,
        side: str,
        base_rotation,
        controllers_by_id: dict[str, object],
        effectors_by_id: dict[str, object],
    ):
        rest_vector = (0.0, 0.0, 0.0)
        desired_vector = (0.0, 0.0, 0.0)
        weight_total = 0.0
        for finger_name in ("index", "pinky"):
            controller = controllers_by_id.get(f"{finger_name}_{side}_tip")
            effector = effectors_by_id.get(f"{finger_name}_{side}_tip")
            if not self._controller_engaged(controller) or effector is None:
                continue
            if distance(controller.target.world_position, controller.target.default_position) <= 0.001:
                continue
            rest_vector = add(rest_vector, subtract(controller.target.default_position, bone.rest_world_position))
            desired_vector = add(desired_vector, subtract(effector.solved_world_position, bone.rest_world_position))
            weight_total += 1.0
        if weight_total <= 0.0:
            return base_rotation

        rest_vector = normalize(scale(rest_vector, 1.0 / weight_total))
        desired_vector = normalize(scale(desired_vector, 1.0 / weight_total))
        if rest_vector == (0.0, 0.0, 0.0) or desired_vector == (0.0, 0.0, 0.0):
            return base_rotation
        spread_delta = rotation_between_vectors(rest_vector, desired_vector)
        base = base_rotation if base_rotation is not None else bone.rest_world_rotation
        weight = 0.05 if base_rotation is not None else 0.2
        weighted_delta = quaternion_slerp((1.0, 0.0, 0.0, 0.0), spread_delta, weight)
        return normalize_quaternion(quaternion_multiply(weighted_delta, base))

    def _apply_ankle_foot_rules(
        self,
        rest_by_name: dict[str, BoneModel],
        solved_by_name: dict[str, object],
        controllers_by_id: dict[str, object],
        effectors_by_id: dict[str, object],
        desired_world_rotations: dict[str, tuple[float, float, float, float]],
        pose_map: dict[str, RetargetJointPoseModel],
    ) -> None:
        for side in ("l", "r"):
            foot_name = f"foot_{side}"
            foot_bone = rest_by_name.get(foot_name)
            foot_pose = pose_map.get(foot_name)
            if foot_bone is None or foot_pose is None or foot_name not in solved_by_name:
                continue

            flex_controller = controllers_by_id.get(f"foot_{side}_secondary")
            roll_controller = controllers_by_id.get(f"foot_{side}_additional")
            ball_controller = controllers_by_id.get(f"ball_{side}_lesser")
            flex_engaged = self._controller_engaged(flex_controller)
            roll_engaged = self._controller_engaged(roll_controller) or self._controller_engaged(ball_controller)
            if not flex_engaged and not roll_engaged:
                continue

            parent_world = self._parent_world_rotation(foot_bone, desired_world_rotations, rest_by_name)
            base_world = desired_world_rotations.get(foot_name, foot_bone.rest_world_rotation)
            desired_flex_world = base_world
            if flex_engaged:
                desired_flex_world = (
                    self._terminal_rotation_from_controller(
                        foot_bone,
                        solved_by_name,
                        controllers_by_id,
                        effectors_by_id,
                        (f"foot_{side}_secondary",),
                    )
                    or base_world
                )
            flex_delta = self._local_delta_from_world(foot_bone, parent_world, desired_flex_world)
            flexion_axis = self._foot_flexion_axis(foot_bone)
            ankle_result = self._ankle_rule.solve(flex_delta, foot_bone.primary_axis, flexion_axis)

            roll_rotation = (1.0, 0.0, 0.0, 0.0)
            if roll_engaged:
                desired_roll_world = (
                    self._terminal_rotation_from_controller(
                        foot_bone,
                        solved_by_name,
                        controllers_by_id,
                        effectors_by_id,
                        (f"foot_{side}_additional", f"ball_{side}_lesser"),
                    )
                    or base_world
                )
                roll_delta = self._local_delta_from_world(foot_bone, parent_world, desired_roll_world)
                foot_result = self._foot_rule.solve_hindfoot_roll(roll_delta, foot_bone.primary_axis)
                roll_rotation = foot_result.rotation
                self.last_joint_rule_results[f"foot_{side}"] = foot_result

            local_delta = normalize_quaternion(quaternion_multiply(roll_rotation, ankle_result.rotation))
            local_rotation = normalize_quaternion(quaternion_multiply(local_delta, foot_bone.rest_local_rotation))
            desired_world = normalize_quaternion(quaternion_multiply(parent_world, local_rotation))
            desired_world_rotations[foot_name] = desired_world
            pose_map[foot_name] = RetargetJointPoseModel(
                name=foot_name,
                local_position=foot_bone.rest_local_position,
                local_rotation=local_rotation,
            )
            self.last_joint_rule_results[f"ankle_{side}"] = ankle_result

    def _ball_controller_roll_hint(
        self,
        ball_name: str,
        ball_bone: BoneModel,
        controllers_by_id: dict[str, object],
    ) -> Vec3 | None:
        controller = controllers_by_id.get(f"{ball_name}_lesser")
        if not self._controller_engaged(controller):
            return None
        rest_hint = subtract(controller.target.default_position, ball_bone.rest_world_position)
        desired_hint = subtract(controller.target.world_position, ball_bone.rest_world_position)
        if distance((0.0, 0.0, 0.0), rest_hint) <= 1e-6 or distance((0.0, 0.0, 0.0), desired_hint) <= 1e-6:
            return None
        return desired_hint

    def _apply_finger_rotations(
        self,
        rig: AutoPosingRigModel,
        rest_by_name: dict[str, BoneModel],
        solved_by_name: dict[str, object],
        controllers_by_id: dict[str, object],
        effectors_by_id: dict[str, object],
        desired_world_rotations: dict[str, tuple[float, float, float, float]],
        pose_map: dict[str, RetargetJointPoseModel],
        skeleton: SkeletonModel,
    ) -> None:
        if not rig.fingers_enabled:
            return
        profile = build_finger_anatomy_profile(skeleton)
        for chain in profile.chains.values():
            if not self._finger_chain_engaged(chain, controllers_by_id):
                continue
            self._propagate_finger_parent_rotations(chain.root_name, chain.hand_name, rest_by_name, desired_world_rotations, pose_map)
            for index, joint_name in enumerate(chain.joint_names):
                bone = rest_by_name.get(joint_name)
                solved_joint = solved_by_name.get(joint_name)
                if bone is None or solved_joint is None:
                    continue
                if index + 1 < len(chain.joint_names):
                    child_name = chain.joint_names[index + 1]
                    child = solved_by_name.get(child_name)
                    rest_child_position = rest_by_name[child_name].rest_world_position if child_name in rest_by_name else None
                    desired_child_position = child.world_position if child is not None else None
                else:
                    effector = effectors_by_id.get(chain.tip_controller_id)
                    rest_child_position = chain.tip_world_position
                    desired_child_position = effector.solved_world_position if effector is not None else None
                if rest_child_position is None or desired_child_position is None:
                    continue
                desired_world_rotation = self._aim_world_rotation(
                    bone,
                    solved_joint.world_position,
                    rest_child_position,
                    desired_child_position,
                )
                parent_rotation = self._parent_world_rotation(bone, desired_world_rotations, rest_by_name)
                desired_world_rotations[joint_name] = desired_world_rotation
                pose_map[joint_name] = RetargetJointPoseModel(
                    name=joint_name,
                    local_position=bone.rest_local_position,
                    local_rotation=normalize_quaternion(
                        quaternion_multiply(inverse_quaternion(parent_rotation), desired_world_rotation)
                    ),
                )

    def _aim_world_rotation(
        self,
        bone: BoneModel,
        solved_position: Vec3,
        rest_child_position: Vec3,
        desired_child_position: Vec3,
    ):
        rest_aim = subtract(rest_child_position, bone.rest_world_position)
        desired_aim = subtract(desired_child_position, solved_position)
        if distance((0.0, 0.0, 0.0), rest_aim) <= 1e-6 or distance((0.0, 0.0, 0.0), desired_aim) <= 1e-6:
            return bone.rest_world_rotation
        delta_rotation = rotation_between_vectors(rest_aim, desired_aim)
        return normalize_quaternion(quaternion_multiply(delta_rotation, bone.rest_world_rotation))

    def _propagate_finger_parent_rotations(
        self,
        root_name: str,
        hand_name: str,
        rest_by_name: dict[str, BoneModel],
        desired_world_rotations: dict[str, tuple[float, float, float, float]],
        pose_map: dict[str, RetargetJointPoseModel],
    ) -> None:
        parent_names: list[str] = []
        parent_name = rest_by_name.get(root_name).parent_name if root_name in rest_by_name else None
        while parent_name and parent_name != hand_name and parent_name in rest_by_name:
            parent_names.append(parent_name)
            parent_name = rest_by_name[parent_name].parent_name
        for name in reversed(parent_names):
            bone = rest_by_name[name]
            parent_rotation = self._parent_world_rotation(bone, desired_world_rotations, rest_by_name)
            desired_world_rotation = normalize_quaternion(quaternion_multiply(parent_rotation, bone.rest_local_rotation))
            desired_world_rotations[name] = desired_world_rotation
            pose_map[name] = RetargetJointPoseModel(
                name=name,
                local_position=bone.rest_local_position,
                local_rotation=bone.rest_local_rotation,
            )

    @staticmethod
    def _finger_chain_engaged(chain, controllers_by_id: dict[str, object]) -> bool:
        tip = controllers_by_id.get(chain.tip_controller_id)
        pre_tip = controllers_by_id.get(chain.pre_tip_controller_id)
        return RuntimeRetargeter._finger_controller_driven(tip) or RuntimeRetargeter._finger_controller_driven(pre_tip)

    @staticmethod
    def _finger_controller_driven(controller) -> bool:
        if controller is None:
            return False
        if controller.fixed:
            return True
        return controller.active and distance(controller.target.world_position, controller.target.default_position) > 0.001

    def _terminal_rotation_from_controller(
        self,
        bone: BoneModel,
        solved_by_name: dict[str, object],
        controllers_by_id: dict[str, object],
        effectors_by_id: dict[str, object],
        controller_ids: tuple[str, ...],
    ):
        desired_secondary = self._engaged_effector_vector(
            controller_ids,
            bone.name,
            solved_by_name,
            controllers_by_id,
            effectors_by_id,
        )
        if desired_secondary is None:
            return None

        rest_secondary = normalize(rotate_vector(bone.rest_world_rotation, (0.0, 0.0, 1.0)))
        desired_secondary = normalize(desired_secondary)
        if rest_secondary == (0.0, 0.0, 0.0) or desired_secondary == (0.0, 0.0, 0.0):
            return None
        delta_rotation = rotation_between_vectors(rest_secondary, desired_secondary)
        return normalize_quaternion(quaternion_multiply(delta_rotation, bone.rest_world_rotation))

    def _rest_secondary_vector(self, bone: BoneModel, rest_by_name: dict[str, BoneModel]) -> Vec3:
        if bone.name == "pelvis":
            return self._pair_vector(rest_by_name, "thigh_l", "thigh_r", bone.rest_world_rotation)
        if bone.name in {"spine_01", "spine_02", "spine_03", "spine_04", "spine_05"}:
            return self._pair_vector(rest_by_name, "clavicle_l", "clavicle_r", bone.rest_world_rotation)
        if bone.name.startswith("upperarm") or bone.name.startswith("lowerarm"):
            return normalize(rotate_vector(bone.rest_world_rotation, (0.0, 0.0, 1.0)))
        if bone.name.startswith("thigh") or bone.name.startswith("calf"):
            return normalize(rotate_vector(bone.rest_world_rotation, (0.0, 0.0, 1.0)))
        return normalize(rotate_vector(bone.rest_world_rotation, (0.0, 0.0, 1.0)))

    def _desired_secondary_vector(
        self,
        bone_name: str,
        solved_by_name: dict[str, object],
        controllers_by_id: dict[str, object],
        effectors_by_id: dict[str, object],
        rest_by_name: dict[str, BoneModel],
    ) -> Vec3:
        if bone_name == "pelvis":
            direction_controller = controllers_by_id.get("pelvis_dir")
            if self._controller_engaged(direction_controller):
                return subtract(direction_controller.target.world_position, solved_by_name[bone_name].world_position)
            additional = self._engaged_effector_vector(
                ("pelvis_additional",),
                bone_name,
                solved_by_name,
                controllers_by_id,
                effectors_by_id,
            )
            if additional is not None:
                return additional
            return self._pair_vector_from_solved(solved_by_name, "thigh_l", "thigh_r", rest_by_name[bone_name].rest_world_rotation)
        if bone_name in {"spine_01", "spine_02", "spine_03", "spine_04", "spine_05"}:
            direction_controller = controllers_by_id.get("chest_dir")
            if self._controller_engaged(direction_controller) and bone_name == "spine_05":
                return subtract(direction_controller.target.world_position, solved_by_name[bone_name].world_position)
            if bone_name == "spine_05":
                additional = self._engaged_effector_vector(
                    ("chest_additional",),
                    bone_name,
                    solved_by_name,
                    controllers_by_id,
                    effectors_by_id,
                )
                if additional is not None:
                    return additional
            return self._pair_vector_from_solved(solved_by_name, "clavicle_l", "clavicle_r", rest_by_name[bone_name].rest_world_rotation)
        if bone_name in {"neck_01", "neck_02", "head"}:
            direction_controller = controllers_by_id.get("head_dir")
            if self._controller_engaged(direction_controller):
                return subtract(direction_controller.target.world_position, solved_by_name[bone_name].world_position)
            if bone_name == "head":
                additional = self._engaged_effector_vector(
                    ("head_additional",),
                    bone_name,
                    solved_by_name,
                    controllers_by_id,
                    effectors_by_id,
                )
                if additional is not None:
                    return additional
        if bone_name.startswith("upperarm"):
            hint = effectors_by_id.get("elbow_l_secondary" if bone_name.endswith("_l") else "elbow_r_secondary")
            if hint is not None:
                return subtract(hint.solved_world_position, solved_by_name[bone_name].world_position)
        if bone_name.startswith("lowerarm"):
            hint = effectors_by_id.get("elbow_l_secondary" if bone_name.endswith("_l") else "elbow_r_secondary")
            if hint is not None:
                return subtract(hint.solved_world_position, solved_by_name[bone_name].world_position)
        if bone_name.startswith("thigh") or bone_name.startswith("calf"):
            hint = effectors_by_id.get("knee_l_secondary" if bone_name.endswith("_l") else "knee_r_secondary")
            if hint is not None:
                return subtract(hint.solved_world_position, solved_by_name[bone_name].world_position)
        return normalize(rotate_vector(rest_by_name[bone_name].rest_world_rotation, (0.0, 0.0, 1.0)))

    def _engaged_effector_vector(
        self,
        controller_ids: tuple[str, ...],
        origin_name: str,
        solved_by_name: dict[str, object],
        controllers_by_id: dict[str, object],
        effectors_by_id: dict[str, object],
    ) -> Vec3 | None:
        origin = solved_by_name.get(origin_name)
        if origin is None:
            return None
        for controller_id in controller_ids:
            controller = controllers_by_id.get(controller_id)
            effector = effectors_by_id.get(controller_id)
            if not self._controller_engaged(controller) or effector is None:
                continue
            vector = subtract(effector.solved_world_position, origin.world_position)
            if distance((0.0, 0.0, 0.0), vector) > 1e-6:
                return vector
        return None

    def _pair_vector(self, rest_by_name: dict[str, BoneModel], left_name: str, right_name: str, fallback_rotation) -> Vec3:
        left = rest_by_name.get(left_name)
        right = rest_by_name.get(right_name)
        if left is None or right is None:
            return normalize(rotate_vector(fallback_rotation, (0.0, 0.0, 1.0)))
        vector = subtract(right.rest_world_position, left.rest_world_position)
        return normalize(vector) if vector != (0.0, 0.0, 0.0) else normalize(rotate_vector(fallback_rotation, (0.0, 0.0, 1.0)))

    def _pair_vector_from_solved(self, solved_by_name: dict[str, object], left_name: str, right_name: str, fallback_rotation) -> Vec3:
        left = solved_by_name.get(left_name)
        right = solved_by_name.get(right_name)
        if left is None or right is None:
            return normalize(rotate_vector(fallback_rotation, (0.0, 0.0, 1.0)))
        vector = subtract(right.world_position, left.world_position)
        return normalize(vector) if vector != (0.0, 0.0, 0.0) else normalize(rotate_vector(fallback_rotation, (0.0, 0.0, 1.0)))

    def _local_delta_from_world(self, bone: BoneModel, parent_world_rotation, desired_world_rotation):
        local_rotation = normalize_quaternion(quaternion_multiply(inverse_quaternion(parent_world_rotation), desired_world_rotation))
        return normalize_quaternion(quaternion_multiply(local_rotation, inverse_quaternion(bone.rest_local_rotation)))

    @staticmethod
    def _local_flexion_axis(bone: BoneModel) -> Vec3:
        axis = normalize(project_on_plane((1.0, 0.0, 0.0), bone.primary_axis))
        if length(axis) > 1e-6:
            return axis
        axis = normalize(project_on_plane((0.0, 0.0, 1.0), bone.primary_axis))
        return axis if length(axis) > 1e-6 else (1.0, 0.0, 0.0)

    @staticmethod
    def _local_lateral_axis(bone: BoneModel, flexion_axis: Vec3) -> Vec3:
        axis = normalize(project_on_plane((0.0, 0.0, 1.0), bone.primary_axis))
        if length(axis) > 1e-6 and abs(sum(axis[index] * flexion_axis[index] for index in range(3))) < 0.95:
            return axis
        axis = normalize(project_on_plane((0.0, 1.0, 0.0), bone.primary_axis))
        return axis if length(axis) > 1e-6 else (0.0, 0.0, 1.0)

    @staticmethod
    def _foot_flexion_axis(bone: BoneModel) -> Vec3:
        axis = normalize(project_on_plane((1.0, 0.0, 0.0), bone.primary_axis))
        if length(axis) > 1e-6:
            return axis
        axis = normalize(project_on_plane((0.0, 1.0, 0.0), bone.primary_axis))
        return axis if length(axis) > 1e-6 else (1.0, 0.0, 0.0)

    def _parent_world_rotation(
        self,
        bone: BoneModel,
        desired_world_rotations: dict[str, tuple[float, float, float, float]],
        rest_by_name: dict[str, BoneModel],
    ):
        if bone.parent_name in desired_world_rotations:
            return desired_world_rotations[bone.parent_name]
        if bone.parent_name in rest_by_name:
            return rest_by_name[bone.parent_name].rest_world_rotation
        return normalize_quaternion(
            quaternion_multiply(
                bone.rest_world_rotation,
                inverse_quaternion(bone.rest_local_rotation),
            )
        )

    def _twist_ratio(self, parent_bone: BoneModel, twist_bone: BoneModel, child_bone: BoneModel | None) -> float:
        if child_bone is None:
            return 0.5
        segment_length = distance(parent_bone.rest_world_position, child_bone.rest_world_position)
        if segment_length <= 1e-6:
            return 0.5
        twist_distance = distance(parent_bone.rest_world_position, twist_bone.rest_world_position)
        return max(0.0, min(1.0, twist_distance / segment_length))

    @staticmethod
    def _controller_engaged(controller) -> bool:
        return controller is not None and (controller.active or controller.fixed or controller.always_active)
