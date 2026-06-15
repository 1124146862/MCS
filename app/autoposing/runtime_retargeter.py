from core.autoposing.models import AutoPosingRigModel, RetargetJointPoseModel, RetargetPoseModel, SolvedPoseModel
from core.math import (
    add,
    angle_between_on_axis,
    distance,
    extract_twist_rotation,
    inverse_quaternion,
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

    def retarget(self, rig: AutoPosingRigModel, solved_pose: SolvedPoseModel, skeleton: SkeletonModel) -> RetargetPoseModel:
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
            world_position = solved_by_name[name].world_position if name == "pelvis" and name in solved_by_name else None
            pose_map[name] = RetargetJointPoseModel(
                name=name,
                local_position=bone.rest_local_position,
                local_rotation=normalize_quaternion(local_rotation),
                world_position=world_position,
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
            hand_rotation = self._terminal_rotation_from_controller(
                bone,
                solved_by_name,
                controllers_by_id,
                effectors_by_id,
                ("hand_l_additional", "hand_l_secondary"),
            )
            return self._blend_finger_palm_spread(bone, "l", hand_rotation, controllers_by_id, effectors_by_id)
        if bone.name == "hand_r":
            hand_rotation = self._terminal_rotation_from_controller(
                bone,
                solved_by_name,
                controllers_by_id,
                effectors_by_id,
                ("hand_r_additional", "hand_r_secondary"),
            )
            return self._blend_finger_palm_spread(bone, "r", hand_rotation, controllers_by_id, effectors_by_id)
        if bone.name == "head":
            return self._terminal_rotation_from_controller(
                bone,
                solved_by_name,
                controllers_by_id,
                effectors_by_id,
                ("head_additional",),
            )
        return None

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
            additional = self._engaged_effector_vector(
                ("pelvis_additional",),
                bone_name,
                solved_by_name,
                controllers_by_id,
                effectors_by_id,
            )
            if additional is not None:
                return additional
            direction_controller = controllers_by_id.get("pelvis_dir")
            if self._controller_engaged(direction_controller):
                return subtract(direction_controller.target.world_position, solved_by_name[bone_name].world_position)
            return self._pair_vector_from_solved(solved_by_name, "thigh_l", "thigh_r", rest_by_name[bone_name].rest_world_rotation)
        if bone_name in {"spine_01", "spine_02", "spine_03", "spine_04", "spine_05"}:
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
            direction_controller = controllers_by_id.get("chest_dir")
            if self._controller_engaged(direction_controller) and bone_name == "spine_05":
                return subtract(direction_controller.target.world_position, solved_by_name[bone_name].world_position)
            return self._pair_vector_from_solved(solved_by_name, "clavicle_l", "clavicle_r", rest_by_name[bone_name].rest_world_rotation)
        if bone_name in {"neck_01", "neck_02", "head"}:
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
            direction_controller = controllers_by_id.get("head_dir")
            if self._controller_engaged(direction_controller):
                return subtract(direction_controller.target.world_position, solved_by_name[bone_name].world_position)
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
        if bone_name == "foot_l":
            foot_hint = self._engaged_effector_vector(
                ("foot_l_additional", "foot_l_secondary", "ball_l_lesser"),
                bone_name,
                solved_by_name,
                controllers_by_id,
                effectors_by_id,
            )
            if foot_hint is not None:
                return foot_hint
        if bone_name == "foot_r":
            foot_hint = self._engaged_effector_vector(
                ("foot_r_additional", "foot_r_secondary", "ball_r_lesser"),
                bone_name,
                solved_by_name,
                controllers_by_id,
                effectors_by_id,
            )
            if foot_hint is not None:
                return foot_hint
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
