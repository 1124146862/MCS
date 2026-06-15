from core.autoposing.models import AutoPosingRigModel, RetargetJointPoseModel, RetargetPoseModel, SolvedPoseModel
from core.math import (
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
    subtract,
)
from core.skeleton.models import BoneModel, SkeletonModel, Vec3


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
            desired_world_rotations[ball_name] = desired_world_rotation
            parent_rotation = self._parent_world_rotation(ball_bone, desired_world_rotations, rest_by_name)
            pose_map[ball_name] = RetargetJointPoseModel(
                name=ball_name,
                local_position=ball_bone.rest_local_position,
                local_rotation=normalize_quaternion(
                    quaternion_multiply(inverse_quaternion(parent_rotation), desired_world_rotation)
                ),
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
            return self._pair_vector_from_solved(solved_by_name, "thigh_l", "thigh_r", rest_by_name[bone_name].rest_world_rotation)
        if bone_name in {"spine_01", "spine_02", "spine_03", "spine_04", "spine_05"}:
            direction_controller = controllers_by_id.get("chest_dir")
            if self._controller_engaged(direction_controller) and bone_name == "spine_05":
                return subtract(direction_controller.target.world_position, solved_by_name[bone_name].world_position)
            return self._pair_vector_from_solved(solved_by_name, "clavicle_l", "clavicle_r", rest_by_name[bone_name].rest_world_rotation)
        if bone_name in {"neck_01", "neck_02", "head"}:
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
        return normalize(rotate_vector(rest_by_name[bone_name].rest_world_rotation, (0.0, 0.0, 1.0)))

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
