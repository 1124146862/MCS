from dataclasses import dataclass

from PySide6.QtCore import QObject
from PySide6.QtGui import QQuaternion, QVector3D

from core.autoposing.models import RetargetPoseModel
from core.skeleton.models import BoneModel, SkeletonModel


@dataclass(slots=True)
class RuntimeJointBinding:
    name: str
    node: QObject
    parent_name: str | None
    rest_local_position: QVector3D
    rest_local_rotation: QQuaternion
    rest_world_position: QVector3D
    rest_world_rotation: QQuaternion


class RuntimePoseWriter:
    def __init__(self) -> None:
        self._runtime_root: QObject | None = None
        self._bindings: dict[str, RuntimeJointBinding] = {}
        self._apply_order: list[str] = []

    def clear(self) -> None:
        self._runtime_root = None
        self._bindings = {}
        self._apply_order = []

    def bind_runtime_model(self, runtime_root: QObject | None) -> bool:
        self.clear()
        if runtime_root is None:
            return False

        bindings, apply_order = self._build_bindings(runtime_root)
        if not bindings:
            return False

        self._runtime_root = runtime_root
        self._bindings = bindings
        self._apply_order = apply_order
        return True

    def apply_retarget_pose(self, pose: RetargetPoseModel) -> bool:
        if self._runtime_root is None or not self._bindings:
            return False

        pose_by_name = {joint.name: joint for joint in pose.joints}
        world_positions: dict[str, QVector3D] = {}
        world_rotations: dict[str, QQuaternion] = {}

        for joint_name in self._apply_order:
            binding = self._bindings[joint_name]
            pose_joint = pose_by_name.get(joint_name)
            parent_binding = self._bindings.get(binding.parent_name) if binding.parent_name else None
            parent_world_position = (
                world_positions.get(binding.parent_name, parent_binding.rest_world_position)
                if parent_binding is not None
                else QVector3D()
            )
            parent_world_rotation = (
                world_rotations.get(binding.parent_name, parent_binding.rest_world_rotation)
                if parent_binding is not None
                else QQuaternion()
            )

            local_position = binding.rest_local_position
            local_rotation = binding.rest_local_rotation
            if pose_joint is not None:
                if pose_joint.world_position is not None:
                    local_position = self._world_to_local_position(
                        self._vector3d_from_tuple(pose_joint.world_position),
                        parent_world_position,
                        parent_world_rotation,
                    )
                else:
                    local_position = self._vector3d_from_tuple(pose_joint.local_position)
                local_rotation = self._quaternion_from_tuple(pose_joint.local_rotation)

            binding.node.setProperty("position", local_position)
            binding.node.setProperty("rotation", local_rotation)
            world_positions[joint_name] = self._vector_add(
                parent_world_position,
                parent_world_rotation.rotatedVector(local_position),
            )
            world_rotations[joint_name] = self._normalize_quaternion(parent_world_rotation * local_rotation)

        return True

    def capture_runtime_skeleton(self, reference_skeleton: SkeletonModel) -> SkeletonModel | None:
        if self._runtime_root is None or not reference_skeleton.bones:
            return None

        target_names = {bone.name for bone in reference_skeleton.bones}
        by_name = {bone.name: bone for bone in reference_skeleton.bones}
        captured_bones: list[BoneModel] = []
        world_positions: dict[str, QVector3D] = {}
        world_rotations: dict[str, QQuaternion] = {}
        local_positions: dict[str, QVector3D] = {}
        local_rotations: dict[str, QQuaternion] = {}

        for joint_name in self._apply_order:
            binding = self._bindings[joint_name]
            parent_binding = self._bindings.get(binding.parent_name) if binding.parent_name else None
            parent_world_position = (
                world_positions.get(binding.parent_name, parent_binding.rest_world_position)
                if parent_binding is not None
                else QVector3D()
            )
            parent_world_rotation = (
                world_rotations.get(binding.parent_name, parent_binding.rest_world_rotation)
                if parent_binding is not None
                else QQuaternion()
            )

            local_position = self._read_vector3d_property(binding.node, "position")
            local_rotation = self._read_quaternion_property(binding.node, "rotation")
            local_positions[joint_name] = local_position
            local_rotations[joint_name] = local_rotation
            world_positions[joint_name] = self._vector_add(
                parent_world_position,
                parent_world_rotation.rotatedVector(local_position),
            )
            world_rotations[joint_name] = self._normalize_quaternion(parent_world_rotation * local_rotation)

        for bone_name in target_names:
            reference = by_name[bone_name]
            world_position = world_positions.get(bone_name, self._vector3d_from_tuple(reference.world_position))
            world_rotation = world_rotations.get(bone_name, self._quaternion_from_tuple(reference.world_rotation))
            local_position = local_positions.get(bone_name, self._vector3d_from_tuple(reference.local_position))
            local_rotation = local_rotations.get(bone_name, self._quaternion_from_tuple(reference.local_rotation))
            captured_bones.append(
                BoneModel(
                    name=reference.name,
                    parent_name=reference.parent_name,
                    local_position=self._vector_tuple(local_position),
                    local_rotation=self._quat_tuple(local_rotation),
                    world_position=self._vector_tuple(world_position),
                    world_rotation=self._quat_tuple(world_rotation),
                    rest_local_position=reference.rest_local_position,
                    rest_local_rotation=reference.rest_local_rotation,
                    rest_world_position=reference.rest_world_position,
                    rest_world_rotation=reference.rest_world_rotation,
                    bone_length=reference.bone_length,
                    primary_axis=reference.primary_axis,
                )
            )

        if not captured_bones:
            return None
        return SkeletonModel(name=reference_skeleton.name, bones=captured_bones)

    def _build_bindings(self, runtime_root: QObject) -> tuple[dict[str, RuntimeJointBinding], list[str]]:
        bindings: dict[str, RuntimeJointBinding] = {}
        apply_order: list[str] = []

        def walk(node: QObject, parent_joint_name: str | None, parent_world_position: QVector3D, parent_world_rotation: QQuaternion) -> None:
            local_position = self._read_vector3d_property(node, "position")
            local_rotation = self._read_quaternion_property(node, "rotation")
            world_position = self._vector_add(parent_world_position, parent_world_rotation.rotatedVector(local_position))
            world_rotation = self._normalize_quaternion(parent_world_rotation * local_rotation)

            current_joint_name = parent_joint_name
            object_name = node.property("objectName")
            if isinstance(object_name, str) and object_name not in {"RootNode"}:
                binding = RuntimeJointBinding(
                    name=object_name,
                    node=node,
                    parent_name=parent_joint_name,
                    rest_local_position=local_position,
                    rest_local_rotation=local_rotation,
                    rest_world_position=world_position,
                    rest_world_rotation=world_rotation,
                )
                bindings[object_name] = binding
                apply_order.append(object_name)
                current_joint_name = object_name

            for child in node.children():
                if isinstance(child, QObject):
                    walk(child, current_joint_name, world_position, world_rotation)

        walk(runtime_root, None, QVector3D(), QQuaternion())
        return bindings, apply_order

    @staticmethod
    def _read_vector3d_property(node: QObject, property_name: str) -> QVector3D:
        value = node.property(property_name)
        return value if isinstance(value, QVector3D) else QVector3D()

    @staticmethod
    def _read_quaternion_property(node: QObject, property_name: str) -> QQuaternion:
        value = node.property(property_name)
        return RuntimePoseWriter._normalize_quaternion(value if isinstance(value, QQuaternion) else QQuaternion())

    @staticmethod
    def _world_to_local_position(desired_world_position: QVector3D, parent_world_position: QVector3D, parent_world_rotation: QQuaternion) -> QVector3D:
        return parent_world_rotation.inverted().rotatedVector(desired_world_position - parent_world_position)

    @staticmethod
    def _vector_add(left: QVector3D, right: QVector3D) -> QVector3D:
        return QVector3D(left.x() + right.x(), left.y() + right.y(), left.z() + right.z())

    @staticmethod
    def _vector_tuple(vector: QVector3D) -> tuple[float, float, float]:
        return (vector.x(), vector.y(), vector.z())

    @staticmethod
    def _quat_tuple(quaternion: QQuaternion) -> tuple[float, float, float, float]:
        return (quaternion.scalar(), quaternion.x(), quaternion.y(), quaternion.z())

    @staticmethod
    def _vector3d_from_tuple(vector: tuple[float, float, float]) -> QVector3D:
        return QVector3D(vector[0], vector[1], vector[2])

    @staticmethod
    def _quaternion_from_tuple(quaternion: tuple[float, float, float, float]) -> QQuaternion:
        return QQuaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])

    @staticmethod
    def _normalize_quaternion(quaternion: QQuaternion) -> QQuaternion:
        length = (
            quaternion.scalar() * quaternion.scalar()
            + quaternion.x() * quaternion.x()
            + quaternion.y() * quaternion.y()
            + quaternion.z() * quaternion.z()
        ) ** 0.5
        if length <= 0.000001:
            return QQuaternion()
        return QQuaternion(
            quaternion.scalar() / length,
            quaternion.x() / length,
            quaternion.y() / length,
            quaternion.z() / length,
        )
