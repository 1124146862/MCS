from core.autoposing.models import (
    AutoPosingRigModel,
    ControllerTargetModel,
    SolvedPoseModel,
)
from core.rig.models import RigModel
from core.skeleton.models import BoneModel, SkeletonModel

from .controller_state import (
    controller_state,
    move_controller,
    reset_controller,
    select_controller,
    set_mode,
    toggle_fixed,
)
from .solver import AutoPosingSolver


class AutoPosingService:
    _JOINT_ALIASES = {
        "pelvis": ("pelvis",),
        "chest": ("spine_05", "spine_04", "spine_03", "spine_02"),
        "head": ("head",),
        "wrist_l": ("hand_l",),
        "wrist_r": ("hand_r",),
        "elbow_l": ("lowerarm_l",),
        "elbow_r": ("lowerarm_r",),
        "ankle_l": ("foot_l",),
        "ankle_r": ("foot_r",),
        "knee_l": ("calf_l",),
        "knee_r": ("calf_r",),
    }
    _FORWARD_OFFSET = {
        "pelvis_dir": (0.0, 0.0, 24.0),
        "chest_dir": (0.0, 6.0, 26.0),
        "head_dir": (0.0, 0.0, 22.0),
    }
    _ALWAYS_ACTIVE_IDS = {"ankle_l_main", "ankle_r_main"}
    _CONTROLLER_LAYOUT = (
        ("pelvis_main", "Pelvis", "main", "pelvis"),
        ("chest_secondary", "Chest", "secondary", "chest"),
        ("head_main", "Head", "main", "head"),
        ("wrist_l_main", "Wrist L", "main", "wrist_l"),
        ("elbow_l_secondary", "Elbow L", "secondary", "elbow_l"),
        ("wrist_r_main", "Wrist R", "main", "wrist_r"),
        ("elbow_r_secondary", "Elbow R", "secondary", "elbow_r"),
        ("ankle_l_main", "Ankle L", "main", "ankle_l"),
        ("knee_l_secondary", "Knee L", "secondary", "knee_l"),
        ("ankle_r_main", "Ankle R", "main", "ankle_r"),
        ("knee_r_secondary", "Knee R", "secondary", "knee_r"),
        ("pelvis_dir", "Pelvis Direction", "direction", "pelvis"),
        ("chest_dir", "Chest Direction", "direction", "chest"),
        ("head_dir", "Head Direction", "direction", "head"),
    )

    def __init__(self) -> None:
        self._solver = AutoPosingSolver()

    def build_rig(self, skeleton: SkeletonModel) -> AutoPosingRigModel:
        by_name = {bone.name: bone for bone in skeleton.bones}
        controllers = []
        for identifier, name, controller_type, semantic_joint in self._CONTROLLER_LAYOUT:
            joint = self._resolve_joint(by_name, semantic_joint)
            if joint is None:
                continue

            world_position = joint.rest_world_position
            if controller_type == "direction":
                offset = self._FORWARD_OFFSET.get(identifier, (0.0, 0.0, 20.0))
                world_position = (
                    world_position[0] + offset[0],
                    world_position[1] + offset[1],
                    world_position[2] + offset[2],
                )

            controllers.append(
                self._controller_model(
                    identifier=identifier,
                    name=name,
                    controller_type=controller_type,
                    joint_name=joint.name,
                    world_position=world_position,
                )
            )
        return AutoPosingRigModel(controllers=controllers, current_mode="view", selected_controller_id=None)

    def solve_pose(self, rig: AutoPosingRigModel, skeleton: SkeletonModel) -> SolvedPoseModel:
        solved_pose = self._solver.solve(rig, skeleton)
        effectors_by_id = {effector.controller_id: effector for effector in solved_pose.effectors}
        for controller in rig.controllers:
            effector = effectors_by_id.get(controller.identifier)
            if effector is None:
                continue
            controller.target.solved_world_position = effector.solved_world_position
            controller.target.clamped_world_position = effector.clamped_world_position
            controller.target.pressure = effector.pressure
            controller.target.pressure_state = effector.pressure_state
            controller.target.clamped = effector.clamped
        return solved_pose

    def build_solver_rig(self, rig: AutoPosingRigModel, skeleton: SkeletonModel) -> RigModel:
        return self.solve_pose(rig, skeleton).rig

    def solve_preview_skeleton(self, rig: AutoPosingRigModel, skeleton: SkeletonModel) -> SkeletonModel:
        solved_pose = self.solve_pose(rig, skeleton)
        by_name = {bone.name: bone for bone in skeleton.bones}
        return SkeletonModel(
            name=solved_pose.rig.name,
            bones=[
                BoneModel(
                    name=joint.name,
                    parent_name=joint.parent_name,
                    local_position=by_name.get(joint.name, BoneModel(joint.name)).local_position,
                    local_rotation=by_name.get(joint.name, BoneModel(joint.name)).local_rotation,
                    world_position=joint.world_position,
                    world_rotation=joint.world_rotation,
                    rest_local_position=joint.rest_local_position,
                    rest_local_rotation=joint.rest_local_rotation,
                    rest_world_position=joint.rest_world_position,
                    rest_world_rotation=joint.rest_world_rotation,
                    bone_length=joint.bone_length,
                    primary_axis=joint.primary_axis,
                )
                for joint in solved_pose.rig.joints
            ],
        )

    def set_mode(self, rig: AutoPosingRigModel, mode: str) -> None:
        set_mode(rig, mode)

    def select_controller(self, rig: AutoPosingRigModel, controller_id: str | None) -> None:
        select_controller(rig, controller_id)

    def move_controller(self, rig: AutoPosingRigModel, controller_id: str, world_position: tuple[float, float, float]) -> bool:
        return move_controller(rig, controller_id, world_position)

    def toggle_fixed(
        self,
        rig: AutoPosingRigModel,
        controller_id: str,
        display_world_position: tuple[float, float, float] | None = None,
    ) -> bool:
        return toggle_fixed(rig, controller_id, display_world_position)

    def reset_controller(self, rig: AutoPosingRigModel, controller_id: str) -> bool:
        return reset_controller(rig, controller_id)

    def controller_state(self, controller) -> str:
        return controller_state(controller)

    @staticmethod
    def _resolve_joint(by_name: dict[str, BoneModel], semantic_joint: str) -> BoneModel | None:
        for alias in AutoPosingService._JOINT_ALIASES.get(semantic_joint, (semantic_joint,)):
            joint = by_name.get(alias)
            if joint is not None:
                return joint
        return None

    @staticmethod
    def _controller_model(identifier: str, name: str, controller_type: str, joint_name: str, world_position):
        from core.autoposing.models import AutoPosingControllerModel

        return AutoPosingControllerModel(
            identifier=identifier,
            name=name,
            controller_type=controller_type,
            joint_name=joint_name,
            target=ControllerTargetModel(
                controller_id=identifier,
                world_position=world_position,
                default_position=world_position,
                solved_world_position=world_position,
                clamped_world_position=world_position,
            ),
            active=identifier in AutoPosingService._ALWAYS_ACTIVE_IDS,
            always_active=identifier in AutoPosingService._ALWAYS_ACTIVE_IDS,
        )
