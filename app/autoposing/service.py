from core.autoposing.models import (
    AutoPosingControllerEdgeModel,
    AutoPosingRigModel,
    ControllerTargetModel,
    SolvedPoseModel,
)
from core.math import add, distance, subtract
from core.rig.models import RigModel
from core.skeleton.models import BoneModel, SkeletonModel

from .controller_state import (
    controller_state,
    fix_controller,
    lock_controller,
    move_controller,
    reset_controller,
    select_controller,
    set_mode,
    toggle_fixed,
    unfix_controller,
    unlock_controller,
)
from .controller_schema import AutoPosingControllerDefinition, build_cascadeur_like_controller_schema
from .finger_profile import FINGER_ROLES, is_finger_controller_id
from .finger_schema import build_finger_controller_schema
from .solver import AutoPosingSolver


class AutoPosingService:
    def __init__(self) -> None:
        self._solver = AutoPosingSolver()

    def build_rig(self, skeleton: SkeletonModel) -> AutoPosingRigModel:
        schema = build_cascadeur_like_controller_schema(skeleton)
        finger_schema = build_finger_controller_schema(skeleton)
        controller_definitions = [*schema.controllers, *finger_schema.controllers]
        edge_definitions = [*schema.edges, *finger_schema.edges]
        controllers = [self._controller_model(definition) for definition in controller_definitions]
        controller_positions = {controller.identifier: controller.target.world_position for controller in controllers}
        edges = [
            AutoPosingControllerEdgeModel(
                start_controller_id=edge.start_controller_id,
                end_controller_id=edge.end_controller_id,
                edge_type=edge.edge_type,
                rest_length=distance(controller_positions[edge.start_controller_id], controller_positions[edge.end_controller_id]),
                visible=edge.visible,
            )
            for edge in edge_definitions
            if edge.start_controller_id in controller_positions and edge.end_controller_id in controller_positions
        ]
        return AutoPosingRigModel(controllers=controllers, controller_edges=edges, current_mode="view", selected_controller_id=None)

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

    def reset_controller(
        self,
        rig: AutoPosingRigModel,
        controller_id: str,
        solved_pose: SolvedPoseModel | None = None,
    ) -> bool:
        controller = next((controller for controller in rig.controllers if controller.identifier == controller_id), None)
        reset_position = self._reset_world_position(controller, solved_pose)
        return reset_controller(rig, controller_id, reset_position)

    def lock_controller(self, rig: AutoPosingRigModel, controller_id: str) -> bool:
        return lock_controller(rig, controller_id)

    def unlock_controller(self, rig: AutoPosingRigModel, controller_id: str) -> bool:
        return unlock_controller(rig, controller_id)

    def fix_controller(
        self,
        rig: AutoPosingRigModel,
        controller_id: str,
        display_world_position: tuple[float, float, float] | None = None,
    ) -> bool:
        return fix_controller(rig, controller_id, display_world_position)

    def unfix_controller(self, rig: AutoPosingRigModel, controller_id: str) -> bool:
        return unfix_controller(rig, controller_id)

    def controller_state(self, controller) -> str:
        return controller_state(controller)

    def has_finger_controllers(self, rig: AutoPosingRigModel) -> bool:
        return any(controller.role in FINGER_ROLES or is_finger_controller_id(controller.identifier) for controller in rig.controllers)

    def set_fingers_enabled(self, rig: AutoPosingRigModel, enabled: bool) -> bool:
        if not self.has_finger_controllers(rig):
            enabled = False
        if rig.fingers_enabled == enabled and (not enabled or rig.finger_defaults_initialized):
            return False

        rig.fingers_enabled = enabled
        finger_ids = set()
        for controller in rig.controllers:
            if controller.role in FINGER_ROLES or is_finger_controller_id(controller.identifier):
                controller.visible = enabled
                finger_ids.add(controller.identifier)

        for edge in rig.controller_edges:
            if edge.start_controller_id in finger_ids or edge.end_controller_id in finger_ids:
                edge.visible = enabled

        if enabled and not rig.finger_defaults_initialized:
            for controller in rig.controllers:
                if controller.identifier in {"index_l_tip", "index_r_tip"}:
                    controller.active = True
            rig.finger_defaults_initialized = True

        if not enabled and rig.selected_controller_id in finger_ids:
            select_controller(rig, None)
        return True

    def toggle_fingers_enabled(self, rig: AutoPosingRigModel) -> bool:
        return self.set_fingers_enabled(rig, not rig.fingers_enabled)

    @staticmethod
    def _reset_world_position(controller, solved_pose: SolvedPoseModel | None):
        if controller is None or solved_pose is None:
            return None
        if controller.role not in FINGER_ROLES and not is_finger_controller_id(controller.identifier):
            return None
        joint = next((joint for joint in solved_pose.rig.joints if joint.name == controller.joint_name), None)
        if joint is None:
            return None
        offset = subtract(controller.target.default_position, joint.rest_world_position)
        return add(joint.world_position, offset)

    @staticmethod
    def _controller_model(definition: AutoPosingControllerDefinition):
        from core.autoposing.models import AutoPosingControllerModel

        return AutoPosingControllerModel(
            identifier=definition.identifier,
            name=definition.name,
            controller_type=definition.role,
            role=definition.role,
            joint_name=definition.joint_name,
            target=ControllerTargetModel(
                controller_id=definition.identifier,
                world_position=definition.world_position,
                default_position=definition.world_position,
                solved_world_position=definition.world_position,
                clamped_world_position=definition.world_position,
            ),
            visible=definition.visible,
            active=definition.always_active,
            always_active=definition.always_active,
        )
