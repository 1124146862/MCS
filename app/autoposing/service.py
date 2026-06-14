from dataclasses import replace

from core.autoposing.models import AutoPosingControllerModel, AutoPosingRigModel
from core.skeleton.models import BoneModel, SkeletonModel


class AutoPosingService:
    """Creates and updates the first-pass AutoPosing controller rig."""

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

    def build_rig(self, skeleton: SkeletonModel) -> AutoPosingRigModel:
        by_name = {bone.name: bone for bone in skeleton.bones}
        controllers: list[AutoPosingControllerModel] = []

        for identifier, name, controller_type, semantic_joint in self._CONTROLLER_LAYOUT:
            joint = self._resolve_joint(by_name, semantic_joint)
            if joint is None:
                continue

            world_position = joint.world_position
            if controller_type == "direction":
                offset = self._FORWARD_OFFSET.get(identifier, (0.0, 0.0, 20.0))
                world_position = (
                    world_position[0] + offset[0],
                    world_position[1] + offset[1],
                    world_position[2] + offset[2],
                )

            controllers.append(
                AutoPosingControllerModel(
                    identifier=identifier,
                    name=name,
                    controller_type=controller_type,
                    joint_name=joint.name,
                    world_position=world_position,
                    default_position=world_position,
                )
            )

        return AutoPosingRigModel(controllers=controllers, current_mode="view", selected_controller_id=None)

    def set_mode(self, rig: AutoPosingRigModel, mode: str) -> None:
        rig.current_mode = mode
        if mode != "autoposing":
            self.clear_selection(rig)

    def select_controller(self, rig: AutoPosingRigModel, controller_id: str | None) -> None:
        rig.selected_controller_id = controller_id
        for controller in rig.controllers:
            controller.selected = controller.identifier == controller_id

    def clear_selection(self, rig: AutoPosingRigModel) -> None:
        self.select_controller(rig, None)

    def move_controller(
        self,
        rig: AutoPosingRigModel,
        controller_id: str,
        world_position: tuple[float, float, float],
    ) -> bool:
        controller = self._controller_by_id(rig, controller_id)
        if controller is None:
            return False

        controller.world_position = world_position
        controller.active = True
        self.select_controller(rig, controller_id)
        return True

    def toggle_locked(self, rig: AutoPosingRigModel, controller_id: str) -> bool:
        controller = self._controller_by_id(rig, controller_id)
        if controller is None:
            return False

        controller.locked = not controller.locked
        if controller.locked:
            controller.active = True
        self.select_controller(rig, controller_id)
        return controller.locked

    def reset_controller(self, rig: AutoPosingRigModel, controller_id: str) -> bool:
        controller = self._controller_by_id(rig, controller_id)
        if controller is None:
            return False

        controller.world_position = controller.default_position
        controller.active = False
        controller.locked = False
        self.select_controller(rig, controller_id)
        return True

    @staticmethod
    def _controller_by_id(rig: AutoPosingRigModel, controller_id: str) -> AutoPosingControllerModel | None:
        return next((controller for controller in rig.controllers if controller.identifier == controller_id), None)

    def _resolve_joint(self, by_name: dict[str, BoneModel], semantic_joint: str) -> BoneModel | None:
        for alias in self._JOINT_ALIASES.get(semantic_joint, (semantic_joint,)):
            joint = by_name.get(alias)
            if joint is not None:
                return joint
        return None
