from core.autoposing.models import AutoPosingControllerModel, AutoPosingRigModel
from core.ik.two_bone import (
    TwoBoneIkSolution,
    add,
    distance,
    dot,
    lerp,
    scale,
    solve_two_bone_ik,
    subtract,
)
from core.rig.models import JointModel, RigModel
from core.skeleton.models import BoneModel, SkeletonModel


class AutoPosingService:
    """Creates controller rigs, proxy rigs, and V2 preview solving."""

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

    _PROXY_RIG_LAYOUT = (
        ("pelvis", None),
        ("spine_01", "pelvis"),
        ("spine_02", "spine_01"),
        ("spine_03", "spine_02"),
        ("spine_04", "spine_03"),
        ("spine_05", "spine_04"),
        ("neck_01", "spine_05"),
        ("head", "neck_01"),
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
        ("thigh_r", "pelvis"),
        ("calf_r", "thigh_r"),
        ("foot_r", "calf_r"),
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
                world_position = add(world_position, self._FORWARD_OFFSET.get(identifier, (0.0, 0.0, 20.0)))

            controllers.append(
                AutoPosingControllerModel(
                    identifier=identifier,
                    name=name,
                    controller_type=controller_type,
                    joint_name=joint.name,
                    world_position=world_position,
                    default_position=world_position,
                    active=identifier in self._ALWAYS_ACTIVE_IDS,
                    always_active=identifier in self._ALWAYS_ACTIVE_IDS,
                )
            )

        return AutoPosingRigModel(controllers=controllers, current_mode="view", selected_controller_id=None)

    def build_proxy_rig(self, rig: AutoPosingRigModel, skeleton: SkeletonModel) -> RigModel:
        proxy_rig, _ = self._solve_proxy_rig_data(rig, skeleton)
        return proxy_rig

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
        if controller.controller_type == "direction":
            controller.fixed = True
        self.select_controller(rig, controller_id)
        return True

    def toggle_fixed(
        self,
        rig: AutoPosingRigModel,
        controller_id: str,
        display_world_position: tuple[float, float, float] | None = None,
    ) -> bool:
        controller = self._controller_by_id(rig, controller_id)
        if controller is None:
            return False

        was_engaged = self._controller_engaged(controller)
        controller.fixed = not controller.fixed

        if controller.fixed:
            if not was_engaged and display_world_position is not None:
                controller.world_position = display_world_position
            controller.active = True

        self.select_controller(rig, controller_id)
        return controller.fixed

    def reset_controller(self, rig: AutoPosingRigModel, controller_id: str) -> bool:
        controller = self._controller_by_id(rig, controller_id)
        if controller is None:
            return False

        controller.world_position = controller.default_position
        controller.active = controller.always_active
        controller.fixed = False
        self.select_controller(rig, controller_id)
        return True

    def controller_state(self, controller: AutoPosingControllerModel) -> str:
        if controller.fixed:
            return "fixed"
        if controller.active or controller.always_active:
            return "active"
        return "inactive"

    def solve_proxy_rig(self, rig: AutoPosingRigModel, skeleton: SkeletonModel) -> RigModel:
        proxy_rig, _ = self._solve_proxy_rig_data(rig, skeleton)
        return proxy_rig

    def solve_preview_skeleton(self, rig: AutoPosingRigModel, skeleton: SkeletonModel) -> SkeletonModel:
        proxy_rig = self.solve_proxy_rig(rig, skeleton)
        return SkeletonModel(
            name=proxy_rig.name,
            bones=[
                BoneModel(
                    name=joint.name,
                    parent_name=joint.parent_name,
                    world_position=joint.world_position,
                )
                for joint in proxy_rig.joints
            ],
        )

    def solve_controller_positions(
        self,
        rig: AutoPosingRigModel,
        proxy_rig: RigModel,
    ) -> dict[str, tuple[float, float, float]]:
        return self._solve_controller_display_positions(rig, proxy_rig)

    def _solve_proxy_rig_data(
        self,
        rig: AutoPosingRigModel,
        skeleton: SkeletonModel,
    ) -> tuple[RigModel, dict[str, tuple[float, float, float]]]:
        if not skeleton.bones:
            return RigModel(name="AutoPosing ProxyRig"), {}

        rest_by_name = {bone.name: bone for bone in skeleton.bones}
        controller_by_id = {controller.identifier: controller for controller in rig.controllers}

        rest_pelvis = self._rest_position(rest_by_name, "pelvis")
        rest_chest = self._rest_position(rest_by_name, "spine_05")
        rest_head = self._rest_position(rest_by_name, "head")
        rest_hand_l = self._rest_position(rest_by_name, "hand_l")
        rest_hand_r = self._rest_position(rest_by_name, "hand_r")
        rest_foot_l = self._rest_position(rest_by_name, "foot_l")
        rest_foot_r = self._rest_position(rest_by_name, "foot_r")

        ankle_l_target = self._controller_target_position(controller_by_id, "ankle_l_main", rest_foot_l)
        ankle_r_target = self._controller_target_position(controller_by_id, "ankle_r_main", rest_foot_r)
        ankle_delta = self._average_delta(
            (rest_foot_l, rest_foot_r),
            (ankle_l_target, ankle_r_target),
        )

        pelvis_default = add(rest_pelvis, scale(ankle_delta, 0.75))
        pelvis = self._controller_target_position(controller_by_id, "pelvis_main", pelvis_default)

        head_default = add(rest_head, scale(ankle_delta, 0.2))
        head = self._controller_target_position(controller_by_id, "head_main", head_default)

        chest_default = add(
            rest_chest,
            add(
                scale(subtract(pelvis, rest_pelvis), 0.7),
                scale(subtract(head, rest_head), 0.35),
            ),
        )
        chest = self._controller_target_position(controller_by_id, "chest_secondary", chest_default)

        wrist_l_target = self._resolve_arm_target(
            controller_by_id,
            rest_by_name,
            "wrist_l_main",
            "elbow_l_secondary",
            chest,
            "hand_l",
            "lowerarm_l",
        )
        wrist_r_target = self._resolve_arm_target(
            controller_by_id,
            rest_by_name,
            "wrist_r_main",
            "elbow_r_secondary",
            chest,
            "hand_r",
            "lowerarm_r",
        )
        wrist_delta = self._average_delta(
            (rest_hand_l, rest_hand_r),
            (wrist_l_target, wrist_r_target),
        )

        if not self._controller_is_engaged(controller_by_id, "head_main"):
            head = add(head, scale(wrist_delta, 0.16))
        if not self._controller_is_engaged(controller_by_id, "chest_secondary"):
            chest = add(chest, scale(wrist_delta, 0.28))

        spine_positions = self._solve_spine_chain(rest_by_name, pelvis, chest, head)

        clavicle_l = self._offset_from_reference(rest_by_name, "spine_05", "clavicle_l", spine_positions["spine_05"])
        clavicle_r = self._offset_from_reference(rest_by_name, "spine_05", "clavicle_r", spine_positions["spine_05"])
        upperarm_l_start = self._offset_from_reference(rest_by_name, "clavicle_l", "upperarm_l", clavicle_l)
        upperarm_r_start = self._offset_from_reference(rest_by_name, "clavicle_r", "upperarm_r", clavicle_r)
        thigh_l_start = self._offset_from_reference(rest_by_name, "pelvis", "thigh_l", pelvis)
        thigh_r_start = self._offset_from_reference(rest_by_name, "pelvis", "thigh_r", pelvis)

        left_arm = self._solve_arm_chain(
            rest_by_name,
            controller_by_id,
            upperarm_l_start,
            wrist_l_target,
            "upperarm_l",
            "lowerarm_l",
            "hand_l",
            "elbow_l_secondary",
        )
        right_arm = self._solve_arm_chain(
            rest_by_name,
            controller_by_id,
            upperarm_r_start,
            wrist_r_target,
            "upperarm_r",
            "lowerarm_r",
            "hand_r",
            "elbow_r_secondary",
        )
        left_leg = self._solve_leg_chain(
            rest_by_name,
            controller_by_id,
            thigh_l_start,
            ankle_l_target,
            "thigh_l",
            "calf_l",
            "foot_l",
            "knee_l_secondary",
        )
        right_leg = self._solve_leg_chain(
            rest_by_name,
            controller_by_id,
            thigh_r_start,
            ankle_r_target,
            "thigh_r",
            "calf_r",
            "foot_r",
            "knee_r_secondary",
        )

        solved_positions = {
            **spine_positions,
            "clavicle_l": clavicle_l,
            "upperarm_l": left_arm.start_position,
            "lowerarm_l": left_arm.mid_position,
            "hand_l": left_arm.end_position,
            "clavicle_r": clavicle_r,
            "upperarm_r": right_arm.start_position,
            "lowerarm_r": right_arm.mid_position,
            "hand_r": right_arm.end_position,
            "thigh_l": left_leg.start_position,
            "calf_l": left_leg.mid_position,
            "foot_l": left_leg.end_position,
            "thigh_r": right_leg.start_position,
            "calf_r": right_leg.mid_position,
            "foot_r": right_leg.end_position,
        }

        proxy_rig = RigModel(
            name="AutoPosing ProxyRig",
            joints=[
                JointModel(
                    name=name,
                    parent_name=parent_name,
                    rest_world_position=self._rest_position(rest_by_name, name),
                    world_position=solved_positions.get(name, self._rest_position(rest_by_name, name)),
                )
                for name, parent_name in self._PROXY_RIG_LAYOUT
            ],
        )
        return proxy_rig, self._solve_controller_display_positions(rig, proxy_rig)

    def _solve_spine_chain(
        self,
        rest_by_name: dict[str, BoneModel],
        pelvis: tuple[float, float, float],
        chest: tuple[float, float, float],
        head: tuple[float, float, float],
    ) -> dict[str, tuple[float, float, float]]:
        head_delta = subtract(head, self._rest_position(rest_by_name, "head"))
        chest_delta = subtract(chest, self._rest_position(rest_by_name, "spine_05"))
        pelvis_delta = subtract(pelvis, self._rest_position(rest_by_name, "pelvis"))

        return {
            "pelvis": pelvis,
            "spine_01": add(lerp(pelvis, chest, 0.18), scale(head_delta, 0.02)),
            "spine_02": add(lerp(pelvis, chest, 0.38), scale(head_delta, 0.05)),
            "spine_03": add(lerp(pelvis, chest, 0.58), add(scale(head_delta, 0.08), scale(chest_delta, 0.04))),
            "spine_04": add(lerp(pelvis, chest, 0.8), scale(head_delta, 0.06)),
            "spine_05": chest,
            "neck_01": add(lerp(chest, head, 0.42), scale(pelvis_delta, -0.04)),
            "head": head,
        }

    def _solve_arm_chain(
        self,
        rest_by_name: dict[str, BoneModel],
        controller_by_id: dict[str, AutoPosingControllerModel],
        chain_start: tuple[float, float, float],
        end_target: tuple[float, float, float],
        start_name: str,
        mid_name: str,
        end_name: str,
        hint_controller_id: str,
    ) -> TwoBoneIkSolution:
        upper_length = self._rest_length(rest_by_name, start_name, mid_name)
        lower_length = self._rest_length(rest_by_name, mid_name, end_name)
        bend_hint = self._controller_target_position(
            controller_by_id,
            hint_controller_id,
            self._offset_from_reference(rest_by_name, start_name, mid_name, chain_start),
        )
        return solve_two_bone_ik(chain_start, end_target, upper_length, lower_length, bend_hint)

    def _solve_leg_chain(
        self,
        rest_by_name: dict[str, BoneModel],
        controller_by_id: dict[str, AutoPosingControllerModel],
        chain_start: tuple[float, float, float],
        end_target: tuple[float, float, float],
        start_name: str,
        mid_name: str,
        end_name: str,
        hint_controller_id: str,
    ) -> TwoBoneIkSolution:
        upper_length = self._rest_length(rest_by_name, start_name, mid_name)
        lower_length = self._rest_length(rest_by_name, mid_name, end_name)
        bend_hint = self._controller_target_position(
            controller_by_id,
            hint_controller_id,
            self._offset_from_reference(rest_by_name, start_name, mid_name, chain_start),
        )
        return solve_two_bone_ik(chain_start, end_target, upper_length, lower_length, bend_hint)

    def _resolve_arm_target(
        self,
        controller_by_id: dict[str, AutoPosingControllerModel],
        rest_by_name: dict[str, BoneModel],
        wrist_controller_id: str,
        elbow_controller_id: str,
        chest: tuple[float, float, float],
        hand_name: str,
        elbow_name: str,
    ) -> tuple[float, float, float]:
        wrist_fallback = self._offset_from_reference(rest_by_name, "spine_05", hand_name, chest)
        if self._controller_is_engaged(controller_by_id, elbow_controller_id) and not self._controller_is_engaged(
            controller_by_id, wrist_controller_id
        ):
            elbow_target = self._controller_target_position(
                controller_by_id,
                elbow_controller_id,
                self._rest_position(rest_by_name, elbow_name),
            )
            wrist_fallback = self._offset_from_reference(rest_by_name, elbow_name, hand_name, elbow_target)
        return self._controller_target_position(controller_by_id, wrist_controller_id, wrist_fallback)

    def _solve_controller_display_positions(
        self,
        rig: AutoPosingRigModel,
        proxy_rig: RigModel,
    ) -> dict[str, tuple[float, float, float]]:
        rig_by_name = {joint.name: joint for joint in proxy_rig.joints}
        positions: dict[str, tuple[float, float, float]] = {}

        for controller in rig.controllers:
            if self._controller_engaged(controller):
                positions[controller.identifier] = controller.world_position
                continue

            joint = rig_by_name.get(controller.joint_name)
            if joint is None:
                positions[controller.identifier] = controller.world_position
                continue

            if controller.controller_type == "direction":
                positions[controller.identifier] = add(
                    joint.world_position,
                    self._FORWARD_OFFSET.get(controller.identifier, (0.0, 0.0, 20.0)),
                )
                continue

            positions[controller.identifier] = joint.world_position

        return positions

    @staticmethod
    def _controller_by_id(rig: AutoPosingRigModel, controller_id: str) -> AutoPosingControllerModel | None:
        return next((controller for controller in rig.controllers if controller.identifier == controller_id), None)

    def _resolve_joint(self, by_name: dict[str, BoneModel], semantic_joint: str) -> BoneModel | None:
        for alias in self._JOINT_ALIASES.get(semantic_joint, (semantic_joint,)):
            joint = by_name.get(alias)
            if joint is not None:
                return joint
        return None

    def _controller_target_position(
        self,
        by_id: dict[str, AutoPosingControllerModel],
        controller_id: str,
        fallback: tuple[float, float, float],
    ) -> tuple[float, float, float]:
        controller = by_id.get(controller_id)
        if controller is None:
            return fallback
        return controller.world_position if self._controller_engaged(controller) else fallback

    @staticmethod
    def _controller_engaged(controller: AutoPosingControllerModel) -> bool:
        return controller.active or controller.fixed or controller.always_active

    def _controller_is_engaged(
        self,
        by_id: dict[str, AutoPosingControllerModel],
        controller_id: str,
    ) -> bool:
        controller = by_id.get(controller_id)
        return self._controller_engaged(controller) if controller is not None else False

    @staticmethod
    def _rest_position(by_name: dict[str, BoneModel], bone_name: str) -> tuple[float, float, float]:
        bone = by_name.get(bone_name)
        return bone.world_position if bone is not None else (0.0, 0.0, 0.0)

    def _offset_from_reference(
        self,
        by_name: dict[str, BoneModel],
        reference_name: str,
        target_name: str,
        reference_world_position: tuple[float, float, float],
    ) -> tuple[float, float, float]:
        reference = by_name.get(reference_name)
        target = by_name.get(target_name)
        if reference is None or target is None:
            return reference_world_position
        return add(reference_world_position, subtract(target.world_position, reference.world_position))

    def _rest_length(
        self,
        by_name: dict[str, BoneModel],
        start_name: str,
        end_name: str,
    ) -> float:
        return max(
            distance(self._rest_position(by_name, start_name), self._rest_position(by_name, end_name)),
            0.001,
        )

    def _average_delta(
        self,
        rest_points: tuple[tuple[float, float, float], ...],
        current_points: tuple[tuple[float, float, float], ...],
    ) -> tuple[float, float, float]:
        if not rest_points or len(rest_points) != len(current_points):
            return (0.0, 0.0, 0.0)

        delta = (0.0, 0.0, 0.0)
        for rest_point, current_point in zip(rest_points, current_points, strict=True):
            delta = add(delta, subtract(current_point, rest_point))
        return scale(delta, 1.0 / len(rest_points))
