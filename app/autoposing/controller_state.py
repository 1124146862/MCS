from core.autoposing.models import AutoPosingControllerModel, AutoPosingRigModel
from core.skeleton.models import Vec3


def controller_by_id(rig: AutoPosingRigModel, controller_id: str) -> AutoPosingControllerModel | None:
    return next((controller for controller in rig.controllers if controller.identifier == controller_id), None)


def set_mode(rig: AutoPosingRigModel, mode: str) -> None:
    rig.current_mode = mode
    if mode != "autoposing":
        select_controller(rig, None)


def select_controller(rig: AutoPosingRigModel, controller_id: str | None) -> None:
    rig.selected_controller_id = controller_id
    for controller in rig.controllers:
        controller.selected = controller.identifier == controller_id


def controller_engaged(controller: AutoPosingControllerModel) -> bool:
    return controller.active or controller.fixed or controller.always_active


def move_controller(rig: AutoPosingRigModel, controller_id: str, world_position: Vec3) -> bool:
    controller = controller_by_id(rig, controller_id)
    if controller is None:
        return False

    controller.target.world_position = world_position
    controller.active = True
    if controller.controller_type == "direction":
        controller.fixed = True
    select_controller(rig, controller_id)
    return True


def lock_controller(rig: AutoPosingRigModel, controller_id: str) -> bool:
    controller = controller_by_id(rig, controller_id)
    if controller is None or controller.always_active:
        return False

    controller.active = True
    select_controller(rig, controller_id)
    return True


def unlock_controller(rig: AutoPosingRigModel, controller_id: str) -> bool:
    controller = controller_by_id(rig, controller_id)
    if controller is None or controller.always_active:
        return False

    controller.active = False
    select_controller(rig, controller_id)
    return True


def fix_controller(
    rig: AutoPosingRigModel,
    controller_id: str,
    display_world_position: Vec3 | None = None,
) -> bool:
    controller = controller_by_id(rig, controller_id)
    if controller is None:
        return False

    was_engaged = controller_engaged(controller)
    controller.fixed = True
    if not was_engaged and display_world_position is not None:
        controller.target.world_position = display_world_position

    select_controller(rig, controller_id)
    return True


def unfix_controller(rig: AutoPosingRigModel, controller_id: str) -> bool:
    controller = controller_by_id(rig, controller_id)
    if controller is None:
        return False

    controller.fixed = False
    select_controller(rig, controller_id)
    return True


def toggle_fixed(
    rig: AutoPosingRigModel,
    controller_id: str,
    display_world_position: Vec3 | None = None,
) -> bool:
    controller = controller_by_id(rig, controller_id)
    if controller is None:
        return False
    if controller.fixed:
        unfix_controller(rig, controller_id)
        return False
    fix_controller(rig, controller_id, display_world_position)
    return True


def reset_controller(
    rig: AutoPosingRigModel,
    controller_id: str,
    reset_world_position: Vec3 | None = None,
) -> bool:
    controller = controller_by_id(rig, controller_id)
    if controller is None:
        return False

    reset_position = reset_world_position if reset_world_position is not None else controller.target.default_position
    controller.target.world_position = reset_position
    controller.target.solved_world_position = reset_position
    controller.target.clamped_world_position = reset_position
    controller.target.pressure = 0.0
    controller.target.pressure_state = "normal"
    controller.target.clamped = False
    controller.active = controller.always_active
    controller.fixed = False
    select_controller(rig, controller_id)
    return True


def controller_state(controller: AutoPosingControllerModel) -> str:
    if controller.fixed:
        return "fixed"
    if controller.active or controller.always_active:
        return "active"
    return "inactive"
