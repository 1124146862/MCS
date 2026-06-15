from __future__ import annotations

from core.autoposing.models import AutoPosingControllerModel

from .finger_profile import FINGER_ROLES, is_finger_controller_id


LOCKED_COLOR = "#2a98ff"
INACTIVE_COLOR = "#3fae56"
FIXED_COLOR = "#d94c45"
SELECTED_BORDER_COLOR = "#fff5bf"
CLAMPED_BORDER_COLOR = "#d94c45"
DEFAULT_BORDER_COLOR = "#0f2718"
GHOST_BORDER_COLOR = "#b9f5c4"

DIRECTION_CONTROLLER_LENGTHS = {
    "pelvis_dir": 20.0,
    "chest_dir": 30.0,
    "head_dir": 20.0,
}


def controller_role(controller: AutoPosingControllerModel) -> str:
    return controller.role or controller.controller_type


def is_direction_controller(controller: AutoPosingControllerModel) -> bool:
    return controller_role(controller) == "direction"


def is_locked(controller: AutoPosingControllerModel) -> bool:
    return controller.active or controller.always_active


def menu_kind(controller: AutoPosingControllerModel) -> str:
    if is_direction_controller(controller):
        return "direction"
    if controller.role in FINGER_ROLES or is_finger_controller_id(controller.identifier):
        return "finger"
    if controller.always_active:
        return "alwaysActive"
    return "controller"


def controller_shape(controller: AutoPosingControllerModel) -> str:
    return "square" if is_direction_controller(controller) else "circle"


def radius_role(controller: AutoPosingControllerModel) -> str:
    role = controller_role(controller)
    if role == "direction":
        return "direction"
    if role == "main":
        return "main"
    if role in FINGER_ROLES:
        return "finger"
    if role == "lesser":
        return "lesser"
    return "secondary"


def can_lock(controller: AutoPosingControllerModel) -> bool:
    return not controller.always_active and not is_locked(controller)


def can_unlock(controller: AutoPosingControllerModel) -> bool:
    return not controller.always_active and controller.active


def can_fix(controller: AutoPosingControllerModel) -> bool:
    return not controller.fixed


def can_unfix(controller: AutoPosingControllerModel) -> bool:
    return controller.fixed


def visual_color(controller: AutoPosingControllerModel) -> str:
    if controller.fixed:
        return FIXED_COLOR
    if is_locked(controller):
        return LOCKED_COLOR
    return INACTIVE_COLOR


def border_color(controller: AutoPosingControllerModel, clamped: bool = False) -> str:
    if controller.selected:
        return SELECTED_BORDER_COLOR
    if clamped:
        return CLAMPED_BORDER_COLOR
    if controller.fixed:
        return FIXED_COLOR
    return DEFAULT_BORDER_COLOR


def pressure_color(state: str) -> str:
    if state == "stretch":
        return "#d94c45"
    if state == "squeeze":
        return "#2a98ff"
    return "#1b6f3a"


def show_controller_in_normal(controller: AutoPosingControllerModel) -> bool:
    return controller.visible


def show_pressure_tether(controller: AutoPosingControllerModel, preview: dict[str, object], debug_visible: bool) -> bool:
    if is_direction_controller(controller):
        return False
    if debug_visible:
        return bool(preview.get("showTether"))
    if not bool(preview.get("showTether")):
        return False
    if controller.selected or controller.fixed:
        return True
    if controller.active and not controller.always_active:
        return True
    return bool(controller.always_active and (bool(preview.get("clampedFlag")) or preview.get("pressureState") != "normal"))


def show_target_ghost(controller: AutoPosingControllerModel, preview: dict[str, object], debug_visible: bool) -> bool:
    if is_direction_controller(controller):
        return False
    if debug_visible:
        return bool(preview.get("showGhostTarget"))
    return bool(preview.get("showGhostTarget")) and (controller.active or controller.fixed or bool(preview.get("clampedFlag")))


def controller_fields(controller: AutoPosingControllerModel, clamped: bool = False) -> dict[str, str | bool]:
    locked = is_locked(controller)
    return {
        "locked": locked,
        "canLock": can_lock(controller),
        "canUnlock": can_unlock(controller),
        "canFix": can_fix(controller),
        "canUnfix": can_unfix(controller),
        "isDirection": is_direction_controller(controller),
        "menuKind": menu_kind(controller),
        "visualColor": visual_color(controller),
        "borderColor": border_color(controller, clamped),
        "shape": controller_shape(controller),
        "radiusRole": radius_role(controller),
        "role": controller_role(controller),
    }
