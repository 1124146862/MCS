from __future__ import annotations

from core.autoposing.models import AutoPosingRigModel, SolvedPoseModel
from core.math import add, length, normalize, scale, subtract
from core.skeleton.models import Vec3

from .visual_policy import DIRECTION_CONTROLLER_LENGTHS
from .finger_profile import FINGER_ROLES, is_finger_controller_id


DEFAULT_DIRECTION_OFFSETS: dict[str, Vec3] = {
    "pelvis_dir": (0.0, 0.0, 20.0),
    "chest_dir": (0.0, 6.0, 30.0),
    "head_dir": (0.0, 0.0, 20.0),
}


def direction_rod_length(controller_id: str) -> float:
    return DIRECTION_CONTROLLER_LENGTHS.get(controller_id, 20.0)


def default_direction(controller_id: str) -> Vec3:
    fallback = normalize(DEFAULT_DIRECTION_OFFSETS.get(controller_id, (0.0, 0.0, 20.0)))
    return fallback if length(fallback) > 0.0 else (0.0, 0.0, 1.0)


def direction_endpoint(controller_id: str, base: Vec3, target: Vec3) -> Vec3:
    direction = normalize(subtract(target, base))
    if length(direction) <= 0.0:
        direction = default_direction(controller_id)
    return add(base, scale(direction, direction_rod_length(controller_id)))


def sync_inactive_controller_targets(rig: AutoPosingRigModel, solved_pose: SolvedPoseModel) -> None:
    joints_by_name = {joint.name: joint for joint in solved_pose.rig.joints}
    effectors_by_id = {effector.controller_id: effector for effector in solved_pose.effectors}

    for controller in rig.controllers:
        if controller.fixed or controller.active or controller.always_active:
            continue

        effector = effectors_by_id.get(controller.identifier)
        if effector is None:
            continue

        if controller.controller_type == "direction":
            joint = joints_by_name.get(controller.joint_name)
            base = joint.world_position if joint is not None else effector.solved_world_position
            next_target = direction_endpoint(controller.identifier, base, controller.target.world_position)
        elif controller.role in FINGER_ROLES or is_finger_controller_id(controller.identifier):
            next_target = effector.solved_world_position
        else:
            joint = joints_by_name.get(controller.joint_name)
            if joint is None:
                next_target = effector.solved_world_position
            else:
                offset = subtract(controller.target.default_position, joint.rest_world_position)
                next_target = add(joint.world_position, offset)

        controller.target.world_position = next_target
        controller.target.solved_world_position = next_target
        controller.target.clamped_world_position = next_target
        controller.target.pressure = 0.0
        controller.target.pressure_state = "normal"
        controller.target.clamped = False
