from __future__ import annotations

from dataclasses import dataclass, field

from core.autoposing.models import AutoPosingControllerModel, AutoPosingRigModel
from core.skeleton.models import Vec3

from .anatomy_profile import HumanoidAnatomyProfile


@dataclass(frozen=True, slots=True)
class PositionConstraint:
    controller_id: str
    joint_name: str
    target_world_position: Vec3
    strength: float
    locked: bool
    priority: int
    controller_type: str


@dataclass(frozen=True, slots=True)
class DirectionConstraint:
    controller_id: str
    joint_name: str
    target_world_position: Vec3
    strength: float
    locked: bool
    priority: int


@dataclass(frozen=True, slots=True)
class SupportConstraint:
    controller_id: str
    joint_name: str
    target_world_position: Vec3
    floor_y: float
    strength: float
    locked: bool
    priority: int


@dataclass(slots=True)
class PoseConstraintSet:
    positions: dict[str, PositionConstraint] = field(default_factory=dict)
    directions: dict[str, DirectionConstraint] = field(default_factory=dict)
    supports: dict[str, SupportConstraint] = field(default_factory=dict)
    by_controller_id: dict[str, PositionConstraint | DirectionConstraint | SupportConstraint] = field(default_factory=dict)

    def position_for(self, controller_id: str, fallback: Vec3) -> Vec3:
        constraint = self.positions.get(controller_id)
        return constraint.target_world_position if constraint is not None else fallback

    def direction_for(self, controller_id: str) -> DirectionConstraint | None:
        return self.directions.get(controller_id)

    def locked(self, controller_id: str) -> bool:
        constraint = self.by_controller_id.get(controller_id)
        return bool(constraint.locked) if constraint is not None else False

    def engaged(self, controller_id: str) -> bool:
        return controller_id in self.by_controller_id


def extract_pose_constraints(rig: AutoPosingRigModel, anatomy: HumanoidAnatomyProfile) -> PoseConstraintSet:
    constraints = PoseConstraintSet()
    for controller in rig.controllers:
        if not _controller_engaged(controller):
            continue

        strength = _constraint_strength(controller)
        locked = controller.fixed or controller.selected
        priority = _constraint_priority(controller)
        if controller.controller_type == "direction":
            direction = DirectionConstraint(
                controller_id=controller.identifier,
                joint_name=controller.joint_name,
                target_world_position=controller.target.world_position,
                strength=strength,
                locked=locked,
                priority=priority,
            )
            constraints.directions[controller.identifier] = direction
            constraints.by_controller_id[controller.identifier] = direction
            continue

        position = PositionConstraint(
            controller_id=controller.identifier,
            joint_name=controller.joint_name,
            target_world_position=controller.target.world_position,
            strength=strength,
            locked=locked,
            priority=priority,
            controller_type=controller.controller_type,
        )
        constraints.positions[controller.identifier] = position
        constraints.by_controller_id[controller.identifier] = position

        if controller.joint_name in {"foot_l", "foot_r", "ball_l", "ball_r"}:
            support = SupportConstraint(
                controller_id=controller.identifier,
                joint_name=controller.joint_name,
                target_world_position=controller.target.world_position,
                floor_y=anatomy.floor_y,
                strength=strength,
                locked=controller.fixed or controller.always_active,
                priority=priority,
            )
            constraints.supports[controller.identifier] = support
    return constraints


def _controller_engaged(controller: AutoPosingControllerModel) -> bool:
    return controller.active or controller.fixed or controller.always_active


def _constraint_strength(controller: AutoPosingControllerModel) -> float:
    if controller.fixed:
        return 1.0
    if controller.always_active:
        return 0.92
    if controller.selected:
        return 0.88
    return 0.78 if controller.active else 0.0


def _constraint_priority(controller: AutoPosingControllerModel) -> int:
    if controller.fixed:
        return 100
    if controller.always_active:
        return 85
    if controller.selected:
        return 75
    return 60
