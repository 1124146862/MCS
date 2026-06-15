from __future__ import annotations

from dataclasses import dataclass, field

from core.autoposing.models import AutoPosingControllerModel, AutoPosingRigModel
from core.math import distance
from core.skeleton.models import Vec3

from .anatomy_profile import HumanoidAnatomyProfile
from .finger_profile import finger_controller_parts, is_finger_controller_id


@dataclass(frozen=True, slots=True)
class ConstraintInfluence:
    preservation_weight: float = 0.0
    pose_intent_weight: float = 0.0
    support_weight: float = 0.0
    orientation_weight: float = 0.0
    fixed: bool = False
    always_active: bool = False


@dataclass(frozen=True, slots=True)
class PositionConstraint:
    controller_id: str
    joint_name: str
    target_world_position: Vec3
    strength: float
    locked: bool
    priority: int
    controller_type: str
    preservation_weight: float = 0.0
    pose_intent_weight: float = 0.0
    support_weight: float = 0.0
    orientation_weight: float = 0.0
    fixed: bool = False
    always_active: bool = False


@dataclass(frozen=True, slots=True)
class DirectionConstraint:
    controller_id: str
    joint_name: str
    target_world_position: Vec3
    strength: float
    locked: bool
    priority: int
    preservation_weight: float = 0.0
    pose_intent_weight: float = 0.0
    support_weight: float = 0.0
    orientation_weight: float = 0.0
    fixed: bool = False
    always_active: bool = False


@dataclass(frozen=True, slots=True)
class SupportConstraint:
    controller_id: str
    joint_name: str
    target_world_position: Vec3
    floor_y: float
    strength: float
    locked: bool
    priority: int
    preservation_weight: float = 0.0
    pose_intent_weight: float = 0.0
    support_weight: float = 0.0
    orientation_weight: float = 0.0
    fixed: bool = False
    always_active: bool = False


@dataclass(frozen=True, slots=True)
class BendHintConstraint:
    controller_id: str
    joint_name: str
    target_world_position: Vec3
    strength: float
    locked: bool
    priority: int
    chain_role: str
    preservation_weight: float = 0.0
    pose_intent_weight: float = 0.0
    support_weight: float = 0.0
    orientation_weight: float = 0.0
    fixed: bool = False
    always_active: bool = False


@dataclass(frozen=True, slots=True)
class OrientationHintConstraint:
    controller_id: str
    joint_name: str
    target_world_position: Vec3
    strength: float
    locked: bool
    priority: int
    orientation_role: str
    preservation_weight: float = 0.0
    pose_intent_weight: float = 0.0
    support_weight: float = 0.0
    orientation_weight: float = 0.0
    fixed: bool = False
    always_active: bool = False


@dataclass(frozen=True, slots=True)
class FootContactConstraint:
    controller_id: str
    joint_name: str
    target_world_position: Vec3
    floor_y: float
    strength: float
    locked: bool
    priority: int
    contact_role: str
    preservation_weight: float = 0.0
    pose_intent_weight: float = 0.0
    support_weight: float = 0.0
    orientation_weight: float = 0.0
    fixed: bool = False
    always_active: bool = False


@dataclass(frozen=True, slots=True)
class FingerTipConstraint:
    controller_id: str
    joint_name: str
    target_world_position: Vec3
    strength: float
    locked: bool
    priority: int
    finger_name: str
    side: str
    preservation_weight: float = 0.0
    pose_intent_weight: float = 0.0
    support_weight: float = 0.0
    orientation_weight: float = 0.0
    fixed: bool = False
    always_active: bool = False


@dataclass(frozen=True, slots=True)
class FingerBendHintConstraint:
    controller_id: str
    joint_name: str
    target_world_position: Vec3
    strength: float
    locked: bool
    priority: int
    finger_name: str
    side: str
    preservation_weight: float = 0.0
    pose_intent_weight: float = 0.0
    support_weight: float = 0.0
    orientation_weight: float = 0.0
    fixed: bool = False
    always_active: bool = False


@dataclass(frozen=True, slots=True)
class PalmSpreadHintConstraint:
    controller_id: str
    joint_name: str
    target_world_position: Vec3
    strength: float
    locked: bool
    priority: int
    finger_name: str
    side: str
    preservation_weight: float = 0.0
    pose_intent_weight: float = 0.0
    support_weight: float = 0.0
    orientation_weight: float = 0.0
    fixed: bool = False
    always_active: bool = False


@dataclass(slots=True)
class PoseConstraintSet:
    positions: dict[str, PositionConstraint] = field(default_factory=dict)
    directions: dict[str, DirectionConstraint] = field(default_factory=dict)
    supports: dict[str, SupportConstraint] = field(default_factory=dict)
    bend_hints: dict[str, BendHintConstraint] = field(default_factory=dict)
    orientation_hints: dict[str, OrientationHintConstraint] = field(default_factory=dict)
    foot_contacts: dict[str, FootContactConstraint] = field(default_factory=dict)
    finger_tips: dict[str, FingerTipConstraint] = field(default_factory=dict)
    finger_bend_hints: dict[str, FingerBendHintConstraint] = field(default_factory=dict)
    palm_spread_hints: dict[str, PalmSpreadHintConstraint] = field(default_factory=dict)
    by_controller_id: dict[
        str,
        PositionConstraint
        | DirectionConstraint
        | SupportConstraint
        | BendHintConstraint
        | OrientationHintConstraint
        | FootContactConstraint
        | FingerTipConstraint
        | FingerBendHintConstraint
        | PalmSpreadHintConstraint,
    ] = field(default_factory=dict)

    def position_for(self, controller_id: str, fallback: Vec3) -> Vec3:
        constraint = self.positions.get(controller_id)
        return constraint.target_world_position if constraint is not None else fallback

    def direction_for(self, controller_id: str) -> DirectionConstraint | None:
        return self.directions.get(controller_id)

    def bend_hint_for(self, controller_id: str, fallback: Vec3) -> Vec3:
        constraint = self.bend_hints.get(controller_id)
        return constraint.target_world_position if constraint is not None else self.position_for(controller_id, fallback)

    def orientation_for(self, *controller_ids: str) -> OrientationHintConstraint | None:
        for controller_id in controller_ids:
            constraint = self.orientation_hints.get(controller_id)
            if constraint is not None:
                return constraint
        return None

    def locked(self, controller_id: str) -> bool:
        constraint = self.by_controller_id.get(controller_id)
        return bool(constraint.locked) if constraint is not None else False

    def fixed(self, controller_id: str) -> bool:
        constraint = self.by_controller_id.get(controller_id)
        return bool(getattr(constraint, "fixed", False)) if constraint is not None else False

    def pose_intent_weight(self, controller_id: str) -> float:
        constraint = self.by_controller_id.get(controller_id)
        return float(getattr(constraint, "pose_intent_weight", 0.0)) if constraint is not None else 0.0

    def preservation_weight(self, controller_id: str) -> float:
        constraint = self.by_controller_id.get(controller_id)
        return float(getattr(constraint, "preservation_weight", 0.0)) if constraint is not None else 0.0

    def support_weight(self, controller_id: str) -> float:
        constraint = self.by_controller_id.get(controller_id)
        return float(getattr(constraint, "support_weight", 0.0)) if constraint is not None else 0.0

    def orientation_weight(self, controller_id: str) -> float:
        constraint = self.by_controller_id.get(controller_id)
        return float(getattr(constraint, "orientation_weight", 0.0)) if constraint is not None else 0.0

    def engaged(self, controller_id: str) -> bool:
        return controller_id in self.by_controller_id

    def finger_tip_for(self, controller_id: str) -> FingerTipConstraint | None:
        return self.finger_tips.get(controller_id)

    def finger_bend_hint_for(self, controller_id: str) -> FingerBendHintConstraint | None:
        return self.finger_bend_hints.get(controller_id)


def extract_pose_constraints(rig: AutoPosingRigModel, anatomy: HumanoidAnatomyProfile) -> PoseConstraintSet:
    constraints = PoseConstraintSet()
    for controller in rig.controllers:
        if is_finger_controller_id(controller.identifier) and not rig.fingers_enabled:
            continue
        if not _controller_engaged(controller):
            continue

        strength = _constraint_strength(controller)
        influence = _constraint_influence(controller)
        locked = controller.active or controller.fixed or controller.always_active
        priority = _constraint_priority(controller)
        if controller.controller_type == "direction":
            direction = DirectionConstraint(
                controller_id=controller.identifier,
                joint_name=controller.joint_name,
                target_world_position=controller.target.world_position,
                strength=strength,
                locked=locked,
                priority=priority,
                preservation_weight=influence.preservation_weight,
                pose_intent_weight=influence.pose_intent_weight,
                support_weight=influence.support_weight,
                orientation_weight=influence.orientation_weight,
                fixed=influence.fixed,
                always_active=influence.always_active,
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
            preservation_weight=influence.preservation_weight,
            pose_intent_weight=influence.pose_intent_weight,
            support_weight=influence.support_weight,
            orientation_weight=influence.orientation_weight,
            fixed=influence.fixed,
            always_active=influence.always_active,
        )
        constraints.positions[controller.identifier] = position
        constraints.by_controller_id[controller.identifier] = position

        finger_parts = finger_controller_parts(controller.identifier)
        if finger_parts is not None:
            finger_name, side, finger_role = finger_parts
            if finger_role == "tip":
                finger_tip = FingerTipConstraint(
                    controller_id=controller.identifier,
                    joint_name=controller.joint_name,
                    target_world_position=controller.target.world_position,
                    strength=strength,
                    locked=locked,
                    priority=priority,
                    finger_name=finger_name,
                    side=side,
                    preservation_weight=influence.preservation_weight,
                    pose_intent_weight=influence.pose_intent_weight,
                    support_weight=influence.support_weight,
                    orientation_weight=influence.orientation_weight,
                    fixed=influence.fixed,
                    always_active=influence.always_active,
                )
                constraints.finger_tips[controller.identifier] = finger_tip
                constraints.by_controller_id[controller.identifier] = finger_tip
                if finger_name in {"index", "pinky"}:
                    constraints.palm_spread_hints[controller.identifier] = PalmSpreadHintConstraint(
                        controller_id=controller.identifier,
                        joint_name=controller.joint_name,
                        target_world_position=controller.target.world_position,
                        strength=strength,
                        locked=locked,
                        priority=priority,
                        finger_name=finger_name,
                        side=side,
                        preservation_weight=influence.preservation_weight,
                        pose_intent_weight=influence.pose_intent_weight,
                        support_weight=influence.support_weight,
                        orientation_weight=influence.orientation_weight,
                        fixed=influence.fixed,
                        always_active=influence.always_active,
                    )
            elif finger_role == "pre_tip":
                finger_bend = FingerBendHintConstraint(
                    controller_id=controller.identifier,
                    joint_name=controller.joint_name,
                    target_world_position=controller.target.world_position,
                    strength=strength,
                    locked=locked,
                    priority=priority,
                    finger_name=finger_name,
                    side=side,
                    preservation_weight=influence.preservation_weight,
                    pose_intent_weight=influence.pose_intent_weight,
                    support_weight=influence.support_weight,
                    orientation_weight=influence.orientation_weight,
                    fixed=influence.fixed,
                    always_active=influence.always_active,
                )
                constraints.finger_bend_hints[controller.identifier] = finger_bend
                constraints.by_controller_id[controller.identifier] = finger_bend
            continue

        bend_role = _bend_hint_role(controller.identifier)
        if bend_role:
            constraints.bend_hints[controller.identifier] = BendHintConstraint(
                controller_id=controller.identifier,
                joint_name=controller.joint_name,
                target_world_position=controller.target.world_position,
                strength=strength,
                locked=locked,
                priority=priority,
                chain_role=bend_role,
                preservation_weight=influence.preservation_weight,
                pose_intent_weight=influence.pose_intent_weight,
                support_weight=influence.support_weight,
                orientation_weight=influence.orientation_weight,
                fixed=influence.fixed,
                always_active=influence.always_active,
            )

        orientation_role = _orientation_hint_role(controller.identifier)
        if orientation_role:
            constraints.orientation_hints[controller.identifier] = OrientationHintConstraint(
                controller_id=controller.identifier,
                joint_name=controller.joint_name,
                target_world_position=controller.target.world_position,
                strength=strength,
                locked=locked,
                priority=priority,
                orientation_role=orientation_role,
                preservation_weight=influence.preservation_weight,
                pose_intent_weight=influence.pose_intent_weight,
                support_weight=influence.support_weight,
                orientation_weight=influence.orientation_weight,
                fixed=influence.fixed,
                always_active=influence.always_active,
            )

        if controller.joint_name in {"foot_l", "foot_r", "ball_l", "ball_r"}:
            support = SupportConstraint(
                controller_id=controller.identifier,
                joint_name=controller.joint_name,
                target_world_position=controller.target.world_position,
                floor_y=anatomy.floor_y,
                strength=strength,
                locked=controller.fixed or controller.always_active,
                priority=priority,
                preservation_weight=influence.preservation_weight,
                pose_intent_weight=influence.pose_intent_weight,
                support_weight=influence.support_weight,
                orientation_weight=influence.orientation_weight,
                fixed=influence.fixed,
                always_active=influence.always_active,
            )
            constraints.supports[controller.identifier] = support

        foot_contact_role = _foot_contact_role(controller.identifier)
        if foot_contact_role:
            constraints.foot_contacts[controller.identifier] = FootContactConstraint(
                controller_id=controller.identifier,
                joint_name=controller.joint_name,
                target_world_position=controller.target.world_position,
                floor_y=anatomy.floor_y,
                strength=strength,
                locked=locked,
                priority=priority,
                contact_role=foot_contact_role,
                preservation_weight=influence.preservation_weight,
                pose_intent_weight=influence.pose_intent_weight,
                support_weight=influence.support_weight,
                orientation_weight=influence.orientation_weight,
                fixed=influence.fixed,
                always_active=influence.always_active,
            )
    return constraints


def _bend_hint_role(controller_id: str) -> str:
    if controller_id.startswith("elbow_"):
        return "arm"
    if controller_id.startswith("knee_"):
        return "leg"
    return ""


def _orientation_hint_role(controller_id: str) -> str:
    if controller_id.startswith("hand_"):
        return "palm"
    if controller_id.startswith("foot_") or controller_id.startswith("ball_"):
        return "foot"
    if controller_id in {"pelvis_additional", "chest_additional", "head_additional"}:
        return "body"
    return ""


def _foot_contact_role(controller_id: str) -> str:
    if controller_id.startswith("foot_"):
        return "foot"
    if controller_id.startswith("ball_"):
        return "ball"
    return ""


def _controller_engaged(controller: AutoPosingControllerModel) -> bool:
    return controller.active or controller.fixed or controller.always_active


def _constraint_influence(controller: AutoPosingControllerModel) -> ConstraintInfluence:
    strength = _constraint_strength(controller)
    fixed = controller.fixed
    always_active = controller.always_active
    is_support = controller.joint_name in {"foot_l", "foot_r", "ball_l", "ball_r"}
    is_middle_chain = controller.identifier.startswith(("elbow_", "knee_"))
    target_moved = distance(controller.target.world_position, controller.target.default_position) > 0.001
    is_moving_always_active = always_active and (controller.selected or target_moved)

    if fixed:
        pose_intent = 0.0 if is_middle_chain else (0.10 if controller.controller_type == "main" else 0.0)
        return ConstraintInfluence(
            preservation_weight=1.0,
            pose_intent_weight=pose_intent,
            support_weight=1.0 if is_support else 0.0,
            orientation_weight=1.0 if _orientation_hint_role(controller.identifier) else pose_intent,
            fixed=True,
            always_active=always_active,
        )

    if is_moving_always_active:
        return ConstraintInfluence(
            preservation_weight=0.95,
            pose_intent_weight=0.80 * strength,
            support_weight=0.20 if is_support else 0.0,
            orientation_weight=0.80 * strength,
            fixed=False,
            always_active=True,
        )

    if always_active:
        return ConstraintInfluence(
            preservation_weight=0.85,
            pose_intent_weight=0.10,
            support_weight=1.0 if is_support else 0.0,
            orientation_weight=0.10,
            fixed=False,
            always_active=True,
        )

    if controller.selected:
        return ConstraintInfluence(
            preservation_weight=0.95,
            pose_intent_weight=1.0,
            support_weight=0.20 if is_support else 0.0,
            orientation_weight=1.0,
        )

    if controller.active:
        return ConstraintInfluence(
            preservation_weight=0.90,
            pose_intent_weight=1.0,
            support_weight=0.20 if is_support else 0.0,
            orientation_weight=1.0,
        )

    return ConstraintInfluence()


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
