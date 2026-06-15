from __future__ import annotations

from dataclasses import dataclass

from core.math import add, distance, length, normalize, scale, subtract
from core.skeleton.models import BoneModel, SkeletonModel, Vec3


FINGER_NAMES = ("thumb", "index", "middle", "ring", "pinky")
FINGER_ROLES = {"finger_tip", "finger_pre_tip"}


@dataclass(frozen=True, slots=True)
class FingerChainProfile:
    finger_name: str
    side: str
    hand_name: str
    joint_names: tuple[str, ...]
    tip_controller_id: str
    pre_tip_controller_id: str
    tip_world_position: Vec3
    pre_tip_world_position: Vec3
    opposable: bool = False
    opposition_axis: Vec3 = (0.0, 0.0, 0.0)

    @property
    def key(self) -> str:
        return f"{self.finger_name}_{self.side}"

    @property
    def root_name(self) -> str:
        return self.joint_names[0]

    @property
    def distal_name(self) -> str:
        return self.joint_names[-1]

    @property
    def pre_tip_name(self) -> str:
        return self.joint_names[-2] if len(self.joint_names) > 1 else self.joint_names[-1]


@dataclass(frozen=True, slots=True)
class FingerAnatomyProfile:
    rest_by_name: dict[str, BoneModel]
    chains: dict[str, FingerChainProfile]

    def has_chains(self) -> bool:
        return bool(self.chains)

    def chain_for_controller(self, controller_id: str) -> FingerChainProfile | None:
        for chain in self.chains.values():
            if controller_id in {chain.tip_controller_id, chain.pre_tip_controller_id}:
                return chain
        return None

    def rest(self, bone_name: str) -> Vec3:
        bone = self.rest_by_name.get(bone_name)
        return bone.rest_world_position if bone is not None else (0.0, 0.0, 0.0)


def build_finger_anatomy_profile(skeleton: SkeletonModel) -> FingerAnatomyProfile:
    rest_by_name = {bone.name: bone for bone in skeleton.bones}
    chains: dict[str, FingerChainProfile] = {}
    for side in ("l", "r"):
        hand_name = f"hand_{side}"
        if hand_name not in rest_by_name:
            continue
        for finger_name in FINGER_NAMES:
            joint_names = _finger_joint_names(rest_by_name, finger_name, side)
            if len(joint_names) < 2:
                continue
            tip_position = _tip_position(rest_by_name, joint_names)
            pre_tip_position = _pre_tip_position(rest_by_name, hand_name, joint_names, side)
            chain = FingerChainProfile(
                finger_name=finger_name,
                side=side,
                hand_name=hand_name,
                joint_names=joint_names,
                tip_controller_id=f"{finger_name}_{side}_tip",
                pre_tip_controller_id=f"{finger_name}_{side}_pre_tip",
                tip_world_position=tip_position,
                pre_tip_world_position=pre_tip_position,
                opposable=finger_name == "thumb",
                opposition_axis=_opposition_axis(rest_by_name, hand_name, joint_names, side),
            )
            chains[chain.key] = chain
    return FingerAnatomyProfile(rest_by_name=rest_by_name, chains=chains)


def is_finger_controller_id(controller_id: str) -> bool:
    parts = controller_id.split("_")
    return (
        len(parts) >= 3
        and parts[0] in FINGER_NAMES
        and parts[1] in {"l", "r"}
        and "_".join(parts[2:]) in {"tip", "pre_tip"}
    )


def finger_controller_parts(controller_id: str) -> tuple[str, str, str] | None:
    if not is_finger_controller_id(controller_id):
        return None
    parts = controller_id.split("_")
    return parts[0], parts[1], "_".join(parts[2:])


def _finger_joint_names(rest_by_name: dict[str, BoneModel], finger_name: str, side: str) -> tuple[str, ...]:
    numbered = tuple(
        name
        for name in (
            f"{finger_name}_01_{side}",
            f"{finger_name}_02_{side}",
            f"{finger_name}_03_{side}",
        )
        if name in rest_by_name
    )
    if len(numbered) >= 2:
        return numbered

    semantic = tuple(
        name
        for name in (
            f"{finger_name}_proximal_{side}",
            f"{finger_name}_intermediate_{side}",
            f"{finger_name}_distal_{side}",
        )
        if name in rest_by_name
    )
    if len(semantic) >= 2:
        return semantic

    metacarpal = f"{finger_name}_metacarpal_{side}"
    if metacarpal in rest_by_name and numbered:
        return (metacarpal, *numbered)
    return ()


def _tip_position(rest_by_name: dict[str, BoneModel], joint_names: tuple[str, ...]) -> Vec3:
    distal = rest_by_name[joint_names[-1]].rest_world_position
    previous = rest_by_name[joint_names[-2]].rest_world_position
    direction = normalize(subtract(distal, previous))
    segment_length = distance(previous, distal)
    if length(direction) <= 0.0:
        return distal
    return add(distal, scale(direction, max(segment_length * 0.62, 1.0)))


def _pre_tip_position(rest_by_name: dict[str, BoneModel], hand_name: str, joint_names: tuple[str, ...], side: str) -> Vec3:
    distal = rest_by_name[joint_names[-1]].rest_world_position
    previous = rest_by_name[joint_names[-2]].rest_world_position
    hand = rest_by_name[hand_name].rest_world_position
    along_finger = normalize(subtract(distal, previous))
    palm_out = normalize(subtract(distal, hand))
    if length(palm_out) <= 0.0:
        palm_out = (1.0 if side == "l" else -1.0, 0.0, 0.0)
    return add(add(previous, scale(subtract(distal, previous), 0.55)), scale(palm_out, 2.0 if length(along_finger) > 0.0 else 1.0))


def _opposition_axis(rest_by_name: dict[str, BoneModel], hand_name: str, joint_names: tuple[str, ...], side: str) -> Vec3:
    hand = rest_by_name[hand_name].rest_world_position
    root = rest_by_name[joint_names[0]].rest_world_position
    distal = rest_by_name[joint_names[-1]].rest_world_position
    axis = normalize(subtract(root, hand))
    if length(axis) <= 0.0:
        axis = normalize(subtract(distal, hand))
    if length(axis) <= 0.0:
        axis = (1.0 if side == "l" else -1.0, 0.0, 0.0)
    return axis
