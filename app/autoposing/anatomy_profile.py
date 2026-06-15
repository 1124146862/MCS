from __future__ import annotations

from dataclasses import dataclass

from core.math import distance
from core.skeleton.models import BoneModel, SkeletonModel, Vec3


PRIMARY_LAYOUT = (
    ("pelvis", None),
    ("spine_01", "pelvis"),
    ("spine_02", "spine_01"),
    ("spine_03", "spine_02"),
    ("spine_04", "spine_03"),
    ("spine_05", "spine_04"),
    ("neck_01", "spine_05"),
    ("neck_02", "neck_01"),
    ("head", "neck_02"),
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
    ("ball_l", "foot_l"),
    ("thigh_r", "pelvis"),
    ("calf_r", "thigh_r"),
    ("foot_r", "calf_r"),
    ("ball_r", "foot_r"),
)

FORWARD_OFFSET = {
    "pelvis_dir": (0.0, 0.0, 24.0),
    "chest_dir": (0.0, 6.0, 26.0),
    "head_dir": (0.0, 0.0, 22.0),
}


@dataclass(frozen=True, slots=True)
class LimbChainProfile:
    name: str
    side: str
    start_name: str
    mid_name: str
    end_name: str
    end_controller_id: str
    hint_controller_id: str


@dataclass(frozen=True, slots=True)
class HumanoidAnatomyProfile:
    rest_by_name: dict[str, BoneModel]
    primary_layout: tuple[tuple[str, str | None], ...]
    spine_chain: tuple[str, ...]
    neck_chain: tuple[str, ...]
    limb_chains: dict[str, LimbChainProfile]
    support_joint_names: tuple[str, ...]
    direction_offsets: dict[str, Vec3]
    floor_y: float

    def has(self, bone_name: str) -> bool:
        return bone_name in self.rest_by_name

    def rest(self, bone_name: str) -> Vec3:
        bone = self.rest_by_name.get(bone_name)
        return bone.rest_world_position if bone is not None else (0.0, 0.0, 0.0)

    def offset(self, reference_name: str, target_name: str, reference_world_position: Vec3) -> Vec3:
        reference = self.rest_by_name.get(reference_name)
        target = self.rest_by_name.get(target_name)
        if reference is None or target is None:
            return reference_world_position
        return (
            reference_world_position[0] + target.rest_world_position[0] - reference.rest_world_position[0],
            reference_world_position[1] + target.rest_world_position[1] - reference.rest_world_position[1],
            reference_world_position[2] + target.rest_world_position[2] - reference.rest_world_position[2],
        )

    def length(self, start_name: str, end_name: str) -> float:
        return max(distance(self.rest(start_name), self.rest(end_name)), 0.001)

    def chain_available(self, chain: tuple[str, ...]) -> bool:
        return all(name in self.rest_by_name for name in chain)


def build_humanoid_anatomy_profile(skeleton: SkeletonModel) -> HumanoidAnatomyProfile:
    rest_by_name = {bone.name: bone for bone in skeleton.bones}
    support_names = tuple(name for name in ("foot_l", "ball_l", "foot_r", "ball_r") if name in rest_by_name)
    floor_y = min((rest_by_name[name].rest_world_position[1] for name in support_names), default=0.0)
    limb_chains = {
        "left_arm": LimbChainProfile("left_arm", "l", "upperarm_l", "lowerarm_l", "hand_l", "wrist_l_main", "elbow_l_secondary"),
        "right_arm": LimbChainProfile("right_arm", "r", "upperarm_r", "lowerarm_r", "hand_r", "wrist_r_main", "elbow_r_secondary"),
        "left_leg": LimbChainProfile("left_leg", "l", "thigh_l", "calf_l", "foot_l", "ankle_l_main", "knee_l_secondary"),
        "right_leg": LimbChainProfile("right_leg", "r", "thigh_r", "calf_r", "foot_r", "ankle_r_main", "knee_r_secondary"),
    }
    return HumanoidAnatomyProfile(
        rest_by_name=rest_by_name,
        primary_layout=PRIMARY_LAYOUT,
        spine_chain=("pelvis", "spine_01", "spine_02", "spine_03", "spine_04", "spine_05"),
        neck_chain=("spine_05", "neck_01", "neck_02", "head"),
        limb_chains=limb_chains,
        support_joint_names=support_names,
        direction_offsets=FORWARD_OFFSET,
        floor_y=floor_y,
    )
