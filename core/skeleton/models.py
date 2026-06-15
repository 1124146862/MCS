from dataclasses import dataclass, field


Vec3 = tuple[float, float, float]
Quat = tuple[float, float, float, float]


@dataclass(slots=True)
class BoneModel:
    name: str
    parent_name: str | None = None
    local_position: Vec3 = (0.0, 0.0, 0.0)
    local_rotation: Quat = (1.0, 0.0, 0.0, 0.0)
    world_position: Vec3 = (0.0, 0.0, 0.0)
    world_rotation: Quat = (1.0, 0.0, 0.0, 0.0)
    rest_local_position: Vec3 = (0.0, 0.0, 0.0)
    rest_local_rotation: Quat = (1.0, 0.0, 0.0, 0.0)
    rest_world_position: Vec3 = (0.0, 0.0, 0.0)
    rest_world_rotation: Quat = (1.0, 0.0, 0.0, 0.0)
    bone_length: float = 0.0
    primary_axis: Vec3 = (1.0, 0.0, 0.0)


@dataclass(slots=True)
class SkeletonModel:
    name: str = "Rig"
    bones: list[BoneModel] = field(default_factory=list)
