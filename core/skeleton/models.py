from dataclasses import dataclass, field


@dataclass(slots=True)
class BoneModel:
    name: str
    parent_name: str | None = None
    world_position: tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass(slots=True)
class SkeletonModel:
    name: str = "Rig"
    bones: list[BoneModel] = field(default_factory=list)
