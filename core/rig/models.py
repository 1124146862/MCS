from dataclasses import dataclass, field


@dataclass(slots=True)
class JointModel:
    name: str
    parent_name: str | None = None
    rest_world_position: tuple[float, float, float] = (0.0, 0.0, 0.0)
    world_position: tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass(slots=True)
class RigModel:
    name: str = "ProxyRig"
    joints: list[JointModel] = field(default_factory=list)
