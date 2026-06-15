from dataclasses import dataclass, field

from core.skeleton.models import Quat, Vec3


@dataclass(slots=True)
class JointModel:
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
    pressure: float = 0.0
    pressure_state: str = "normal"


@dataclass(slots=True)
class RigModel:
    name: str = "SolverRig"
    joints: list[JointModel] = field(default_factory=list)
