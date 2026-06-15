from dataclasses import dataclass, field

from core.rig.models import RigModel
from core.skeleton.models import Quat, SkeletonModel, Vec3


@dataclass(slots=True)
class ControllerTargetModel:
    controller_id: str
    world_position: Vec3
    default_position: Vec3
    solved_world_position: Vec3 = (0.0, 0.0, 0.0)
    clamped_world_position: Vec3 = (0.0, 0.0, 0.0)
    pressure: float = 0.0
    pressure_state: str = "normal"
    clamped: bool = False


@dataclass(slots=True)
class AutoPosingControllerModel:
    identifier: str
    name: str
    controller_type: str
    joint_name: str
    target: ControllerTargetModel
    role: str = ""
    visible: bool = True
    active: bool = False
    fixed: bool = False
    always_active: bool = False
    selected: bool = False


@dataclass(slots=True)
class AutoPosingControllerEdgeModel:
    start_controller_id: str
    end_controller_id: str
    edge_type: str = "topology"
    rest_length: float = 0.0
    visible: bool = True


@dataclass(slots=True)
class SolvedEffectorModel:
    controller_id: str
    joint_name: str
    target_world_position: Vec3
    solved_world_position: Vec3
    clamped_world_position: Vec3
    pressure: float = 0.0
    pressure_state: str = "normal"
    clamped: bool = False


@dataclass(slots=True)
class SolvedPoseModel:
    rig: RigModel = field(default_factory=RigModel)
    effectors: list[SolvedEffectorModel] = field(default_factory=list)
    status_message: str = ""


@dataclass(slots=True)
class RetargetJointPoseModel:
    name: str
    local_position: Vec3
    local_rotation: Quat
    world_position: Vec3 | None = None


@dataclass(slots=True)
class RetargetPoseModel:
    joints: list[RetargetJointPoseModel] = field(default_factory=list)
    status_message: str = ""


@dataclass(slots=True)
class PreviewFrameModel:
    revision: int = 0
    solved_pose: SolvedPoseModel = field(default_factory=SolvedPoseModel)
    retarget_pose: RetargetPoseModel = field(default_factory=RetargetPoseModel)
    posed_skeleton: SkeletonModel = field(default_factory=SkeletonModel)
    effectors: list[SolvedEffectorModel] = field(default_factory=list)
    status_message: str = ""
    timings: dict[str, float] = field(default_factory=dict)


@dataclass(slots=True)
class AutoPosingRigModel:
    controllers: list[AutoPosingControllerModel] = field(default_factory=list)
    controller_edges: list[AutoPosingControllerEdgeModel] = field(default_factory=list)
    current_mode: str = "view"
    selected_controller_id: str | None = None
    fingers_enabled: bool = False
    finger_defaults_initialized: bool = False
