from dataclasses import dataclass, field
from pathlib import Path

from core.autoposing.models import AutoPosingRigModel, RetargetPoseModel, SolvedPoseModel
from core.animation.models import AnimationClipModel
from core.rig.models import RigModel
from core.scene.models import SceneModel
from core.skeleton.models import SkeletonModel


@dataclass(slots=True)
class DocumentModel:
    name: str
    scene: SceneModel
    skeleton: SkeletonModel
    posed_skeleton: SkeletonModel = field(default_factory=SkeletonModel)
    clips: list[AnimationClipModel] = field(default_factory=list)
    autoposing: AutoPosingRigModel = field(default_factory=AutoPosingRigModel)
    solver_rig: RigModel = field(default_factory=RigModel)
    solved_pose: SolvedPoseModel = field(default_factory=SolvedPoseModel)
    retarget_pose: RetargetPoseModel = field(default_factory=RetargetPoseModel)
    available_model_paths: list[Path] = field(default_factory=list)
    source_model_path: Path | None = None
    generated_component_path: Path | None = None
    status_message: str = ""
