from dataclasses import dataclass, field
from pathlib import Path

from core.autoposing.models import AutoPosingRigModel
from core.animation.models import AnimationClipModel
from core.rig.models import RigModel
from core.scene.models import SceneModel
from core.skeleton.models import SkeletonModel


@dataclass(slots=True)
class DocumentModel:
    name: str
    scene: SceneModel
    skeleton: SkeletonModel
    clips: list[AnimationClipModel] = field(default_factory=list)
    autoposing: AutoPosingRigModel = field(default_factory=AutoPosingRigModel)
    proxy_rig: RigModel = field(default_factory=RigModel)
    available_model_paths: list[Path] = field(default_factory=list)
    source_model_path: Path | None = None
    generated_component_path: Path | None = None
    status_message: str = ""
