from core.animation.models import AnimationClipModel
from core.scene.models import SceneModel
from core.skeleton.models import SkeletonModel
from infra.importers.runtime_asset_importer import prepare_startup_asset_state
from infra.importers.skeleton_extractor import extract_skeleton_from_generated_component

from .models import DocumentModel


class DocumentManager:
    """Creates and owns editor documents."""

    def open_startup_document(self) -> DocumentModel:
        asset_state = prepare_startup_asset_state()
        skeleton = extract_skeleton_from_generated_component(asset_state.generated_component_path, [])

        return DocumentModel(
            name="Startup Document",
            scene=SceneModel(name="Viewport Scene"),
            skeleton=skeleton,
            clips=[AnimationClipModel(name="Take001", frame_count=100)],
            available_model_paths=asset_state.available_model_paths,
            source_model_path=asset_state.source_model_path,
            generated_component_path=asset_state.generated_component_path,
            status_message=asset_state.status_message,
        )
