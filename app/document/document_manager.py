from app.autoposing.runtime_retargeter import RuntimeRetargeter
from app.autoposing.service import AutoPosingService
from app.autoposing.solver_adapter import SyncAutoPosingSolverAdapter
from core.animation.models import AnimationClipModel
from core.scene.models import SceneModel
from core.skeleton.models import BoneModel, SkeletonModel
from infra.importers.runtime_asset_importer import prepare_startup_asset_state
from infra.importers.skeleton_extractor import extract_skeleton_from_generated_component

from .models import DocumentModel


class DocumentManager:
    """Creates and owns editor documents."""

    def __init__(self) -> None:
        self._autoposing_service = AutoPosingService()
        self._runtime_retargeter = RuntimeRetargeter()
        self._solver_adapter = SyncAutoPosingSolverAdapter(self._autoposing_service, self._runtime_retargeter)

    def open_startup_document(self) -> DocumentModel:
        asset_state = prepare_startup_asset_state()
        skeleton = extract_skeleton_from_generated_component(asset_state.generated_component_path, [])
        autoposing_rig = self._autoposing_service.build_rig(skeleton)
        preview_frame = self._solver_adapter.solve_frame(autoposing_rig, skeleton, revision=0)

        return DocumentModel(
            name="Startup Document",
            scene=SceneModel(name="Viewport Scene"),
            skeleton=skeleton,
            posed_skeleton=SkeletonModel(
                name=preview_frame.posed_skeleton.name,
                bones=[
                    BoneModel(
                        name=bone.name,
                        parent_name=bone.parent_name,
                        local_position=bone.local_position,
                        local_rotation=bone.local_rotation,
                        world_position=bone.world_position,
                        world_rotation=bone.world_rotation,
                        rest_local_position=bone.rest_local_position,
                        rest_local_rotation=bone.rest_local_rotation,
                        rest_world_position=bone.rest_world_position,
                        rest_world_rotation=bone.rest_world_rotation,
                        bone_length=bone.bone_length,
                        primary_axis=bone.primary_axis,
                    )
                    for bone in preview_frame.posed_skeleton.bones
                ],
            ),
            clips=[AnimationClipModel(name="Take001", frame_count=100)],
            autoposing=autoposing_rig,
            solver_rig=preview_frame.solved_pose.rig,
            solved_pose=preview_frame.solved_pose,
            retarget_pose=preview_frame.retarget_pose,
            preview_frame=preview_frame,
            available_model_paths=asset_state.available_model_paths,
            source_model_path=asset_state.source_model_path,
            generated_component_path=asset_state.generated_component_path,
            status_message=asset_state.status_message,
        )
