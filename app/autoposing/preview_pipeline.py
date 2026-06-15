from __future__ import annotations

from dataclasses import replace

from PySide6.QtCore import QObject

from app.document.models import DocumentModel
from core.autoposing.models import PreviewFrameModel

from .service import AutoPosingService
from .solver_adapter import AutoPosingSolverAdapter
from infra.runtime.runtime_pose_writer import RuntimePoseWriter


class AutoPosingPreviewPipeline:
    def __init__(
        self,
        document: DocumentModel,
        autoposing_service: AutoPosingService,
        solver_adapter: AutoPosingSolverAdapter,
        runtime_pose_writer: RuntimePoseWriter,
    ) -> None:
        self._document = document
        self._autoposing_service = autoposing_service
        self._solver_adapter = solver_adapter
        self._runtime_pose_writer = runtime_pose_writer
        self._latest_frame = document.preview_frame
        self._latest_input_revision = document.preview_frame.revision
        self._latest_completed_revision = document.preview_frame.revision
        self._pending_revision: int | None = None
        self._drag_active = False
        self._dragging_controller_id = ""
        self._preview_status_message = document.preview_frame.status_message
        self._dropped_frame_count = 0

    @property
    def has_pending_preview(self) -> bool:
        return self._pending_revision is not None

    @property
    def preview_status_message(self) -> str:
        return self._preview_status_message

    @property
    def dragging_controller_id(self) -> str:
        return self._dragging_controller_id

    def is_dragging_controller(self, controller_id: str) -> bool:
        return self._drag_active and self._dragging_controller_id == controller_id

    def consume_latest_frame(self) -> PreviewFrameModel:
        return self._latest_frame

    def next_revision(self) -> int:
        self._latest_input_revision += 1
        return self._latest_input_revision

    def prime(self, frame: PreviewFrameModel) -> None:
        self._latest_frame = frame
        self._latest_input_revision = frame.revision
        self._latest_completed_revision = frame.revision
        self._pending_revision = None
        self._drag_active = False
        self._dragging_controller_id = ""
        self._preview_status_message = frame.status_message
        self._dropped_frame_count = 0

    def begin_drag(self, controller_id: str) -> None:
        self._drag_active = True
        self._dragging_controller_id = controller_id
        self._preview_status_message = self._latest_frame.status_message

    def update_target(self, controller_id: str, world_position: tuple[float, float, float], revision: int) -> bool:
        moved = self._autoposing_service.move_controller(self._document.autoposing, controller_id, world_position)
        if not moved:
            return False
        self.mark_dirty(revision, "Preview solving...")
        return True

    def mark_dirty(self, revision: int, preview_status_message: str = "Preview solving...") -> None:
        self._latest_input_revision = max(self._latest_input_revision, revision)
        self._pending_revision = self._latest_input_revision
        self._preview_status_message = preview_status_message

    def request_preview(self) -> PreviewFrameModel | None:
        return self._solve_pending(force=False, capture_runtime_skeleton=False)

    def solve_immediate(self) -> PreviewFrameModel:
        frame = self._solve_pending(force=True, capture_runtime_skeleton=True)
        return frame if frame is not None else self._latest_frame

    def end_drag(self, controller_id: str) -> None:
        self._drag_active = False
        if self._dragging_controller_id == controller_id:
            self._dragging_controller_id = ""

    def bind_runtime_model(self, runtime_model: QObject | None) -> bool:
        bound = self._runtime_pose_writer.bind_runtime_model(runtime_model)
        self._latest_frame = self._refresh_runtime_capture(self._latest_frame)
        self._preview_status_message = self._latest_frame.status_message
        return bound

    def clear_runtime_model(self) -> None:
        self._runtime_pose_writer.clear()
        self._latest_frame = self._with_posed_skeleton(
            self._latest_frame,
            self._solver_adapter.build_posed_skeleton(self._document.skeleton, self._latest_frame.solved_pose),
        )
        self._preview_status_message = self._latest_frame.status_message

    def _solve_pending(self, force: bool, capture_runtime_skeleton: bool) -> PreviewFrameModel | None:
        if self._pending_revision is None:
            if not force:
                return None
            if capture_runtime_skeleton:
                self._latest_frame = self._refresh_runtime_capture(self._latest_frame)
            else:
                self._latest_frame = self._apply_runtime_pose(self._latest_frame)
            return self._latest_frame

        requested_revision = self._pending_revision
        self._pending_revision = None
        frame = self._solver_adapter.solve_frame(self._document.autoposing, self._document.skeleton, requested_revision)
        if frame.revision < self._latest_input_revision:
            self._dropped_frame_count += 1
            self._preview_status_message = "Frame dropped"
            return None

        if capture_runtime_skeleton:
            frame = self._refresh_runtime_capture(frame)
        else:
            frame = self._apply_runtime_pose(frame)
        self._latest_frame = frame
        self._latest_completed_revision = frame.revision
        self._preview_status_message = frame.status_message
        return frame

    def _apply_runtime_pose(self, frame: PreviewFrameModel) -> PreviewFrameModel:
        if frame.retarget_pose.joints:
            self._runtime_pose_writer.apply_retarget_pose(frame.retarget_pose)
        return frame

    def _refresh_runtime_capture(self, frame: PreviewFrameModel) -> PreviewFrameModel:
        frame = self._apply_runtime_pose(frame)
        runtime_skeleton = self._runtime_pose_writer.capture_runtime_skeleton(self._document.skeleton)
        if runtime_skeleton is not None:
            return self._with_posed_skeleton(frame, runtime_skeleton)
        fallback_skeleton = self._solver_adapter.build_posed_skeleton(self._document.skeleton, frame.solved_pose)
        return self._with_posed_skeleton(frame, fallback_skeleton)

    @staticmethod
    def _with_posed_skeleton(frame: PreviewFrameModel, posed_skeleton) -> PreviewFrameModel:
        return replace(frame, posed_skeleton=posed_skeleton)
