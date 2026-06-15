import re
from pathlib import Path

from PySide6.QtCore import QObject, Property, QTimer, QUrl, Signal, Slot

from app.autoposing.controller_layout import direction_endpoint, direction_rod_length, sync_inactive_controller_targets
from app.autoposing.preview_pipeline import AutoPosingPreviewPipeline
from app.autoposing.runtime_retargeter import RuntimeRetargeter
from app.autoposing.service import AutoPosingService
from app.autoposing.solver_adapter import SyncAutoPosingSolverAdapter
from app.autoposing.visual_policy import controller_fields, pressure_color, show_controller_in_normal, show_pressure_tether, show_target_ghost
from app.document.models import DocumentModel
from core.autoposing.models import PreviewFrameModel
from core.rig.models import RigModel
from core.skeleton.models import BoneModel, SkeletonModel
from infra.importers.runtime_asset_importer import (
    discover_available_models,
    import_fbx_to_runtime_assets,
    import_model_into_project,
)
from infra.importers.skeleton_extractor import extract_skeleton_from_generated_component
from infra.runtime.runtime_pose_writer import RuntimePoseWriter


class DocumentViewModel(QObject):
    documentChanged = Signal()
    modelsChanged = Signal()
    autoposingChanged = Signal()
    autoPosingControllerHandlesChanged = Signal()
    viewportPreviewChanged = Signal()

    def __init__(self, document: DocumentModel):
        super().__init__()
        self._document = document
        self._available_models = list(document.available_model_paths)
        self._autoposing_service = AutoPosingService()
        self._runtime_retargeter = RuntimeRetargeter()
        self._runtime_pose_writer = RuntimePoseWriter()
        self._solver_adapter = SyncAutoPosingSolverAdapter(self._autoposing_service, self._runtime_retargeter)
        self._preview_pipeline = AutoPosingPreviewPipeline(
            document,
            self._autoposing_service,
            self._solver_adapter,
            self._runtime_pose_writer,
        )
        self._preview_request_timer = QTimer(self)
        self._preview_request_timer.setInterval(16)
        self._preview_request_timer.setSingleShot(True)
        self._preview_request_timer.timeout.connect(self._flush_preview_request)
        self._preview_flush_in_progress = False
        self._base_status_message = document.status_message
        self._autoposing_display_mode = "fingers" if document.autoposing.fingers_enabled else "normal"
        self._autoposing_debug_visible = self._autoposing_display_mode == "debug"

        if not document.preview_frame.solved_pose.rig.joints and document.skeleton.bones:
            initial_frame = self._solver_adapter.solve_frame(document.autoposing, document.skeleton, revision=0)
            self._preview_pipeline.prime(initial_frame)
            self._commit_preview_frame(initial_frame)
        else:
            self._preview_pipeline.prime(document.preview_frame)
            self._update_committed_status_message()

    @staticmethod
    def _display_name_for(path: Path) -> str:
        words = re.sub(r"([a-zA-Z])(\d)", r"\1 \2", path.stem.replace("_", " ")).split()
        return " ".join(word.capitalize() for word in words) if words else path.stem

    @staticmethod
    def _pressure_color(state: str) -> str:
        return pressure_color(state)

    @staticmethod
    def _distance(left: tuple[float, float, float], right: tuple[float, float, float]) -> float:
        dx = left[0] - right[0]
        dy = left[1] - right[1]
        dz = left[2] - right[2]
        return (dx * dx + dy * dy + dz * dz) ** 0.5

    @staticmethod
    def _clone_skeleton(skeleton: SkeletonModel) -> SkeletonModel:
        return SkeletonModel(
            name=skeleton.name,
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
                for bone in skeleton.bones
            ],
        )

    def _compose_status_message(self, detail: str) -> str:
        parts = []
        if self._base_status_message:
            parts.append(self._base_status_message)
        if detail and detail != self._base_status_message:
            parts.append(detail)
        return " | ".join(parts)

    def _current_preview_frame(self) -> PreviewFrameModel:
        return self._document.preview_frame

    def _current_preview_skeleton(self) -> SkeletonModel:
        frame = self._current_preview_frame()
        if frame.posed_skeleton.bones:
            return frame.posed_skeleton
        if self._document.posed_skeleton.bones:
            return self._document.posed_skeleton
        return self._document.skeleton

    def _preview_effectors_by_id(self) -> dict[str, object]:
        return {effector.controller_id: effector for effector in self._current_preview_frame().effectors}

    def _preview_pressure_by_name(self) -> dict[str, str]:
        return {joint.name: joint.pressure_state for joint in self._current_preview_frame().solved_pose.rig.joints}

    def _current_controller_preview(self, controller) -> dict[str, tuple[float, float, float] | float | str | bool]:
        effector = self._preview_effectors_by_id().get(controller.identifier)
        target = controller.target.world_position
        solved = effector.solved_world_position if effector is not None else controller.target.solved_world_position
        clamped = effector.clamped_world_position if effector is not None else controller.target.clamped_world_position
        pressure = effector.pressure if effector is not None else controller.target.pressure
        pressure_state = effector.pressure_state if effector is not None else controller.target.pressure_state
        clamped_flag = effector.clamped if effector is not None else controller.target.clamped
        separated = self._distance(target, solved) > 0.01
        dragging = self._preview_pipeline.is_dragging_controller(controller.identifier)
        show_tether = dragging or separated or pressure_state != "normal" or clamped_flag
        return {
            "target": target,
            "solved": solved,
            "clamped": clamped,
            "pressure": pressure,
            "pressureState": pressure_state,
            "clampedFlag": clamped_flag,
            "showGhostTarget": dragging or separated or clamped_flag,
            "showTether": show_tether,
        }

    def _update_committed_status_message(self) -> None:
        committed_detail = self._current_preview_frame().status_message
        self._document.status_message = self._compose_status_message(committed_detail)

    def _apply_preview_frame(self, frame: PreviewFrameModel) -> None:
        self._document.preview_frame = frame

    def _commit_preview_frame(self, frame: PreviewFrameModel) -> None:
        self._apply_preview_frame(frame)
        self._document.solved_pose = frame.solved_pose
        self._document.solver_rig = frame.solved_pose.rig
        self._document.retarget_pose = frame.retarget_pose
        self._document.posed_skeleton = self._clone_skeleton(frame.posed_skeleton)
        self._sync_inactive_controller_layout()
        self._update_committed_status_message()

    def _sync_inactive_controller_layout(self) -> None:
        sync_inactive_controller_targets(self._document.autoposing, self._document.preview_frame.solved_pose)

    def _mark_preview_dirty(self, preview_status_message: str = "Preview solving...") -> int:
        revision = self._preview_pipeline.next_revision()
        self._preview_pipeline.mark_dirty(revision, preview_status_message)
        return revision

    def _schedule_preview_request(self) -> None:
        if self._preview_flush_in_progress:
            return
        if not self._preview_request_timer.isActive():
            self._preview_request_timer.start()

    def _flush_preview_request(self) -> None:
        if self._preview_flush_in_progress:
            return

        self._preview_flush_in_progress = True
        try:
            frame = self._preview_pipeline.request_preview()
            if frame is not None:
                self._apply_preview_frame(frame)
                self.viewportPreviewChanged.emit()
        finally:
            self._preview_flush_in_progress = False

        if self._preview_pipeline.has_pending_preview:
            self._schedule_preview_request()

    def _cancel_scheduled_preview(self) -> None:
        if self._preview_request_timer.isActive():
            self._preview_request_timer.stop()

    def _document_name(self) -> str:
        return self._document.name

    def _source_model_name(self) -> str:
        return self._document.source_model_path.name if self._document.source_model_path else "No source model"

    def _source_model_path(self) -> str:
        return str(self._document.source_model_path) if self._document.source_model_path else ""

    def _current_model_file_name(self) -> str:
        return self._document.source_model_path.name if self._document.source_model_path else ""

    def _current_model_display_name(self) -> str:
        if not self._document.source_model_path:
            return "No source model"
        return self._display_name_for(self._document.source_model_path)

    def _generated_component_url(self) -> QUrl:
        if self._document.generated_component_path:
            return QUrl.fromLocalFile(str(self._document.generated_component_path))
        return QUrl()

    def _has_generated_model(self) -> bool:
        return self._document.generated_component_path is not None

    def _status_message(self) -> str:
        return self._document.status_message

    def _preview_status_message(self) -> str:
        preview_detail = self._preview_pipeline.preview_status_message or self._current_preview_frame().status_message
        return self._compose_status_message(preview_detail)

    def _solver_rig(self) -> RigModel:
        return self._document.solver_rig

    def _joint_display_skeleton(self) -> SkeletonModel:
        if self._document.posed_skeleton.bones:
            return self._document.posed_skeleton
        return self._document.skeleton

    def _current_edit_mode(self) -> str:
        return self._document.autoposing.current_mode

    def _is_autoposing_mode(self) -> bool:
        return self._document.autoposing.current_mode == "autoposing"

    def _is_autoposing_debug_visible(self) -> bool:
        return self._autoposing_display_mode == "debug"

    def _autoposing_display_mode_value(self) -> str:
        return self._autoposing_display_mode

    def _has_autoposing_fingers(self) -> bool:
        return self._autoposing_service.has_finger_controllers(self._document.autoposing)

    def _is_autoposing_fingers_enabled(self) -> bool:
        return self._autoposing_display_mode == "fingers"

    def _skeleton_joints_data(self) -> list[dict[str, str | float | bool]]:
        skeleton = self._joint_display_skeleton()
        return [
            {
                "name": bone.name,
                "parentName": bone.parent_name or "",
                "x": bone.world_position[0],
                "y": bone.world_position[1],
                "z": bone.world_position[2],
                "isTerminal": not any(child.parent_name == bone.name for child in skeleton.bones),
            }
            for bone in skeleton.bones
        ]

    def _skeleton_bones_data(self) -> list[dict[str, str | float]]:
        skeleton = self._joint_display_skeleton()
        by_name = {bone.name: bone for bone in skeleton.bones}
        return [
            {
                "name": bone.name,
                "parentName": bone.parent_name,
                "startX": by_name[bone.parent_name].world_position[0],
                "startY": by_name[bone.parent_name].world_position[1],
                "startZ": by_name[bone.parent_name].world_position[2],
                "endX": bone.world_position[0],
                "endY": bone.world_position[1],
                "endZ": bone.world_position[2],
            }
            for bone in skeleton.bones
            if bone.parent_name and bone.parent_name in by_name
        ]

    def _autoposing_preview_joints_data(self) -> list[dict[str, str | float | bool]]:
        if not self._autoposing_debug_visible:
            return []
        preview_skeleton = self._current_preview_skeleton()
        pressure_by_name = self._preview_pressure_by_name()
        return [
            {
                "name": bone.name,
                "parentName": bone.parent_name or "",
                "x": bone.world_position[0],
                "y": bone.world_position[1],
                "z": bone.world_position[2],
                "isTerminal": not any(child.parent_name == bone.name for child in preview_skeleton.bones),
                "color": self._pressure_color(pressure_by_name.get(bone.name, "normal")),
            }
            for bone in preview_skeleton.bones
        ]

    def _autoposing_preview_bones_data(self) -> list[dict[str, str | float]]:
        if not self._autoposing_debug_visible:
            return []
        preview_skeleton = self._current_preview_skeleton()
        pressure_by_name = self._preview_pressure_by_name()
        by_name = {bone.name: bone for bone in preview_skeleton.bones}
        return [
            {
                "name": bone.name,
                "parentName": bone.parent_name,
                "startX": by_name[bone.parent_name].world_position[0],
                "startY": by_name[bone.parent_name].world_position[1],
                "startZ": by_name[bone.parent_name].world_position[2],
                "endX": bone.world_position[0],
                "endY": bone.world_position[1],
                "endZ": bone.world_position[2],
                "color": self._pressure_color(pressure_by_name.get(bone.name, "normal")),
            }
            for bone in preview_skeleton.bones
            if bone.parent_name and bone.parent_name in by_name
        ]

    def _autoposing_controller_handles_data(self) -> list[dict[str, str | float | bool]]:
        data = []
        for controller in self._document.autoposing.controllers:
            if not show_controller_in_normal(controller):
                continue
            preview = self._current_controller_preview(controller)
            solved = preview["solved"]
            base = solved
            if controller.controller_type == "direction":
                base = self._solved_joint_world_position(controller.joint_name, solved)
                solved = direction_endpoint(controller.identifier, base, preview["target"])
            policy = controller_fields(controller, bool(preview["clampedFlag"]))
            data.append(
                {
                    "id": controller.identifier,
                    "name": controller.name,
                    "controllerType": controller.controller_type,
                    "role": controller.role or controller.controller_type,
                    "jointName": controller.joint_name,
                    "handleX": solved[0],
                    "handleY": solved[1],
                    "handleZ": solved[2],
                    "targetX": controller.target.world_position[0],
                    "targetY": controller.target.world_position[1],
                    "targetZ": controller.target.world_position[2],
                    "baseX": base[0],
                    "baseY": base[1],
                    "baseZ": base[2],
                    "directionLength": direction_rod_length(controller.identifier) if controller.controller_type == "direction" else 0.0,
                    "active": controller.active,
                    "fixed": controller.fixed,
                    "alwaysActive": controller.always_active,
                    "state": self._autoposing_service.controller_state(controller),
                    "constraintRole": self._autoposing_constraint_role(controller),
                    "orientationState": self._autoposing_orientation_state(controller),
                    "selected": controller.selected,
                    "visible": controller.visible,
                    **policy,
                }
            )
        return data

    def _autoposing_controller_visuals_data(self) -> list[dict[str, str | float | bool]]:
        data = []
        for controller in self._document.autoposing.controllers:
            if not show_controller_in_normal(controller):
                continue
            preview = self._current_controller_preview(controller)
            target = preview["target"]
            solved = preview["solved"]
            clamped = preview["clamped"]
            base = solved
            if controller.controller_type == "direction":
                base = self._solved_joint_world_position(controller.joint_name, solved)
                solved = direction_endpoint(controller.identifier, base, target)
                clamped = solved
            policy = controller_fields(controller, bool(preview["clampedFlag"]))
            data.append(
                {
                    "id": controller.identifier,
                    "name": controller.name,
                    "controllerType": controller.controller_type,
                    "role": controller.role or controller.controller_type,
                    "jointName": controller.joint_name,
                    "displayX": solved[0],
                    "displayY": solved[1],
                    "displayZ": solved[2],
                    "x": solved[0],
                    "y": solved[1],
                    "z": solved[2],
                    "targetX": target[0],
                    "targetY": target[1],
                    "targetZ": target[2],
                    "baseX": base[0],
                    "baseY": base[1],
                    "baseZ": base[2],
                    "directionLength": direction_rod_length(controller.identifier) if controller.controller_type == "direction" else 0.0,
                    "solvedX": solved[0],
                    "solvedY": solved[1],
                    "solvedZ": solved[2],
                    "clampedX": clamped[0],
                    "clampedY": clamped[1],
                    "clampedZ": clamped[2],
                    "pressure": preview["pressure"],
                    "pressureState": preview["pressureState"],
                    "clamped": preview["clampedFlag"],
                    "active": controller.active,
                    "fixed": controller.fixed,
                    "alwaysActive": controller.always_active,
                    "state": self._autoposing_service.controller_state(controller),
                    "constraintRole": self._autoposing_constraint_role(controller),
                    "orientationState": self._autoposing_orientation_state(controller),
                    "selected": controller.selected,
                    "visible": controller.visible,
                    "showGhostTarget": show_target_ghost(controller, preview, self._autoposing_debug_visible),
                    "showTether": show_pressure_tether(controller, preview, self._autoposing_debug_visible),
                    **policy,
                }
            )
        return data

    def _autoposing_controllers_data(self) -> list[dict[str, str | float | bool]]:
        return self._autoposing_controller_visuals_data()

    @staticmethod
    def _autoposing_constraint_role(controller) -> str:
        identifier = controller.identifier
        if controller.controller_type == "direction":
            return "direction"
        if identifier.startswith("elbow_") or identifier.startswith("knee_"):
            return "bend"
        if identifier.startswith("hand_"):
            return "palm"
        if identifier.startswith("foot_") or identifier.startswith("ball_"):
            return "foot"
        if identifier.endswith("_tip") or identifier.endswith("_pre_tip"):
            return "finger"
        if identifier in {"pelvis_additional", "chest_additional", "head_additional"}:
            return "body"
        return "position"

    def _autoposing_orientation_state(self, controller) -> str:
        role = self._autoposing_constraint_role(controller)
        if role not in {"palm", "foot", "body", "direction", "finger"}:
            return ""
        return "active" if (controller.active or controller.fixed or controller.always_active) else "inactive"

    def _solved_joint_world_position(self, joint_name: str, fallback: tuple[float, float, float]) -> tuple[float, float, float]:
        joint = next((joint for joint in self._current_preview_frame().solved_pose.rig.joints if joint.name == joint_name), None)
        return joint.world_position if joint is not None else fallback

    def _controller_display_world_position(self, controller) -> tuple[float, float, float]:
        preview = self._current_controller_preview(controller)
        solved = preview["solved"]
        if controller.controller_type == "direction":
            base = self._solved_joint_world_position(controller.joint_name, solved)
            return direction_endpoint(controller.identifier, base, preview["target"])
        return solved

    def _autoposing_topology_edges_data(self) -> list[dict[str, str | float | bool]]:
        controllers_by_id = {controller.identifier: controller for controller in self._document.autoposing.controllers}
        edges = []
        for edge in self._document.autoposing.controller_edges:
            if not edge.visible or edge.edge_type != "topology":
                continue
            start_controller = controllers_by_id.get(edge.start_controller_id)
            end_controller = controllers_by_id.get(edge.end_controller_id)
            if start_controller is None or end_controller is None:
                continue
            if not show_controller_in_normal(start_controller) or not show_controller_in_normal(end_controller):
                continue
            start = self._controller_display_world_position(start_controller)
            end = self._controller_display_world_position(end_controller)
            current_length = self._distance(start, end)
            color = "#1b6f3a"
            if edge.rest_length > 0.001:
                ratio = current_length / edge.rest_length
                if ratio > 1.08:
                    color = "#d94c45"
                elif ratio < 0.92:
                    color = "#2a98ff"
            edges.append(
                {
                    "startId": edge.start_controller_id,
                    "endId": edge.end_controller_id,
                    "startX": start[0],
                    "startY": start[1],
                    "startZ": start[2],
                    "endX": end[0],
                    "endY": end[1],
                    "endZ": end[2],
                    "restLength": edge.rest_length,
                    "currentLength": current_length,
                    "edgeType": edge.edge_type,
                    "color": color,
                    "visible": edge.visible,
                }
            )
        return edges

    def _autoposing_pressure_tethers_data(self) -> list[dict[str, str | float]]:
        segments = []
        for controller in self._document.autoposing.controllers:
            if not show_controller_in_normal(controller):
                continue
            preview = self._current_controller_preview(controller)
            if not show_pressure_tether(controller, preview, self._autoposing_debug_visible):
                continue
            target = preview["target"]
            solved = preview["solved"]
            segments.append(
                {
                    "startId": controller.identifier,
                    "endId": controller.identifier + "_solved",
                    "startX": target[0],
                    "startY": target[1],
                    "startZ": target[2],
                    "endX": solved[0],
                    "endY": solved[1],
                    "endZ": solved[2],
                    "color": self._pressure_color(preview["pressureState"]),
                }
            )
        return segments

    def _autoposing_direction_rods_data(self) -> list[dict[str, str | float | bool]]:
        rods = []
        for controller in self._document.autoposing.controllers:
            if not show_controller_in_normal(controller) or controller.controller_type != "direction":
                continue
            preview = self._current_controller_preview(controller)
            base = self._solved_joint_world_position(controller.joint_name, preview["solved"])
            target = preview["target"]
            endpoint = direction_endpoint(controller.identifier, base, target)
            policy = controller_fields(controller, bool(preview["clampedFlag"]))
            rods.append(
                {
                    "id": controller.identifier,
                    "name": controller.name,
                    "startId": controller.identifier + "_base",
                    "endId": controller.identifier,
                    "startX": base[0],
                    "startY": base[1],
                    "startZ": base[2],
                    "endX": endpoint[0],
                    "endY": endpoint[1],
                    "endZ": endpoint[2],
                    "length": direction_rod_length(controller.identifier),
                    "color": policy["visualColor"],
                    "borderColor": policy["borderColor"],
                    "selected": controller.selected,
                    "fixed": controller.fixed,
                    "locked": policy["locked"],
                    "visible": controller.visible,
                }
            )
        return rods

    def _autoposing_segments_data(self) -> list[dict[str, str | float]]:
        return self._autoposing_pressure_tethers_data()

    def _selected_autoposing_controller(self):
        selected_id = self._document.autoposing.selected_controller_id
        if not selected_id:
            return None
        return next(
            (
                controller
                for controller in self._document.autoposing.controllers
                if controller.identifier == selected_id
            ),
            None,
        )

    def _autoposing_controller_by_id(self, controller_id: str):
        return next(
            (
                controller
                for controller in self._document.autoposing.controllers
                if controller.identifier == controller_id
            ),
            None,
        )

    def _controller_menu_state(self, controller_id: str) -> dict[str, str | bool]:
        controller = self._autoposing_controller_by_id(controller_id)
        if controller is None:
            return {}
        return {
            "id": controller.identifier,
            "name": controller.name,
            "fixed": controller.fixed,
            "active": controller.active,
            "alwaysActive": controller.always_active,
            "state": self._autoposing_service.controller_state(controller),
            **controller_fields(controller, controller.target.clamped),
        }

    def _solve_commit_and_emit(self, status_message: str) -> None:
        self._base_status_message = status_message
        self._mark_preview_dirty()
        frame = self._preview_pipeline.solve_immediate()
        self._commit_preview_frame(frame)
        self.viewportPreviewChanged.emit()
        self.autoPosingControllerHandlesChanged.emit()
        self.documentChanged.emit()
        self.autoposingChanged.emit()

    def _selected_autoposing_controller_name(self) -> str:
        controller = self._selected_autoposing_controller()
        return controller.name if controller is not None else ""

    def _selected_autoposing_controller_type(self) -> str:
        controller = self._selected_autoposing_controller()
        return controller.controller_type if controller is not None else ""

    def _selected_autoposing_controller_joint_name(self) -> str:
        controller = self._selected_autoposing_controller()
        return controller.joint_name if controller is not None else ""

    def _selected_autoposing_controller_status(self) -> str:
        controller = self._selected_autoposing_controller()
        if controller is None:
            return ""
        preview = self._current_controller_preview(controller)
        base = self._autoposing_service.controller_state(controller)
        if preview["clampedFlag"]:
            base += " / clamped"
        elif preview["pressureState"] == "squeeze":
            base += " / squeeze"
        elif preview["pressureState"] == "stretch":
            base += " / stretch"
        return base

    def _selected_autoposing_controller_position(self) -> str:
        controller = self._selected_autoposing_controller()
        if controller is None:
            return ""
        preview = self._current_controller_preview(controller)
        target = preview["target"]
        solved = preview["solved"]
        return f"target {target[0]:.2f}, {target[1]:.2f}, {target[2]:.2f} | solved {solved[0]:.2f}, {solved[1]:.2f}, {solved[2]:.2f}"

    def _available_models_data(self) -> list[dict[str, str | bool]]:
        current_path = self._document.source_model_path
        return [
            {
                "fileName": model_path.name,
                "displayName": self._display_name_for(model_path),
                "sourcePath": str(model_path),
                "selected": current_path == model_path,
            }
            for model_path in self._available_models
        ]

    def _refresh_available_models(self, status_lines: list[str] | None = None) -> None:
        refresh_status_lines = status_lines if status_lines is not None else []
        self._available_models = discover_available_models(refresh_status_lines)
        self._document.available_model_paths = list(self._available_models)

    def _apply_asset_state(
        self,
        source_model: Path | None,
        generated_component_path: Path | None,
        status_lines: list[str],
    ) -> None:
        self._cancel_scheduled_preview()
        self._preview_pipeline.clear_runtime_model()
        self._document.source_model_path = source_model
        self._document.generated_component_path = generated_component_path
        self._document.skeleton = extract_skeleton_from_generated_component(generated_component_path, status_lines)
        self._document.posed_skeleton = self._clone_skeleton(self._document.skeleton)
        self._document.autoposing = self._autoposing_service.build_rig(self._document.skeleton)
        self._document.solver_rig = RigModel()
        self._document.solved_pose = PreviewFrameModel().solved_pose
        self._document.retarget_pose = PreviewFrameModel().retarget_pose
        self._document.preview_frame = PreviewFrameModel()
        self._base_status_message = " | ".join(status_lines)

        frame = self._solver_adapter.solve_frame(self._document.autoposing, self._document.skeleton, revision=0)
        self._preview_pipeline.prime(frame)
        self._commit_preview_frame(frame)
        self.viewportPreviewChanged.emit()
        self.autoPosingControllerHandlesChanged.emit()
        self.documentChanged.emit()
        self.modelsChanged.emit()
        self.autoposingChanged.emit()

    def _select_model_path(self, source_model: Path, prefix_status_lines: list[str] | None = None) -> None:
        status_lines = list(prefix_status_lines or [])
        status_lines.append(f"Selected model: {source_model.name}")
        generated_component = import_fbx_to_runtime_assets(source_model, status_lines)
        if generated_component is None:
            status_lines.append("Showing placeholder body until runtime asset import succeeds.")
        self._apply_asset_state(source_model, generated_component, status_lines)

    @Slot(str)
    def selectModel(self, file_name: str) -> None:
        source_model = next((path for path in self._available_models if path.name == file_name), None)
        if source_model is None:
            self._apply_asset_state(
                self._document.source_model_path,
                self._document.generated_component_path,
                [f"Model not found in library: {file_name}"],
            )
            return
        self._select_model_path(source_model)

    @Slot()
    def newScene(self) -> None:
        self._apply_asset_state(None, None, ["New scene created.", "Placeholder model active."])

    @Slot(str)
    def setEditMode(self, mode: str) -> None:
        normalized_mode = mode if mode in {"view", "joint", "autoposing"} else "view"
        if self._document.autoposing.current_mode == normalized_mode:
            return
        self._cancel_scheduled_preview()
        self._autoposing_service.set_mode(self._document.autoposing, normalized_mode)
        self._base_status_message = f"Edit mode: {normalized_mode}"
        self._update_committed_status_message()
        self.viewportPreviewChanged.emit()
        self.autoPosingControllerHandlesChanged.emit()
        self.documentChanged.emit()
        self.autoposingChanged.emit()

    @Slot(bool)
    def setAutoPosingDebugVisible(self, visible: bool) -> None:
        self.setAutoPosingDisplayMode("debug" if visible else "normal")

    @Slot()
    def toggleAutoPosingDebugVisible(self) -> None:
        self.setAutoPosingDebugVisible(self._autoposing_display_mode != "debug")

    @Slot(str)
    def setAutoPosingDisplayMode(self, mode: str) -> None:
        normalized_mode = mode if mode in {"normal", "fingers", "debug"} else "normal"
        if normalized_mode == "fingers" and not self._autoposing_service.has_finger_controllers(self._document.autoposing):
            normalized_mode = "normal"
        fingers_enabled = normalized_mode in {"fingers", "debug"}
        mode_changed = self._autoposing_display_mode != normalized_mode
        self._cancel_scheduled_preview()
        fingers_changed = self._autoposing_service.set_fingers_enabled(self._document.autoposing, fingers_enabled)
        if not mode_changed and not fingers_changed:
            return
        self._autoposing_display_mode = normalized_mode
        self._autoposing_debug_visible = normalized_mode == "debug"
        status = {
            "normal": "AutoPosing display: Normal",
            "fingers": "AutoPosing display: Fingers",
            "debug": "AutoPosing display: Debug",
        }[normalized_mode]
        self._solve_commit_and_emit(status)

    @Slot(bool)
    def setAutoPosingFingersEnabled(self, enabled: bool) -> None:
        self.setAutoPosingDisplayMode("fingers" if enabled else "normal")

    @Slot()
    def toggleAutoPosingFingersEnabled(self) -> None:
        self.setAutoPosingFingersEnabled(self._autoposing_display_mode != "fingers")

    @Slot(str)
    def selectAutoPosingController(self, controller_id: str) -> None:
        self._autoposing_service.select_controller(self._document.autoposing, controller_id or None)
        self.viewportPreviewChanged.emit()
        self.autoPosingControllerHandlesChanged.emit()
        self.autoposingChanged.emit()

    @Slot(str)
    def beginAutoPosingDrag(self, controller_id: str) -> None:
        self._cancel_scheduled_preview()
        self._autoposing_service.select_controller(self._document.autoposing, controller_id or None)
        self._preview_pipeline.begin_drag(controller_id)
        self.autoposingChanged.emit()

    @Slot(str, float, float, float)
    def previewAutoPosingController(self, controller_id: str, x: float, y: float, z: float) -> None:
        revision = self._preview_pipeline.next_revision()
        moved = self._preview_pipeline.update_target(controller_id, (x, y, z), revision)
        if not moved:
            return
        self._schedule_preview_request()

    @Slot(str, float, float, float)
    def moveAutoPosingController(self, controller_id: str, x: float, y: float, z: float) -> None:
        self.previewAutoPosingController(controller_id, x, y, z)

    @Slot(str)
    def endAutoPosingDrag(self, controller_id: str) -> None:
        self._cancel_scheduled_preview()
        self._preview_pipeline.end_drag(controller_id)
        self._base_status_message = f"AutoPosing moved: {controller_id}"
        frame = self._preview_pipeline.solve_immediate()
        self._commit_preview_frame(frame)
        self.viewportPreviewChanged.emit()
        self.autoPosingControllerHandlesChanged.emit()
        self.documentChanged.emit()
        self.autoposingChanged.emit()

    @Slot()
    def toggleSelectedAutoPosingFixed(self) -> None:
        controller = self._selected_autoposing_controller()
        if controller is None:
            return
        self._cancel_scheduled_preview()
        preview = self._current_controller_preview(controller)
        fixed = self._autoposing_service.toggle_fixed(
            self._document.autoposing,
            controller.identifier,
            preview["solved"],
        )
        self._solve_commit_and_emit(f"{controller.name} {'fixed' if fixed else 'unfixed'}")

    @Slot()
    def resetSelectedAutoPosingController(self) -> None:
        controller = self._selected_autoposing_controller()
        if controller is None:
            return
        self.resetAutoPosingController(controller.identifier)

    @Slot()
    def unlockOrResetSelectedAutoPosingController(self) -> None:
        controller = self._selected_autoposing_controller()
        if controller is None:
            return
        if controller.active and not controller.always_active:
            self.unlockAutoPosingController(controller.identifier)
            return
        self.resetAutoPosingController(controller.identifier)

    @Slot()
    def toggleSelectedAutoPosingLock(self) -> None:
        controller = self._selected_autoposing_controller()
        if controller is None or controller.always_active:
            return
        if controller.active:
            self.unlockAutoPosingController(controller.identifier)
        else:
            self.lockAutoPosingController(controller.identifier)

    @Slot(str)
    def lockAutoPosingController(self, controller_id: str) -> None:
        controller = self._autoposing_controller_by_id(controller_id)
        if controller is None:
            return
        self._cancel_scheduled_preview()
        if not self._autoposing_service.lock_controller(self._document.autoposing, controller_id):
            return
        self._solve_commit_and_emit(f"{controller.name} locked")

    @Slot(str)
    def unlockAutoPosingController(self, controller_id: str) -> None:
        controller = self._autoposing_controller_by_id(controller_id)
        if controller is None:
            return
        self._cancel_scheduled_preview()
        if not self._autoposing_service.unlock_controller(self._document.autoposing, controller_id):
            return
        self._solve_commit_and_emit(f"{controller.name} unlocked")

    @Slot(str)
    def fixAutoPosingController(self, controller_id: str) -> None:
        controller = self._autoposing_controller_by_id(controller_id)
        if controller is None:
            return
        self._cancel_scheduled_preview()
        preview = self._current_controller_preview(controller)
        if not self._autoposing_service.fix_controller(self._document.autoposing, controller_id, preview["solved"]):
            return
        self._solve_commit_and_emit(f"{controller.name} fixed")

    @Slot(str)
    def unfixAutoPosingController(self, controller_id: str) -> None:
        controller = self._autoposing_controller_by_id(controller_id)
        if controller is None:
            return
        self._cancel_scheduled_preview()
        if not self._autoposing_service.unfix_controller(self._document.autoposing, controller_id):
            return
        self._solve_commit_and_emit(f"{controller.name} unfixed")

    @Slot(str)
    def resetAutoPosingController(self, controller_id: str) -> None:
        controller = self._autoposing_controller_by_id(controller_id)
        if controller is None:
            return
        self._cancel_scheduled_preview()
        reset = self._autoposing_service.reset_controller(
            self._document.autoposing,
            controller_id,
            self._current_preview_frame().solved_pose,
        )
        if not reset:
            return
        self._solve_commit_and_emit(f"AutoPosing reset: {controller.name}")

    @Slot(str, result="QVariantMap")
    def autoPosingControllerMenuState(self, controller_id: str) -> dict[str, str | bool]:
        return self._controller_menu_state(controller_id)

    @Slot(QObject)
    def bindRuntimeModel(self, runtime_model: QObject) -> None:
        self._cancel_scheduled_preview()
        self._preview_pipeline.bind_runtime_model(runtime_model)
        self._commit_preview_frame(self._preview_pipeline.consume_latest_frame())
        self.viewportPreviewChanged.emit()
        self.autoPosingControllerHandlesChanged.emit()
        self.documentChanged.emit()
        self.autoposingChanged.emit()

    @Slot()
    def clearRuntimeModel(self) -> None:
        self._cancel_scheduled_preview()
        self._preview_pipeline.clear_runtime_model()
        self._commit_preview_frame(self._preview_pipeline.consume_latest_frame())
        self.viewportPreviewChanged.emit()
        self.autoPosingControllerHandlesChanged.emit()
        self.documentChanged.emit()
        self.autoposingChanged.emit()

    @Slot(str)
    def importModelFromFile(self, file_url: str) -> None:
        local_file = QUrl(file_url).toLocalFile() or file_url
        if not local_file:
            self._apply_asset_state(
                self._document.source_model_path,
                self._document.generated_component_path,
                ["No model selected."],
            )
            return

        status_lines: list[str] = []
        imported_model = import_model_into_project(Path(local_file), status_lines)
        self._refresh_available_models(status_lines)
        if imported_model is None:
            self._apply_asset_state(
                self._document.source_model_path,
                self._document.generated_component_path,
                status_lines or ["Model import failed."],
            )
            return
        self._select_model_path(imported_model, status_lines)

    documentName = Property(str, _document_name, constant=True)
    sourceModelName = Property(str, _source_model_name, notify=documentChanged)
    sourceModelPath = Property(str, _source_model_path, notify=documentChanged)
    currentModelFileName = Property(str, _current_model_file_name, notify=documentChanged)
    currentModelDisplayName = Property(str, _current_model_display_name, notify=documentChanged)
    generatedComponentUrl = Property(QUrl, _generated_component_url, notify=documentChanged)
    hasGeneratedModel = Property(bool, _has_generated_model, notify=documentChanged)
    statusMessage = Property(str, _status_message, notify=documentChanged)
    previewStatusMessage = Property(str, _preview_status_message, notify=viewportPreviewChanged)
    currentEditMode = Property(str, _current_edit_mode, notify=autoposingChanged)
    isAutoPosingMode = Property(bool, _is_autoposing_mode, notify=autoposingChanged)
    autoPosingDisplayMode = Property(str, _autoposing_display_mode_value, notify=autoposingChanged)
    isAutoPosingDebugVisible = Property(bool, _is_autoposing_debug_visible, notify=autoposingChanged)
    hasAutoPosingFingers = Property(bool, _has_autoposing_fingers, notify=autoPosingControllerHandlesChanged)
    isAutoPosingFingersEnabled = Property(bool, _is_autoposing_fingers_enabled, notify=autoposingChanged)
    skeletonJoints = Property("QVariantList", _skeleton_joints_data, notify=documentChanged)
    skeletonBones = Property("QVariantList", _skeleton_bones_data, notify=documentChanged)
    autoPosingPreviewJoints = Property("QVariantList", _autoposing_preview_joints_data, notify=viewportPreviewChanged)
    autoPosingPreviewBones = Property("QVariantList", _autoposing_preview_bones_data, notify=viewportPreviewChanged)
    autoPosingControllerHandles = Property("QVariantList", _autoposing_controller_handles_data, notify=autoPosingControllerHandlesChanged)
    autoPosingControllerVisuals = Property("QVariantList", _autoposing_controller_visuals_data, notify=viewportPreviewChanged)
    autoPosingControllers = Property("QVariantList", _autoposing_controllers_data, notify=viewportPreviewChanged)
    autoPosingTopologyEdges = Property("QVariantList", _autoposing_topology_edges_data, notify=viewportPreviewChanged)
    autoPosingPressureTethers = Property("QVariantList", _autoposing_pressure_tethers_data, notify=viewportPreviewChanged)
    autoPosingDirectionRods = Property("QVariantList", _autoposing_direction_rods_data, notify=viewportPreviewChanged)
    autoPosingSegments = Property("QVariantList", _autoposing_segments_data, notify=viewportPreviewChanged)
    selectedAutoPosingControllerName = Property(str, _selected_autoposing_controller_name, notify=autoposingChanged)
    selectedAutoPosingControllerType = Property(str, _selected_autoposing_controller_type, notify=autoposingChanged)
    selectedAutoPosingControllerJointName = Property(str, _selected_autoposing_controller_joint_name, notify=autoposingChanged)
    selectedAutoPosingControllerStatus = Property(str, _selected_autoposing_controller_status, notify=viewportPreviewChanged)
    selectedAutoPosingControllerPosition = Property(str, _selected_autoposing_controller_position, notify=viewportPreviewChanged)
    availableModels = Property("QVariantList", _available_models_data, notify=modelsChanged)
