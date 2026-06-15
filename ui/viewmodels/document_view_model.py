import re
from pathlib import Path

from PySide6.QtCore import QObject, Property, QTimer, QUrl, Signal, Slot

from app.autoposing.runtime_retargeter import RuntimeRetargeter
from app.autoposing.service import AutoPosingService
from app.document.models import DocumentModel
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
    viewportPoseChanged = Signal()

    def __init__(self, document: DocumentModel):
        super().__init__()
        self._document = document
        self._available_models = list(document.available_model_paths)
        self._autoposing_service = AutoPosingService()
        self._runtime_retargeter = RuntimeRetargeter()
        self._runtime_pose_writer = RuntimePoseWriter()
        self._drag_preview_timer = QTimer(self)
        self._drag_preview_timer.setInterval(16)
        self._drag_preview_timer.timeout.connect(self._flush_drag_preview)
        self._drag_active = False
        self._drag_preview_pending = False
        self._base_status_message = document.status_message

    @staticmethod
    def _display_name_for(path: Path) -> str:
        words = re.sub(r"([a-zA-Z])(\d)", r"\1 \2", path.stem.replace("_", " ")).split()
        return " ".join(word.capitalize() for word in words) if words else path.stem

    @staticmethod
    def _pressure_color(state: str) -> str:
        if state == "stretch":
            return "#d94c45"
        if state == "squeeze":
            return "#2a98ff"
        return "#1b6f3a"

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

    def _solver_rig(self) -> RigModel:
        return self._document.solver_rig

    def _joint_display_skeleton(self) -> SkeletonModel:
        if self._document.posed_skeleton.bones:
            return self._document.posed_skeleton
        return self._document.skeleton

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

    @staticmethod
    def _skeleton_from_solver_rig(reference_skeleton: SkeletonModel, solver_rig: RigModel) -> SkeletonModel:
        reference_by_name = {bone.name: bone for bone in reference_skeleton.bones}
        bones = []
        for joint in solver_rig.joints:
            reference = reference_by_name.get(joint.name)
            bones.append(
                BoneModel(
                    name=joint.name,
                    parent_name=joint.parent_name,
                    local_position=reference.local_position if reference else joint.local_position,
                    local_rotation=reference.local_rotation if reference else joint.local_rotation,
                    world_position=joint.world_position,
                    world_rotation=joint.world_rotation,
                    rest_local_position=reference.rest_local_position if reference else joint.rest_local_position,
                    rest_local_rotation=reference.rest_local_rotation if reference else joint.rest_local_rotation,
                    rest_world_position=reference.rest_world_position if reference else joint.rest_world_position,
                    rest_world_rotation=reference.rest_world_rotation if reference else joint.rest_world_rotation,
                    bone_length=reference.bone_length if reference else joint.bone_length,
                    primary_axis=reference.primary_axis if reference else joint.primary_axis,
                )
            )
        return SkeletonModel(name=reference_skeleton.name, bones=bones)

    def _current_edit_mode(self) -> str:
        return self._document.autoposing.current_mode

    def _is_autoposing_mode(self) -> bool:
        return self._document.autoposing.current_mode == "autoposing"

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

    def _autoposing_preview_skeleton(self) -> SkeletonModel:
        solver_skeleton = self._skeleton_from_solver_rig(self._document.skeleton, self._document.solver_rig)
        solver_joint_names = {bone.name for bone in solver_skeleton.bones}
        if not solver_joint_names:
            return SkeletonModel(name=self._document.skeleton.name)

        by_name = {bone.name: bone for bone in self._document.skeleton.bones}

        def visible_parent_name(bone: BoneModel) -> str | None:
            parent_name = bone.parent_name
            while parent_name:
                if parent_name in solver_joint_names:
                    return parent_name
                parent = by_name.get(parent_name)
                parent_name = parent.parent_name if parent is not None else None
            return None

        return SkeletonModel(
            name=solver_skeleton.name,
            bones=[
                BoneModel(
                    name=bone.name,
                    parent_name=visible_parent_name(bone),
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
                for bone in solver_skeleton.bones
                if bone.name in solver_joint_names
            ],
        )

    def _autoposing_preview_joints_data(self) -> list[dict[str, str | float | bool]]:
        preview_skeleton = self._autoposing_preview_skeleton()
        pressure_by_name = {joint.name: joint.pressure_state for joint in self._document.solver_rig.joints}
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
        preview_skeleton = self._autoposing_preview_skeleton()
        pressure_by_name = {joint.name: joint.pressure_state for joint in self._document.solver_rig.joints}
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

    def _effectors_by_id(self) -> dict[str, object]:
        return {effector.controller_id: effector for effector in self._document.solved_pose.effectors}

    def _autoposing_controllers_data(self) -> list[dict[str, str | float | bool]]:
        effectors_by_id = self._effectors_by_id()
        return [
            {
                "id": controller.identifier,
                "name": controller.name,
                "controllerType": controller.controller_type,
                "jointName": controller.joint_name,
                "x": effectors_by_id.get(controller.identifier, controller.target).solved_world_position[0]
                if controller.identifier in effectors_by_id
                else controller.target.solved_world_position[0],
                "y": effectors_by_id.get(controller.identifier, controller.target).solved_world_position[1]
                if controller.identifier in effectors_by_id
                else controller.target.solved_world_position[1],
                "z": effectors_by_id.get(controller.identifier, controller.target).solved_world_position[2]
                if controller.identifier in effectors_by_id
                else controller.target.solved_world_position[2],
                "targetX": controller.target.world_position[0],
                "targetY": controller.target.world_position[1],
                "targetZ": controller.target.world_position[2],
                "solvedX": effectors_by_id.get(controller.identifier, controller.target).solved_world_position[0]
                if controller.identifier in effectors_by_id
                else controller.target.solved_world_position[0],
                "solvedY": effectors_by_id.get(controller.identifier, controller.target).solved_world_position[1]
                if controller.identifier in effectors_by_id
                else controller.target.solved_world_position[1],
                "solvedZ": effectors_by_id.get(controller.identifier, controller.target).solved_world_position[2]
                if controller.identifier in effectors_by_id
                else controller.target.solved_world_position[2],
                "clampedX": controller.target.clamped_world_position[0],
                "clampedY": controller.target.clamped_world_position[1],
                "clampedZ": controller.target.clamped_world_position[2],
                "pressure": controller.target.pressure,
                "pressureState": controller.target.pressure_state,
                "clamped": controller.target.clamped,
                "active": controller.active,
                "fixed": controller.fixed,
                "alwaysActive": controller.always_active,
                "state": self._autoposing_service.controller_state(controller),
                "selected": controller.selected,
                "visible": controller.visible,
                "showGhostTarget": controller.target.clamped
                or controller.target.pressure_state != "normal",
            }
            for controller in self._document.autoposing.controllers
            if controller.visible
        ]

    def _autoposing_segments_data(self) -> list[dict[str, str | float]]:
        segments = []
        for controller in self._document.autoposing.controllers:
            if not controller.visible:
                continue
            target = controller.target.world_position
            solved = controller.target.solved_world_position
            if controller.controller_type == "direction" and target == solved:
                continue
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
                    "color": self._pressure_color(controller.target.pressure_state),
                }
            )
        return segments

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
        base = self._autoposing_service.controller_state(controller)
        if controller.target.clamped:
            base += " / clamped"
        elif controller.target.pressure_state == "squeeze":
            base += " / squeeze"
        return base

    def _selected_autoposing_controller_position(self) -> str:
        controller = self._selected_autoposing_controller()
        if controller is None:
            return ""
        target = controller.target.world_position
        solved = controller.target.solved_world_position
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

    def _update_status_message(self) -> None:
        parts = [part for part in [self._base_status_message, self._document.solved_pose.status_message] if part]
        self._document.status_message = " | ".join(parts)

    def _refresh_posed_skeleton(self) -> None:
        runtime_skeleton = self._runtime_pose_writer.capture_runtime_skeleton(self._document.skeleton)
        if runtime_skeleton is not None:
            self._document.posed_skeleton = runtime_skeleton
            return
        self._document.posed_skeleton = self._skeleton_from_solver_rig(self._document.skeleton, self._document.solver_rig)

    def _refresh_solver_state(self) -> None:
        self._document.solved_pose = self._autoposing_service.solve_pose(self._document.autoposing, self._document.skeleton)
        self._document.solver_rig = self._document.solved_pose.rig
        self._document.retarget_pose = self._runtime_retargeter.retarget(
            self._document.autoposing,
            self._document.solved_pose,
            self._document.skeleton,
        )
        self._runtime_pose_writer.apply_retarget_pose(self._document.retarget_pose)
        self._refresh_posed_skeleton()
        self._update_status_message()
        self.viewportPoseChanged.emit()

    def _schedule_drag_preview(self) -> None:
        self._drag_preview_pending = True
        if not self._drag_preview_timer.isActive():
            self._drag_preview_timer.start()

    def _flush_drag_preview(self) -> None:
        if not self._drag_preview_pending:
            if self._drag_preview_timer.isActive():
                self._drag_preview_timer.stop()
            return
        self._drag_preview_pending = False
        self._refresh_solver_state()
        if not self._drag_active and self._drag_preview_timer.isActive():
            self._drag_preview_timer.stop()

    def _apply_asset_state(
        self,
        source_model: Path | None,
        generated_component_path: Path | None,
        status_lines: list[str],
    ) -> None:
        self._runtime_pose_writer.clear()
        self._drag_preview_timer.stop()
        self._drag_active = False
        self._drag_preview_pending = False
        self._document.source_model_path = source_model
        self._document.generated_component_path = generated_component_path
        self._document.skeleton = extract_skeleton_from_generated_component(generated_component_path, status_lines)
        self._document.posed_skeleton = self._clone_skeleton(self._document.skeleton)
        self._document.autoposing = self._autoposing_service.build_rig(self._document.skeleton)
        self._base_status_message = " | ".join(status_lines)
        self._refresh_solver_state()
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
        self._autoposing_service.set_mode(self._document.autoposing, normalized_mode)
        self._base_status_message = f"Edit mode: {normalized_mode}"
        self._update_status_message()
        self.viewportPoseChanged.emit()
        self.documentChanged.emit()
        self.autoposingChanged.emit()

    @Slot(str)
    def selectAutoPosingController(self, controller_id: str) -> None:
        self._autoposing_service.select_controller(self._document.autoposing, controller_id or None)
        self.viewportPoseChanged.emit()
        self.autoposingChanged.emit()

    @Slot(str)
    def beginAutoPosingDrag(self, controller_id: str) -> None:
        self._drag_active = True
        self._autoposing_service.select_controller(self._document.autoposing, controller_id or None)
        self.viewportPoseChanged.emit()
        self.autoposingChanged.emit()

    @Slot(str, float, float, float)
    def previewAutoPosingController(self, controller_id: str, x: float, y: float, z: float) -> None:
        moved = self._autoposing_service.move_controller(self._document.autoposing, controller_id, (x, y, z))
        if not moved:
            return
        self._schedule_drag_preview()

    @Slot(str, float, float, float)
    def moveAutoPosingController(self, controller_id: str, x: float, y: float, z: float) -> None:
        self.previewAutoPosingController(controller_id, x, y, z)

    @Slot(str)
    def endAutoPosingDrag(self, controller_id: str) -> None:
        self._drag_active = False
        self._base_status_message = f"AutoPosing moved: {controller_id}"
        self._flush_drag_preview()
        self._update_status_message()
        self.viewportPoseChanged.emit()
        self.documentChanged.emit()
        self.autoposingChanged.emit()

    @Slot()
    def toggleSelectedAutoPosingFixed(self) -> None:
        controller = self._selected_autoposing_controller()
        if controller is None:
            return
        fixed = self._autoposing_service.toggle_fixed(
            self._document.autoposing,
            controller.identifier,
            controller.target.solved_world_position,
        )
        self._base_status_message = f"{controller.name} {'fixed' if fixed else 'unfixed'}"
        self._refresh_solver_state()
        self.documentChanged.emit()
        self.autoposingChanged.emit()

    @Slot()
    def resetSelectedAutoPosingController(self) -> None:
        controller = self._selected_autoposing_controller()
        if controller is None:
            return
        reset = self._autoposing_service.reset_controller(self._document.autoposing, controller.identifier)
        if not reset:
            return
        self._base_status_message = f"AutoPosing reset: {controller.name}"
        self._refresh_solver_state()
        self.documentChanged.emit()
        self.autoposingChanged.emit()

    @Slot()
    def toggleSelectedAutoPosingLock(self) -> None:
        self.toggleSelectedAutoPosingFixed()

    @Slot(QObject)
    def bindRuntimeModel(self, runtime_model: QObject) -> None:
        self._runtime_pose_writer.bind_runtime_model(runtime_model)
        self._runtime_pose_writer.apply_retarget_pose(self._document.retarget_pose)
        self._refresh_posed_skeleton()
        self.viewportPoseChanged.emit()
        self.documentChanged.emit()
        self.autoposingChanged.emit()

    @Slot()
    def clearRuntimeModel(self) -> None:
        self._runtime_pose_writer.clear()
        self._refresh_posed_skeleton()
        self.viewportPoseChanged.emit()
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
    statusMessage = Property(str, _status_message, notify=viewportPoseChanged)
    currentEditMode = Property(str, _current_edit_mode, notify=autoposingChanged)
    isAutoPosingMode = Property(bool, _is_autoposing_mode, notify=autoposingChanged)
    skeletonJoints = Property("QVariantList", _skeleton_joints_data, notify=documentChanged)
    skeletonBones = Property("QVariantList", _skeleton_bones_data, notify=documentChanged)
    autoPosingPreviewJoints = Property("QVariantList", _autoposing_preview_joints_data, notify=viewportPoseChanged)
    autoPosingPreviewBones = Property("QVariantList", _autoposing_preview_bones_data, notify=viewportPoseChanged)
    autoPosingControllers = Property("QVariantList", _autoposing_controllers_data, notify=viewportPoseChanged)
    autoPosingSegments = Property("QVariantList", _autoposing_segments_data, notify=viewportPoseChanged)
    selectedAutoPosingControllerName = Property(str, _selected_autoposing_controller_name, notify=autoposingChanged)
    selectedAutoPosingControllerType = Property(str, _selected_autoposing_controller_type, notify=autoposingChanged)
    selectedAutoPosingControllerJointName = Property(str, _selected_autoposing_controller_joint_name, notify=autoposingChanged)
    selectedAutoPosingControllerStatus = Property(str, _selected_autoposing_controller_status, notify=viewportPoseChanged)
    selectedAutoPosingControllerPosition = Property(str, _selected_autoposing_controller_position, notify=viewportPoseChanged)
    availableModels = Property("QVariantList", _available_models_data, notify=modelsChanged)
