import re
from pathlib import Path

from PySide6.QtCore import QObject, Property, QUrl, Signal, Slot

from app.autoposing.service import AutoPosingService
from app.document.models import DocumentModel
from core.rig.models import RigModel
from infra.importers.runtime_asset_importer import (
    discover_available_models,
    import_fbx_to_runtime_assets,
    import_model_into_project,
)
from infra.importers.skeleton_extractor import extract_skeleton_from_generated_component


class DocumentViewModel(QObject):
    documentChanged = Signal()
    modelsChanged = Signal()
    autoposingChanged = Signal()

    _AUTOPOSING_SEGMENT_LAYOUT = (
        ("pelvis_main", "chest_secondary"),
        ("chest_secondary", "head_main"),
        ("chest_secondary", "elbow_l_secondary"),
        ("elbow_l_secondary", "wrist_l_main"),
        ("chest_secondary", "elbow_r_secondary"),
        ("elbow_r_secondary", "wrist_r_main"),
        ("pelvis_main", "knee_l_secondary"),
        ("knee_l_secondary", "ankle_l_main"),
        ("pelvis_main", "knee_r_secondary"),
        ("knee_r_secondary", "ankle_r_main"),
        ("pelvis_main", "pelvis_dir"),
        ("chest_secondary", "chest_dir"),
        ("head_main", "head_dir"),
    )

    def __init__(self, document: DocumentModel):
        super().__init__()
        self._document = document
        self._available_models = list(document.available_model_paths)
        self._autoposing_service = AutoPosingService()

    @staticmethod
    def _display_name_for(path: Path) -> str:
        words = re.sub(r"([a-zA-Z])(\d)", r"\1 \2", path.stem.replace("_", " ")).split()
        return " ".join(word.capitalize() for word in words) if words else path.stem

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

    def _proxy_rig(self) -> RigModel:
        return self._document.proxy_rig

    def _current_edit_mode(self) -> str:
        return self._document.autoposing.current_mode

    def _is_autoposing_mode(self) -> bool:
        return self._document.autoposing.current_mode == "autoposing"

    def _skeleton_joints_data(self) -> list[dict[str, str | float | bool]]:
        return [
            {
                "name": bone.name,
                "parentName": bone.parent_name or "",
                "x": bone.world_position[0],
                "y": bone.world_position[1],
                "z": bone.world_position[2],
                "isTerminal": not any(child.parent_name == bone.name for child in self._document.skeleton.bones),
            }
            for bone in self._document.skeleton.bones
        ]

    def _skeleton_bones_data(self) -> list[dict[str, str | float]]:
        by_name = {bone.name: bone for bone in self._document.skeleton.bones}
        bone_segments: list[dict[str, str | float]] = []

        for bone in self._document.skeleton.bones:
            if not bone.parent_name:
                continue
            parent = by_name.get(bone.parent_name)
            if parent is None:
                continue

            bone_segments.append(
                {
                    "name": bone.name,
                    "parentName": bone.parent_name,
                    "startX": parent.world_position[0],
                    "startY": parent.world_position[1],
                    "startZ": parent.world_position[2],
                    "endX": bone.world_position[0],
                    "endY": bone.world_position[1],
                    "endZ": bone.world_position[2],
                }
            )

        return bone_segments

    def _autoposing_controller_positions(self) -> dict[str, tuple[float, float, float]]:
        return self._autoposing_service.solve_controller_positions(
            self._document.autoposing,
            self._proxy_rig(),
        )

    def _autoposing_preview_joints_data(self) -> list[dict[str, str | float | bool]]:
        proxy_rig = self._proxy_rig()
        return [
            {
                "name": joint.name,
                "parentName": joint.parent_name or "",
                "x": joint.world_position[0],
                "y": joint.world_position[1],
                "z": joint.world_position[2],
                "isTerminal": not any(child.parent_name == joint.name for child in proxy_rig.joints),
            }
            for joint in proxy_rig.joints
        ]

    def _autoposing_preview_bones_data(self) -> list[dict[str, str | float]]:
        proxy_rig = self._proxy_rig()
        by_name = {joint.name: joint for joint in proxy_rig.joints}
        segments: list[dict[str, str | float]] = []

        for joint in proxy_rig.joints:
            if not joint.parent_name:
                continue
            parent = by_name.get(joint.parent_name)
            if parent is None:
                continue

            segments.append(
                {
                    "name": joint.name,
                    "parentName": joint.parent_name,
                    "startX": parent.world_position[0],
                    "startY": parent.world_position[1],
                    "startZ": parent.world_position[2],
                    "endX": joint.world_position[0],
                    "endY": joint.world_position[1],
                    "endZ": joint.world_position[2],
                }
            )

        return segments

    def _autoposing_controllers_data(self) -> list[dict[str, str | float | bool]]:
        display_positions = self._autoposing_controller_positions()
        return [
            {
                "id": controller.identifier,
                "name": controller.name,
                "controllerType": controller.controller_type,
                "jointName": controller.joint_name,
                "x": display_positions.get(controller.identifier, controller.world_position)[0],
                "y": display_positions.get(controller.identifier, controller.world_position)[1],
                "z": display_positions.get(controller.identifier, controller.world_position)[2],
                "active": controller.active,
                "fixed": controller.fixed,
                "alwaysActive": controller.always_active,
                "state": self._autoposing_service.controller_state(controller),
                "selected": controller.selected,
                "visible": controller.visible,
            }
            for controller in self._document.autoposing.controllers
            if controller.visible
        ]

    def _autoposing_segments_data(self) -> list[dict[str, str | float]]:
        by_id = {controller.identifier: controller for controller in self._document.autoposing.controllers}
        display_positions = self._autoposing_controller_positions()
        segments: list[dict[str, str | float]] = []

        for start_id, end_id in self._AUTOPOSING_SEGMENT_LAYOUT:
            start = by_id.get(start_id)
            end = by_id.get(end_id)
            if start is None or end is None or not start.visible or not end.visible:
                continue
            start_position = display_positions.get(start_id, start.world_position)
            end_position = display_positions.get(end_id, end.world_position)
            segments.append(
                {
                    "startId": start_id,
                    "endId": end_id,
                    "startX": start_position[0],
                    "startY": start_position[1],
                    "startZ": start_position[2],
                    "endX": end_position[0],
                    "endY": end_position[1],
                    "endZ": end_position[2],
                }
            )

        return segments

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
        return self._autoposing_service.controller_state(controller)

    def _selected_autoposing_controller_position(self) -> str:
        controller = self._selected_autoposing_controller()
        if controller is None:
            return ""
        x, y, z = controller.world_position
        return f"{x:.2f}, {y:.2f}, {z:.2f}"

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

    def _refresh_proxy_rig(self) -> None:
        self._document.proxy_rig = self._autoposing_service.solve_proxy_rig(
            self._document.autoposing,
            self._document.skeleton,
        )

    def _apply_asset_state(
        self,
        source_model: Path | None,
        generated_component_path: Path | None,
        status_lines: list[str],
    ) -> None:
        self._document.source_model_path = source_model
        self._document.generated_component_path = generated_component_path
        self._document.skeleton = extract_skeleton_from_generated_component(generated_component_path, status_lines)
        self._document.autoposing = self._autoposing_service.build_rig(self._document.skeleton)
        self._refresh_proxy_rig()
        self._document.status_message = " | ".join(status_lines)
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
        self._document.status_message = f"Edit mode: {normalized_mode}"
        self.documentChanged.emit()
        self.autoposingChanged.emit()

    @Slot(str)
    def selectAutoPosingController(self, controller_id: str) -> None:
        self._autoposing_service.select_controller(self._document.autoposing, controller_id or None)
        self.autoposingChanged.emit()

    @Slot(str, float, float, float)
    def moveAutoPosingController(self, controller_id: str, x: float, y: float, z: float) -> None:
        moved = self._autoposing_service.move_controller(self._document.autoposing, controller_id, (x, y, z))
        if not moved:
            return

        self._refresh_proxy_rig()
        self._document.status_message = f"AutoPosing moved: {controller_id}"
        self.documentChanged.emit()
        self.autoposingChanged.emit()

    @Slot()
    def toggleSelectedAutoPosingFixed(self) -> None:
        controller = self._selected_autoposing_controller()
        if controller is None:
            return

        display_positions = self._autoposing_controller_positions()
        fixed = self._autoposing_service.toggle_fixed(
            self._document.autoposing,
            controller.identifier,
            display_positions.get(controller.identifier),
        )
        self._refresh_proxy_rig()
        self._document.status_message = f"{controller.name} {'fixed' if fixed else 'unfixed'}"
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

        self._refresh_proxy_rig()
        self._document.status_message = f"AutoPosing reset: {controller.name}"
        self.documentChanged.emit()
        self.autoposingChanged.emit()

    @Slot()
    def toggleSelectedAutoPosingLock(self) -> None:
        self.toggleSelectedAutoPosingFixed()

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

    documentName = Property(str, _document_name, constant=True)
    sourceModelName = Property(str, _source_model_name, notify=documentChanged)
    sourceModelPath = Property(str, _source_model_path, notify=documentChanged)
    currentModelFileName = Property(str, _current_model_file_name, notify=documentChanged)
    currentModelDisplayName = Property(str, _current_model_display_name, notify=documentChanged)
    generatedComponentUrl = Property(QUrl, _generated_component_url, notify=documentChanged)
    hasGeneratedModel = Property(bool, _has_generated_model, notify=documentChanged)
    statusMessage = Property(str, _status_message, notify=documentChanged)
    currentEditMode = Property(str, _current_edit_mode, notify=autoposingChanged)
    isAutoPosingMode = Property(bool, _is_autoposing_mode, notify=autoposingChanged)
    skeletonJoints = Property("QVariantList", _skeleton_joints_data, notify=documentChanged)
    skeletonBones = Property("QVariantList", _skeleton_bones_data, notify=documentChanged)
    autoPosingPreviewJoints = Property("QVariantList", _autoposing_preview_joints_data, notify=autoposingChanged)
    autoPosingPreviewBones = Property("QVariantList", _autoposing_preview_bones_data, notify=autoposingChanged)
    autoPosingControllers = Property("QVariantList", _autoposing_controllers_data, notify=autoposingChanged)
    autoPosingSegments = Property("QVariantList", _autoposing_segments_data, notify=autoposingChanged)
    selectedAutoPosingControllerName = Property(str, _selected_autoposing_controller_name, notify=autoposingChanged)
    selectedAutoPosingControllerType = Property(str, _selected_autoposing_controller_type, notify=autoposingChanged)
    selectedAutoPosingControllerJointName = Property(str, _selected_autoposing_controller_joint_name, notify=autoposingChanged)
    selectedAutoPosingControllerStatus = Property(str, _selected_autoposing_controller_status, notify=autoposingChanged)
    selectedAutoPosingControllerPosition = Property(str, _selected_autoposing_controller_position, notify=autoposingChanged)
    availableModels = Property("QVariantList", _available_models_data, notify=modelsChanged)
