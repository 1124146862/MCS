import re
from pathlib import Path

from PySide6.QtCore import QObject, Property, QUrl, Signal, Slot

from app.document.models import DocumentModel
from infra.importers.runtime_asset_importer import (
    discover_available_models,
    import_fbx_to_runtime_assets,
    import_model_into_project,
)
from infra.importers.skeleton_extractor import extract_skeleton_from_generated_component


class DocumentViewModel(QObject):
    documentChanged = Signal()
    modelsChanged = Signal()

    def __init__(self, document: DocumentModel):
        super().__init__()
        self._document = document
        self._available_models = list(document.available_model_paths)

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
        self._document.source_model_path = source_model
        self._document.generated_component_path = generated_component_path
        self._document.skeleton = extract_skeleton_from_generated_component(generated_component_path, status_lines)
        self._document.status_message = " | ".join(status_lines)
        self.documentChanged.emit()
        self.modelsChanged.emit()

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
    skeletonJoints = Property("QVariantList", _skeleton_joints_data, notify=documentChanged)
    skeletonBones = Property("QVariantList", _skeleton_bones_data, notify=documentChanged)
    availableModels = Property("QVariantList", _available_models_data, notify=modelsChanged)
