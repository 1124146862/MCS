import shutil
import subprocess
import sys
import tempfile
from dataclasses import dataclass
from hashlib import sha256
from pathlib import Path

from infra.io.paths import DEFAULT_SOURCE_MODEL_NAMES, DESKTOP_DIR, GENERATED_ASSETS_DIR, PROJECT_ROOT, SOURCE_ASSETS_DIR


@dataclass(slots=True)
class RuntimeAssetState:
    available_model_paths: list[Path]
    source_model_path: Path | None
    generated_component_path: Path | None
    status_message: str


def ensure_asset_directories() -> None:
    SOURCE_ASSETS_DIR.mkdir(parents=True, exist_ok=True)
    GENERATED_ASSETS_DIR.mkdir(parents=True, exist_ok=True)


def file_sha256(path: Path) -> str:
    digest = sha256()
    with path.open("rb") as file:
        for chunk in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def sort_source_models(model_paths: list[Path]) -> list[Path]:
    priority = {name: index for index, name in enumerate(DEFAULT_SOURCE_MODEL_NAMES)}
    return sorted(model_paths, key=lambda path: (priority.get(path.name.lower(), len(priority)), path.name.lower()))


def desktop_model_path(file_name: str) -> Path:
    return DESKTOP_DIR / file_name


def sync_named_source_model(file_name: str, status_lines: list[str]) -> Path | None:
    source_model = SOURCE_ASSETS_DIR / file_name
    desktop_model = desktop_model_path(file_name)

    if source_model.exists():
        if desktop_model.exists():
            try:
                if file_sha256(source_model) == file_sha256(desktop_model):
                    desktop_model.unlink()
                    status_lines.append(f"Removed duplicate Desktop {file_name} after confirming project copy.")
                else:
                    status_lines.append(f"Desktop {file_name} differs from project copy. Keeping both files.")
            except OSError as exc:
                status_lines.append(f"Could not clean up Desktop {file_name}: {exc}")
        return source_model

    if desktop_model.exists():
        shutil.move(str(desktop_model), str(source_model))
        status_lines.append(f"Moved Desktop {file_name} into assets/source/")
        return source_model

    return source_model if source_model.exists() else None


def discover_available_models(status_lines: list[str]) -> list[Path]:
    for file_name in DEFAULT_SOURCE_MODEL_NAMES:
        sync_named_source_model(file_name, status_lines)

    available_models = [path for path in SOURCE_ASSETS_DIR.glob("*.fbx") if path.is_file()]
    available_models = sort_source_models(available_models)

    if not available_models:
        status_lines.append("No FBX model found in assets/source/.")

    return available_models


def find_balsam_executable() -> Path | None:
    candidates = [
        Path(sys.executable).with_name("pyside6-balsam.exe"),
        PROJECT_ROOT / ".qtcreator" / "Python_3_12_64_bit_venv" / "Scripts" / "pyside6-balsam.exe",
        Path(r"D:\Qt\6.11.1\mingw_64\bin\balsam.exe"),
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return None


def clear_directory_contents(directory: Path) -> None:
    if not directory.exists():
        return
    for item in directory.iterdir():
        if item.is_dir():
            shutil.rmtree(item)
        else:
            item.unlink()


def generated_assets_dir_for_source(source_model: Path) -> Path:
    return GENERATED_ASSETS_DIR / source_model.stem


def find_generated_component_path(output_directory: Path) -> Path | None:
    qml_files = sorted(path for path in output_directory.glob("*.qml") if path.name.lower() != "qmldir")
    return qml_files[0] if qml_files else None


def import_fbx_to_runtime_assets(source_model: Path, status_lines: list[str]) -> Path | None:
    balsam_executable = find_balsam_executable()
    if not balsam_executable:
        status_lines.append("Balsam not found. FBX source copied, runtime import skipped.")
        return None

    output_directory = generated_assets_dir_for_source(source_model)
    existing_component = find_generated_component_path(output_directory)
    if existing_component and existing_component.stat().st_mtime >= source_model.stat().st_mtime:
        status_lines.append(f"Using cached runtime asset for {source_model.name}: {existing_component.name}")
        return existing_component

    temp_root = Path(tempfile.gettempdir()) / "mcs_balsam_runtime"
    temp_source_dir = temp_root / "source"
    temp_output_dir = temp_root / "output"

    shutil.rmtree(temp_root, ignore_errors=True)
    temp_source_dir.mkdir(parents=True, exist_ok=True)
    temp_output_dir.mkdir(parents=True, exist_ok=True)
    temp_source_model = temp_source_dir / source_model.name
    shutil.copy2(source_model, temp_source_model)

    try:
        result = subprocess.run(
            [
                str(balsam_executable),
                "--outputPath",
                str(temp_output_dir),
                str(temp_source_model),
            ],
            capture_output=True,
            text=True,
            check=False,
        )
    except OSError as exc:
        status_lines.append(f"Failed to start Balsam: {exc}")
        return None

    generated_qml = sorted(path for path in temp_output_dir.glob("*.qml") if path.name.lower() != "qmldir")
    if result.returncode != 0 or not generated_qml:
        status_lines.append("FBX detected, but automatic Qt import did not produce runtime assets yet.")
        if result.stderr.strip():
            status_lines.append(result.stderr.strip().splitlines()[-1])
        return None

    output_directory.mkdir(parents=True, exist_ok=True)
    clear_directory_contents(output_directory)
    for item in temp_output_dir.iterdir():
        destination = output_directory / item.name
        if item.is_dir():
            shutil.copytree(item, destination, dirs_exist_ok=True)
        else:
            shutil.copy2(item, destination)

    component = find_generated_component_path(output_directory)
    if component:
        status_lines.append(f"Imported FBX runtime asset for {source_model.name}: {component.name}")
    return component


def make_unique_destination(destination: Path) -> Path:
    counter = 2
    candidate = destination
    while candidate.exists():
        candidate = destination.with_name(f"{destination.stem}_{counter}{destination.suffix}")
        counter += 1
    return candidate


def import_model_into_project(external_model: Path, status_lines: list[str]) -> Path | None:
    ensure_asset_directories()
    if not external_model.exists():
        status_lines.append(f"Selected model does not exist: {external_model}")
        return None

    if external_model.suffix.lower() != ".fbx":
        status_lines.append(f"Unsupported model format: {external_model.suffix}")
        return None

    if external_model.resolve().parent == SOURCE_ASSETS_DIR.resolve():
        status_lines.append(f"Model already in project: {external_model.name}")
        return external_model

    destination = SOURCE_ASSETS_DIR / external_model.name
    if destination.exists():
        try:
            if file_sha256(destination) == file_sha256(external_model):
                status_lines.append(f"Project already contains identical model: {destination.name}")
                return destination
        except OSError as exc:
            status_lines.append(f"Could not compare {external_model.name}: {exc}")

        destination = make_unique_destination(destination)

    shutil.copy2(external_model, destination)
    status_lines.append(f"Imported model into assets/source/: {destination.name}")
    return destination


def prepare_startup_asset_state() -> RuntimeAssetState:
    ensure_asset_directories()
    status_lines: list[str] = []
    available_models = discover_available_models(status_lines)
    source_model = available_models[0] if available_models else None
    generated_component = None

    if source_model is not None:
        status_lines.append(f"Startup model: {source_model.name}")
        generated_component = import_fbx_to_runtime_assets(source_model, status_lines)
        if generated_component is None:
            status_lines.append("Showing placeholder body until runtime asset import succeeds.")
    else:
        status_lines.append("Showing built-in placeholder.")

    return RuntimeAssetState(
        available_model_paths=available_models,
        source_model_path=source_model,
        generated_component_path=generated_component,
        status_message=" | ".join(status_lines),
    )
