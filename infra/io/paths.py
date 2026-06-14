from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
UI_DIR = PROJECT_ROOT / "ui"
ASSETS_DIR = PROJECT_ROOT / "assets"
SOURCE_ASSETS_DIR = ASSETS_DIR / "source"
GENERATED_ASSETS_DIR = ASSETS_DIR / "generated"
DESKTOP_DIR = Path.home() / "Desktop"
DESKTOP_MODEL = DESKTOP_DIR / "model.fbx"
DEFAULT_SOURCE_MODEL_NAMES = ("model.fbx", "model2.fbx")
