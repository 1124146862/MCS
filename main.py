import os
import sys

from PySide6.QtCore import QUrl
from PySide6.QtGui import QGuiApplication
from PySide6.QtQml import QQmlApplicationEngine

from app.document.document_manager import DocumentManager
from infra.io.paths import PROJECT_ROOT, UI_DIR
from ui.viewmodels.document_view_model import DocumentViewModel


def main() -> int:
    os.environ.setdefault("QT_QUICK_CONTROLS_STYLE", "Basic")

    app = QGuiApplication(sys.argv)
    engine = QQmlApplicationEngine()
    engine.addImportPath(str(PROJECT_ROOT))

    document_manager = DocumentManager()
    startup_document = document_manager.open_startup_document()
    document_view_model = DocumentViewModel(startup_document)

    engine.setInitialProperties({"documentViewModel": document_view_model})
    engine.load(QUrl.fromLocalFile(str(UI_DIR / "Main.qml")))

    if not engine.rootObjects():
        return -1

    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
