from __future__ import annotations

import sys


def main() -> int:
    try:
        from PySide6.QtCore import QUrl  # noqa: F401
        from PySide6.QtGui import QGuiApplication  # noqa: F401
        from PySide6.QtQml import QQmlEngine  # noqa: F401
    except Exception as exc:
        print(f"Qt import failed: {type(exc).__name__}: {exc}")
        return 1

    print("Qt import ok")
    return 0


if __name__ == "__main__":
    sys.exit(main())
