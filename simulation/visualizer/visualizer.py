import sys


def main():
    try:
        from visualizer_qt import main as qt_main
    except ModuleNotFoundError as exc:
        if exc.name == "PySide6":
            print("缺少 PySide6，请先执行：python -m pip install PySide6")
            return 1
        raise
    return qt_main()


if __name__ == "__main__":
    sys.exit(main())
