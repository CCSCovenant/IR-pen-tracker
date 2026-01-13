import sys
import os
from PyQt5.QtWidgets import QApplication

def _ensure_src_on_path():
    current_dir = os.path.dirname(os.path.abspath(__file__))
    src_dir = os.path.abspath(os.path.join(current_dir, ".."))
    if src_dir not in sys.path:
        sys.path.append(src_dir)

def main():
    _ensure_src_on_path()
    from ir_pen_tracker.ui.main_window import MainWindow

    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
