#!/usr/bin/env python3
"""PiCam Ground Control — Entry Point"""

import sys
import os

# Force low-latency FFmpeg flags
os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "fflags;nobuffer|flags;low_delay|probesize;32|analyzeduration;0"

from PyQt6.QtWidgets import QApplication
from main_window import MainWindow

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())