# ------------------------------------------------------
# Markus Hellers, Joel Eberhardsson - 28 maj 2026 - V1.0
# ------------------------------------------------------

## Huvudfilen för GUI:t som startar applikationen och visar huvudfönstret ##

import sys
import os

os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "fflags;nobuffer|flags;low_delay|probesize;32|analyzeduration;0" # Flaggor för att minska latens vid videoströmning

from PyQt6.QtWidgets import QApplication
from main_window import MainWindow

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()   # Skapa huvudfönstret
    window.show()           # Visa huvudfönstret
    sys.exit(app.exec())    # Starta guins event-loop och avsluta när den stängs