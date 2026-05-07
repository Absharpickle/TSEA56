import cv2
import socket
from PyQt6.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
                             QLabel, QComboBox, QFrame, QGridLayout, QPushButton)
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QImage, QPixmap

from threads import VideoThread, TelemetryThread
from map_widget import MapFrame
from protocol import build_command_packet, build_item_list_packet

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PiCam Ground Control")
        self.pi_ip = "10.42.0.1"
        self.pi_port = 5001
        self.item_edges = []
        
        self.setStyleSheet("QMainWindow { background-color: #2c3e50; color: white; }")
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QVBoxLayout(self.central_widget)
        
        # Top Layout
        h_layout = QHBoxLayout()
        self.image_label = QLabel("Video...")
        self.image_label.setFixedSize(480, 360)
        self.map_frame = MapFrame(range(26)) # Rad 48: Skapar kartan med 26 noder
        h_layout.addWidget(self.image_label)
        h_layout.addWidget(self.map_frame)
        self.layout.addLayout(h_layout)

        # Dashboard
        self._setup_ui()

        # Threads
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.telemetry_thread = TelemetryThread(self.sock)
        self.telemetry_thread.telemetry_signal.connect(self.update_telemetry)
        self.telemetry_thread.route_signal.connect(self.map_frame.set_route)
        self.telemetry_thread.start()

    def _setup_ui(self):
        grid = QGridLayout()
        self.ir_label = QLabel("IR: --")
        self.hinder_label = QLabel("Väg: --")
        grid.addWidget(self.ir_label, 0, 0)
        grid.addWidget(self.hinder_label, 0, 1)
        self.layout.addLayout(grid)

    def update_telemetry(self, data):
        # Rad 104: Uppdaterar IR-värdet säkert från dictionaryt
        self.ir_label.setText(f"IR: {data.get('ir_distance', 'N/A')}")
        hinder = (data['flags'] & 0x10) >> 4
        self.hinder_label.setText("HINDER!" if hinder else "Väg: Klar")
        self.hinder_label.setStyleSheet("color: red;" if hinder else "color: green;")

    def update_image(self, img):
        rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        qimg = QImage(rgb.data, w, h, ch*w, QImage.Format.Format_RGB888)
        self.image_label.setPixmap(QPixmap.fromImage(qimg))
