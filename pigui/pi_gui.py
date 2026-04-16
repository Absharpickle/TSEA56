import sys
import cv2
import numpy as np
import socket
import struct
from PyQt6.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QLabel, QPushButton
from PyQt6.QtCore import QThread, pyqtSignal, Qt
from PyQt6.QtGui import QImage, QPixmap
import os

# Force low-latency FFmpeg flags
os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "fflags;nobuffer|flags;low_delay|probesize;32|analyzeduration;0"

class VideoThread(QThread):
    change_pixmap_signal = pyqtSignal(np.ndarray)

    def run(self):
        # Connect to the Pi's UDP stream
        cap = cv2.VideoCapture("udp://0.0.0.0:5000", cv2.CAP_FFMPEG)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        while True:
            ret, frame = cap.read()
            if ret:
                self.change_pixmap_signal.emit(frame)

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PiCam Pro Interface")
        self.setStyleSheet("QMainWindow { background-color: #2c3e50; }")

        # Main Layout
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QVBoxLayout(self.central_widget)

        # Video Box
        self.image_label = QLabel(self)
        self.image_label.setFixedSize(640, 480)
        self.image_label.setStyleSheet("border: 3px solid #34495e; background-color: black;")
        self.layout.addWidget(self.image_label)

        # Snapshot Button
        self.btn = QPushButton("Capture Snapshot")
        self.btn.setStyleSheet("""
            QPushButton {
                background-color: #e74c3c;
                color: white;
                font-size: 16px;
                padding: 10px;
                border-radius: 5px;
            }
            QPushButton:hover { background-color: #c0392b; }
        """)
        self.btn.clicked.connect(self.take_snapshot)
        self.layout.addWidget(self.btn)

        # --- MOVED FROM take_snapshot TO HERE ---
        # 1. Telemetry Setup
        self.pi_ip = "10.42.0.1"
        self.pi_port = 5001
        self.control_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # Set your run state here (1, 2, 3, or 4 based on whiteboard)
        self.run_state = 4 # e.g., 4 = (manual, manual)

        # 2. Test Button
        self.btn_send = QPushButton("Send Test Command (Wheel, 'W')")
        self.btn_send.setStyleSheet("background-color: #27ae60; color: white; padding: 10px; font-size: 16px; border-radius: 5px;")
        
        # Notice we only pass target and action now, because state is fixed!
        self.btn_send.clicked.connect(lambda: self.send_packet("wheel", "W"))
        self.layout.addWidget(self.btn_send)
        # -----------------------------------------

        # Start Video Thread
        self.thread = VideoThread()
        self.thread.change_pixmap_signal.connect(self.update_image)
        self.thread.start()

    def update_image(self, cv_img):
        qt_img = self.convert_cv_qt(cv_img)
        self.image_label.setPixmap(qt_img)

    def convert_cv_qt(self, cv_img):
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        convert_to_Qt_format = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
        return QPixmap.fromImage(convert_to_Qt_format)

    def take_snapshot(self):
        # This function now ONLY takes snapshots
        if self.image_label.pixmap():
            self.image_label.pixmap().save("snapshot.png")
            print("Snapshot saved!")

    def send_packet(self, target_str, action_char):
        # Map target string to byte (0 for wheel, 1 for arm)
        target_byte = 0x00 if target_str == "wheel" else 0x01
        
        # Convert char to ASCII byte
        action_byte = ord(str(action_char)[0])  

        # Pack the 8 bytes using our pre-set self.run_state
        packet = struct.pack('BBBBBBBB',
                             0x05,             # Start byte
                             self.run_state,   # State (1, 2, 3, or 4)
                             target_byte,      # Cmd
                             action_byte,      # Action
                             0x00,             # Line var reserved
                             0x00,             # Gyro 1 reserved
                             0x00,             # Gyro 2 reserved
                             0xFF)             # End byte
        
        try:
            self.control_sock.sendto(packet, (self.pi_ip, self.pi_port))
            print(f"Sent packet: {packet.hex().upper()} to {self.pi_ip}:{self.pi_port}")
        except Exception as e:
            print(f"Error sending packet: {e}")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())