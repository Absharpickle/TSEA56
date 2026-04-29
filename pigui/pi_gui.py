import sys
import cv2
import numpy as np
import socket
import struct
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QPushButton, QComboBox, QFrame)
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

# --- NEW: Telemetry Listener Thread ---
class TelemetryThread(QThread):
    telemetry_signal = pyqtSignal(dict)

    def __init__(self, sock):
        super().__init__()
        self.sock = sock
        self.running = True

    def run(self):
        # Set a small timeout so the thread can gracefully exit if needed
        self.sock.settimeout(1.0)
        
        while self.running:
            try:
                data, addr = self.sock.recvfrom(1024)
                # Check if it's our 9-byte telemetry packet (Starts with 0x06, Ends with 0xFF)
                if len(data) == 9 and data[0] == 0x06 and data[8] == 0xFF:
                    # Unpack 9 unsigned bytes
                    unpacked = struct.unpack('9B', data)
                    
                    # Create a dictionary of the received data
                    telemetry_data = {
                        'phase': unpacked[1],
                        'action': chr(unpacked[2]), # Convert ASCII code back to character
                        'next_action': chr(unpacked[3]),
                        'line_var': unpacked[4],
                        'gyro1': unpacked[5],
                        'gyro2': unpacked[6],
                        'flags': unpacked[7]
                    }
                    self.telemetry_signal.emit(telemetry_data)
            except socket.timeout:
                continue
            except Exception as e:
                print(f"[!] Telemetry error: {e}") # Ignore other socket errors during normal operation
                # DEBUG
                print(f"Received {len(data)} bytes: {data.hex()}")
    def stop(self):
        self.running = False

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PiCam Ground Control")
        self.setStyleSheet("QMainWindow { background-color: #2c3e50; color: white; }")
        
        # Ensure the main window can capture keyboard inputs
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

        # Main Layout
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QVBoxLayout(self.central_widget)

        # Video Box
        self.image_label = QLabel(self)
        self.image_label.setFixedSize(640, 480)
        self.image_label.setStyleSheet("border: 3px solid #34495e; background-color: black;")
        self.layout.addWidget(self.image_label, alignment=Qt.AlignmentFlag.AlignCenter)

        # --- TELEMETRY SETUP ---
        self.pi_ip = "10.42.0.1"
        self.pi_port = 5001
        self.control_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # We must bind our socket to listen for returning packets on whatever port the OS assigns
        self.control_sock.bind(("0.0.0.0", 0)) 

        # --- CONTROL PANEL ---
        self.control_layout = QHBoxLayout()
        self.layout.addLayout(self.control_layout)

        # 1. State Selector
        self.state_combo = QComboBox()
        self.state_combo.setStyleSheet("background-color: white; color: black; padding: 5px; font-size: 14px;")
        self.state_combo.addItem("1: (Auto, Auto)", 0)
        self.state_combo.addItem("2: (Auto, Manual)", 1)
        self.state_combo.addItem("3: (Manual, Auto)", 2)
        self.state_combo.addItem("4: (Manual, Manual)", 3)
        self.state_combo.setCurrentIndex(3) # Default to 3
        # Return focus to main window after clicking so keyboard works
        self.state_combo.currentIndexChanged.connect(lambda: self.setFocus()) 
        self.control_layout.addWidget(QLabel("State:"))
        self.control_layout.addWidget(self.state_combo)

        # 2. Target Selector
        self.target_combo = QComboBox()
        self.target_combo.setStyleSheet("background-color: white; color: black; padding: 5px; font-size: 14px;")
        self.target_combo.addItem("Wheel", "wheel")
        self.target_combo.addItem("Arm", "arm")
        self.target_combo.currentIndexChanged.connect(lambda: self.setFocus())
        self.control_layout.addWidget(QLabel("Target:"))
        self.control_layout.addWidget(self.target_combo)

        # Instructions Label
        self.inst_label = QLabel(
            "HOTKEYS -> STATE: [1-4] | TARGET: [W]heel, [A]rm\n"
            "WHEEL: Arrows (Move), 'S' (Stop), 'E' (CW), 'O' (CCW)  |  ARM: 'V' (Left), 'H' (Right)"
        )
        self.inst_label.setStyleSheet("color: #bdc3c7; font-size: 13px; font-weight: bold;")
        self.layout.addWidget(self.inst_label, alignment=Qt.AlignmentFlag.AlignCenter)

        # --- NEW: LIVE TELEMETRY DASHBOARD ---
        self.dashboard_frame = QFrame()
        self.dashboard_frame.setStyleSheet("QFrame { background-color: #34495e; border-radius: 5px; margin-top: 10px; }")
        self.dashboard_layout = QHBoxLayout(self.dashboard_frame)
        
        self.lbl_phase = QLabel("Phase: IDLE")
        self.lbl_action = QLabel("Last Action: -")
        self.lbl_next_action = QLabel("Next Action: -")
        self.lbl_line = QLabel("Line: 0")
        self.lbl_gyro = QLabel("Gyro: (0, 0)")
        self.lbl_flags = QLabel("Flags: 0")
        
        for lbl in [self.lbl_phase, self.lbl_action, self.lbl_next_action, self.lbl_line, self.lbl_gyro, self.lbl_flags]:
            lbl.setStyleSheet("font-size: 14px; font-weight: bold; color: #ecf0f1; padding: 5px;")
            self.dashboard_layout.addWidget(lbl)
            
        self.layout.addWidget(self.dashboard_frame)

        # Start Video Thread
        self.video_thread = VideoThread()
        self.video_thread.change_pixmap_signal.connect(self.update_image)
        self.video_thread.start()

        # Start Telemetry Thread
        self.telemetry_thread = TelemetryThread(self.control_sock)
        self.telemetry_thread.telemetry_signal.connect(self.update_telemetry_dashboard)
        self.telemetry_thread.start()

        # Phase names mapping
        self.phase_names = {0: "IDLE", 1: "TO ITEM", 2: "PICKUP", 3: "TO HOME"}

    # --- NEW: UPDATE TELEMETRY UI METHOD ---
    def update_telemetry_dashboard(self, data):
        phase_str = self.phase_names.get(data['phase'], "UNKNOWN")
        self.lbl_phase.setText(f"Phase: {phase_str}")
        self.lbl_action.setText(f"Action: '{data['action']}'")
        self.lbl_next_action.setText(f"Next Action: '{data['next_action']}'")
        self.lbl_line.setText(f"Line: {data['line_var']}")
        self.lbl_gyro.setText(f"Gyro: ({data['gyro1']}, {data['gyro2']})")
        self.lbl_flags.setText(f"Flags: {data['flags']}")

    # --- KEYBOARD LISTENER ---
    def keyPressEvent(self, event):
        key = event.key()
        action_char = None

        # 1. --- HOTKEYS FOR STATE SELECTION ---
        if key == Qt.Key.Key_1:
            self.state_combo.setCurrentIndex(0)
            print("Hot-swapped State: 1 (Auto, Auto)")
            return
        elif key == Qt.Key.Key_2:
            self.state_combo.setCurrentIndex(1)
            print("Hot-swapped State: 2 (Auto, Manual)")
            return
        elif key == Qt.Key.Key_3:
            self.state_combo.setCurrentIndex(2)
            print("Hot-swapped State: 3 (Manual, Auto)")
            return
        elif key == Qt.Key.Key_4:
            self.state_combo.setCurrentIndex(3)
            print("Hot-swapped State: 4 (Manual, Manual)")
            return

        # 2. --- HOTKEYS FOR TARGET SELECTION ---
        elif key == Qt.Key.Key_W:
            self.target_combo.setCurrentIndex(0)
            print("Hot-swapped Target: Wheel")
            return
        elif key == Qt.Key.Key_A:
            self.target_combo.setCurrentIndex(1)
            print("Hot-swapped Target: Arm")
            return

        # 3. --- ACTION COMMANDS ---
        # Get the currently selected target to filter inputs
        target = self.target_combo.currentData()
        
        if target == "wheel":
            if key == Qt.Key.Key_Up: action_char = 'f'
            elif key == Qt.Key.Key_Down: action_char = 'b'
            elif key == Qt.Key.Key_Right: action_char = 'r'
            elif key == Qt.Key.Key_Left: action_char = 'l'
            elif key == Qt.Key.Key_S: action_char = 's'
            elif key == Qt.Key.Key_E: action_char = 'e'
            elif key == Qt.Key.Key_O: action_char = 'o'
            elif key == Qt.Key.Key_U: action_char = 'u' # Lagt till U-sväng
            
        elif target == "arm":
            if key == Qt.Key.Key_V: action_char = 'v'
            elif key == Qt.Key.Key_H: action_char = 'h'

        # If a valid action key was pressed, send the packet
        if action_char:
            self.send_packet(action_char)

    def send_packet(self, action_char):
        # Read the current selections from the dropdowns
        current_state = self.state_combo.currentData()
        current_target = self.target_combo.currentData()

        # Map target string to byte (0x00 for wheel, 0x01 for arm)
        target_byte = 0x00 if current_target == "wheel" else 0x01
        
        # Convert character to ASCII byte
        action_byte = ord(str(action_char)[0])  

        # Pack the 8 bytes
        packet = struct.pack('BBBBBBBB',
                             0x05,           # Start byte
                             current_state,  # State (1, 2, 3, or 4)
                             target_byte,    # Cmd (0 or 1)
                             action_byte,    # Action (ASCII int)
                             0x00,           # Line var reserved
                             0x00,           # Gyro 1 reserved
                             0x00,           # Gyro 2 reserved
                             0xFF)           # End byte
        
        try:
            self.control_sock.sendto(packet, (self.pi_ip, self.pi_port))
            # print(f"[{current_target.upper()}] Sent '{action_char}' (Hex: {packet.hex().upper()})")
        except Exception as e:
            print(f"Error sending packet: {e}")

    # Video display methods
    def update_image(self, cv_img):
        qt_img = self.convert_cv_qt(cv_img)
        self.image_label.setPixmap(qt_img)

    def convert_cv_qt(self, cv_img):
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        convert_to_Qt_format = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
        return QPixmap.fromImage(convert_to_Qt_format)

    def closeEvent(self, event):
        # Cleanly stop the thread when closing the application
        self.telemetry_thread.stop()
        self.telemetry_thread.wait()
        super().closeEvent(event)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())