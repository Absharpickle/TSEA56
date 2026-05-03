import sys
import cv2
import numpy as np
import socket
import struct
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QComboBox, QFrame, QGridLayout,
                             QSizePolicy)
from PyQt6.QtCore import QThread, pyqtSignal, Qt, QPoint
from PyQt6.QtGui import QImage, QPixmap, QPainter, QPen, QColor
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
        self.sock.settimeout(1.0)
        
        while self.running:
            try:
                data, addr = self.sock.recvfrom(1024)
                if len(data) == 9 and data[0] == 0x06 and data[8] == 0xFF:
                    unpacked = struct.unpack('9B', data)
                    
                    telemetry_data = {
                        'phase': unpacked[1],
                        'action': chr(unpacked[2]), 
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
                print(f"[!] Telemetry Error: {e}") 

    def stop(self):
        self.running = False

# --- Custom Map Frame with Line Drawing ---
class MapFrame(QFrame):
    def __init__(self, grid_nodes_ref, parent=None):
        super().__init__(parent)
        self.grid_nodes = grid_nodes_ref
        self.setStyleSheet("background-color: #1a252f; border: 2px solid #7f8c8d; border-radius: 5px;")
        self.setFixedSize(480, 480)

    def paintEvent(self, event):
        super().paintEvent(event)
        
        if not self.grid_nodes or len(self.grid_nodes) < 26:
            return

        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        
        pen = QPen(QColor("#2c3e50"), 6)
        pen.setCapStyle(Qt.PenCapStyle.RoundCap)
        painter.setPen(pen)

        def get_center(node_id):
            widget = self.grid_nodes[node_id]
            return widget.pos() + QPoint(widget.width() // 2, widget.height() // 2)

        for i in range(25):
            kol = i % 5
            if kol < 4:  
                p1 = get_center(i)
                p2 = get_center(i + 1)
                painter.drawLine(p1, p2)

        for i in range(25):
            rad = i // 5
            if rad < 4:  
                p1 = get_center(i)
                p2 = get_center(i + 5)
                painter.drawLine(p1, p2)

        p_start = get_center(25)
        p_0 = get_center(0)
        painter.drawLine(p_start, p_0)

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PiCam Ground Control")
        # Added generic QLabel styling to ensure white text everywhere by default
        self.setStyleSheet("""
            QMainWindow { background-color: #2c3e50; color: white; }
            QLabel { color: white; }
        """)
        
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QVBoxLayout(self.central_widget)

        self.top_hlayout = QHBoxLayout()
        self.layout.addLayout(self.top_hlayout)

        self.image_label = QLabel(self)
        self.image_label.setFixedSize(640, 480)
        self.image_label.setStyleSheet("border: 3px solid #34495e; background-color: black;")
        self.top_hlayout.addWidget(self.image_label, alignment=Qt.AlignmentFlag.AlignCenter)

        # --- MAP SETUP ---
        self.grid_nodes = {} 
        self.current_active_node = 25 
        
        self.map_frame = MapFrame(self.grid_nodes)
        self.map_layout = QGridLayout(self.map_frame)
        self.map_layout.setSpacing(15) 

        self.node_style_idle = """
            background-color: #95a5a6; 
            color: black; 
            font-weight: bold; 
            border-radius: 25px; 
            font-size: 16px;
        """
        
        self.node_style_active = """
            background-color: #e74c3c; 
            color: white; 
            font-weight: bold; 
            border-radius: 25px; 
            font-size: 18px;
            border: 3px solid #f1c40f;
        """

        self.lbl_start = QLabel("25")
        self.lbl_start.setFixedSize(50, 50)
        self.lbl_start.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_start.setStyleSheet(self.node_style_active) 
        self.map_layout.addWidget(self.lbl_start, 0, 0, alignment=Qt.AlignmentFlag.AlignCenter)
        self.grid_nodes[25] = self.lbl_start

        for i in range(25):
            rad = (i // 5) + 1 
            kol = i % 5
            lbl = QLabel(str(i))
            lbl.setFixedSize(50, 50)
            lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            lbl.setStyleSheet(self.node_style_idle)
            self.map_layout.addWidget(lbl, rad, kol, alignment=Qt.AlignmentFlag.AlignCenter)
            self.grid_nodes[i] = lbl

        self.top_hlayout.addWidget(self.map_frame, alignment=Qt.AlignmentFlag.AlignCenter)

        # --- TELEMETRY SETUP ---
        self.pi_ip = "192.168.1.50" # BYT TILL DIN PI:S IP
        self.pi_port = 5001
        self.control_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.control_sock.bind(("0.0.0.0", 0)) 

        # --- CONTROL PANEL ---
        # Stramat upp layouten för kontrollerna
        self.control_layout = QHBoxLayout()
        self.control_layout.setSpacing(10) # Minskar avståndet mellan widgets
        self.control_layout.setContentsMargins(50, 10, 50, 10) # Lägger till lite luft på sidorna
        self.layout.addLayout(self.control_layout)

        # Gemensam stil för kontroll-etiketterna
        control_label_style = "font-size: 16px; font-weight: bold; padding-right: 5px; color: #ecf0f1;"
        combo_style = "background-color: #ecf0f1; color: #2c3e50; padding: 5px 10px; font-size: 14px; font-weight: bold; border-radius: 3px;"

        # 1. State Selector
        lbl_state = QLabel("State:")
        lbl_state.setStyleSheet(control_label_style)
        # Sätt en fast storlek på etiketten så den inte sprider ut sig
        lbl_state.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed) 
        
        self.state_combo = QComboBox()
        self.state_combo.setStyleSheet(combo_style)
        self.state_combo.setMinimumWidth(200) # Gör rullgardinen lite bredare
        self.state_combo.addItem("1: (Auto, Auto)", 0)
        self.state_combo.addItem("2: (Auto, Manual)", 1)
        self.state_combo.addItem("3: (Manual, Auto)", 2)
        self.state_combo.addItem("4: (Manual, Manual)", 3)
        self.state_combo.setCurrentIndex(3) 
        self.state_combo.currentIndexChanged.connect(lambda: self.setFocus()) 
        
        self.control_layout.addWidget(lbl_state)
        self.control_layout.addWidget(self.state_combo)
        
        # Lägg till ett fjäder-element (spacer) i mitten för att trycka isär State och Target lite snyggt
        self.control_layout.addStretch()

        # 2. Target Selector
        lbl_target = QLabel("Target:")
        lbl_target.setStyleSheet(control_label_style)
        lbl_target.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)

        self.target_combo = QComboBox()
        self.target_combo.setStyleSheet(combo_style)
        self.target_combo.setMinimumWidth(150)
        self.target_combo.addItem("Wheel", "wheel")
        self.target_combo.addItem("Arm", "arm")
        self.target_combo.currentIndexChanged.connect(lambda: self.setFocus())
        
        self.control_layout.addWidget(lbl_target)
        self.control_layout.addWidget(self.target_combo)

        # Instructions Label
        self.inst_label = QLabel(
            "HOTKEYS -> STATE: [1-4] | TARGET: [W]heel, [A]rm\n"
            "WHEEL: Arrows (Move), 'S' (Stop), 'E' (CW), 'O' (CCW)  |  ARM: 'V' (Left), 'H' (Right)"
        )
        self.inst_label.setStyleSheet("color: #bdc3c7; font-size: 14px; font-weight: bold; margin-top: 10px;")
        self.layout.addWidget(self.inst_label, alignment=Qt.AlignmentFlag.AlignCenter)

        # --- LIVE TELEMETRY DASHBOARD ---
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

        # Start Threads
        self.video_thread = VideoThread()
        self.video_thread.change_pixmap_signal.connect(self.update_image)
        self.video_thread.start()

        self.telemetry_thread = TelemetryThread(self.control_sock)
        self.telemetry_thread.telemetry_signal.connect(self.update_telemetry_dashboard)
        self.telemetry_thread.start()

        self.phase_names = {0: "IDLE", 1: "TO ITEM", 2: "PICKUP", 3: "TO HOME"}

    def update_telemetry_dashboard(self, data):
        phase_str = self.phase_names.get(data['phase'], "UNKNOWN")
        self.lbl_phase.setText(f"Phase: {phase_str}")
        self.lbl_action.setText(f"Action: '{data['action']}'")
        self.lbl_next_action.setText(f"Next Action: '{data['next_action']}'")
        self.lbl_line.setText(f"Line: {data['line_var']}")
        self.lbl_gyro.setText(f"Gyro: ({data['gyro1']}, {data['gyro2']})")
        self.lbl_flags.setText(f"Flags: {data['flags']}")

        if data['phase'] == 0 and self.current_active_node != 25:
            self.set_active_node(25)


    def set_active_node(self, node_id):
        if self.current_active_node in self.grid_nodes:
            self.grid_nodes[self.current_active_node].setStyleSheet(self.node_style_idle)
        
        if node_id in self.grid_nodes:
            self.grid_nodes[node_id].setStyleSheet(self.node_style_active)
            self.current_active_node = node_id

    # --- KEYBOARD LISTENER ---
    def keyPressEvent(self, event):
        key = event.key()
        action_char = None

        if key == Qt.Key.Key_1:
            self.state_combo.setCurrentIndex(0)
            return
        elif key == Qt.Key.Key_2:
            self.state_combo.setCurrentIndex(1)
            return
        elif key == Qt.Key.Key_3:
            self.state_combo.setCurrentIndex(2)
            return
        elif key == Qt.Key.Key_4:
            self.state_combo.setCurrentIndex(3)
            return

        elif key == Qt.Key.Key_W:
            self.target_combo.setCurrentIndex(0)
            return
        elif key == Qt.Key.Key_A:
            self.target_combo.setCurrentIndex(1)
            return

        target = self.target_combo.currentData()
        
        if target == "wheel":
            if key == Qt.Key.Key_Up: action_char = 'f'
            elif key == Qt.Key.Key_Down: action_char = 'b'
            elif key == Qt.Key.Key_Right: action_char = 'r'
            elif key == Qt.Key.Key_Left: action_char = 'l'
            elif key == Qt.Key.Key_S: action_char = 's'
            elif key == Qt.Key.Key_E: action_char = 'e'
            elif key == Qt.Key.Key_O: action_char = 'o'
            elif key == Qt.Key.Key_U: action_char = 'u' 
            
        elif target == "arm":
            if key == Qt.Key.Key_V: action_char = 'v'
            elif key == Qt.Key.Key_H: action_char = 'h'

        if action_char:
            self.send_packet(action_char)

    def send_packet(self, action_char):
        current_state = self.state_combo.currentData()
        current_target = self.target_combo.currentData()

        target_byte = 0x00 if current_target == "wheel" else 0x01
        action_byte = ord(str(action_char)[0])  

        packet = struct.pack('BBBBBBBB',
                             0x05,           
                             current_state,  
                             target_byte,    
                             action_byte,    
                             0x00,           
                             0x00,           
                             0x00,           
                             0xFF)           
        
        try:
            self.control_sock.sendto(packet, (self.pi_ip, self.pi_port))
        except Exception as e:
            print(f"Error sending packet: {e}")

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
        self.telemetry_thread.stop()
        self.telemetry_thread.wait()
        super().closeEvent(event)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())