import sys
import socket
from PyQt6.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                             QLabel, QGridLayout, QFrame)
from PyQt6.QtCore import Qt, pyqtSlot
from PyQt6.QtGui import QImage, QPixmap

import protocol
from threads import VideoThread, TelemetryThread
from map_widget import MapFrame

# Inställningar
ROBOT_IP = "192.168.1.100"  # Ändra till robotens faktiska IP
UDP_PORT = 5001

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Ground Control - Arm & Navigation")
        self.resize(1000, 700)
        
        # State för armen och styrsystemet
        self.selected_joint = 0  
        self.current_state = 0x02 # Starta i Manuellt läge (0x02)
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        self.init_ui()
        self.start_threads()

    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)

        # --- Vänster sida: Video och Kontrollinfo ---
        left_layout = QVBoxLayout()
        
        self.video_label = QLabel("Väntar på video...")
        self.video_label.setFixedSize(640, 480)
        self.video_label.setStyleSheet("background-color: black; color: white;")
        self.video_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        left_layout.addWidget(self.video_label)

        # Kontroll-instruktioner
        info_box = QFrame()
        info_box.setStyleSheet("background-color: #2c3e50; color: white; border-radius: 5px;")
        info_layout = QGridLayout(info_box)
        info_layout.addWidget(QLabel("<b>Hjul:</b> W,A,S,D,Q,E (O = Auto/Manuell)"), 0, 0)
        info_layout.addWidget(QLabel("<b>Välj Joint:</b> 5, 6, 7, 8, 9, 0"), 1, 0)
        info_layout.addWidget(QLabel("<b>Välj Kamera:</b> K"), 1, 1)
        info_layout.addWidget(QLabel("<b>Styr Arm:</b> + (Öka), - (Minska), . (Stopp)"), 2, 0)
        left_layout.addWidget(info_box)
        
        main_layout.addLayout(left_layout)

        # --- Höger sida: Karta och Telemetri ---
        right_layout = QVBoxLayout()
        self.map_widget = MapFrame({}) 
        right_layout.addWidget(self.map_widget)

        # Telemetri-data
        self.tele_box = QFrame()
        self.tele_box.setStyleSheet("background-color: #ecf0f1; border: 1px solid #bdc3c7;")
        tele_layout = QVBoxLayout(self.tele_box)
        
        self.label_phase = QLabel("Fas: IDLE")
        self.label_node = QLabel("Nod: -")
        self.label_action = QLabel("Aktion: -")
        self.label_sensors = QLabel("Sensorer: -")
        self.label_joint_status = QLabel("Vald Joint: 1")
        
        tele_layout.addWidget(QLabel("<b>TELEMETRI</b>"))
        tele_layout.addWidget(self.label_phase)
        tele_layout.addWidget(self.label_node)
        tele_layout.addWidget(self.label_action)
        tele_layout.addWidget(self.label_sensors)
        tele_layout.addWidget(self.label_joint_status)
        
        right_layout.addWidget(self.tele_box)
        main_layout.addLayout(right_layout)

    def start_threads(self):
        self.video_thread = VideoThread()
        self.video_thread.change_pixmap_signal.connect(self.update_image)
        self.video_thread.start()

        self.tele_thread = TelemetryThread(self.udp_sock)
        self.tele_thread.telemetry_signal.connect(self.update_telemetry)
        self.tele_thread.route_signal.connect(self.map_widget.set_route)
        self.tele_thread.start()

    @pyqtSlot(object)
    def update_image(self, cv_img):
        height, width, channel = cv_img.shape
        bytes_per_line = 3 * width
        q_img = QImage(cv_img.data, width, height, bytes_per_line, QImage.Format.Format_RGB888).rgbSwapped()
        self.video_label.setPixmap(QPixmap.fromImage(q_img))

    @pyqtSlot(dict)
    def update_telemetry(self, data):
        phases = ["IDLE", "TILL VARA", "PLOCKAR", "HEM", "LÄMNAR"]
        self.label_phase.setText(f"Fas: {phases[data['phase']] if data['phase'] < 5 else data['phase']}")
        self.label_node.setText(f"Nod: {data['node']} (Riktning: {data['direction']})")
        self.label_action.setText(f"Aktion: {data['action']} -> Nästa: {data['next_action']}")
        self.label_sensors.setText(f"Sens: F:{data['line_var_f']} | Flags: 0x{data['flags']:02X}")

    def keyPressEvent(self, event):
        key = event.text().lower()

        # ==========================================
        # 1. VÄLJ JOINT (5, 6, 7, 8, 9, 0 eller K)
        # ==========================================
        joint_map = {'5': 0, '6': 1, '7': 2, '8': 3, '9': 4, '0': 5}
        if key in joint_map:
            self.selected_joint = joint_map[key]
            self.label_joint_status.setText(f"Vald Joint: {self.selected_joint + 1} (Bit {self.selected_joint + 2})")
            print(f"Vald Joint: {self.selected_joint + 1}")
            return
        elif key == 'k':
            self.selected_joint = 6
            self.label_joint_status.setText("Vald Joint: KAMERA")
            print("Vald: Kamera")
            return

        # ==========================================
        # 2. STYR ARM (+, -, .)
        # ==========================================
        arm_moves = {'+': 'inc', '-': 'dec', '.': 'stop'}
        if key in arm_moves:
            if hasattr(self, 'selected_joint'):
                action_byte = protocol.encode_arm_action(self.selected_joint, arm_moves[key])
                # Target = 0x01 (Arm)
                pkt = protocol.build_command_packet(self.current_state, 0x01, action_byte)
                self.udp_sock.sendto(pkt, (ROBOT_IP, UDP_PORT))
            else:
                print("Välj en joint (5-0 eller K) först!")
            return

        # ==========================================
        # 3. STYR HJUL (W, A, S, D, Q, E, O)
        # ==========================================
        wheel_keys = ['w', 'a', 's', 'd', 'q', 'e', 'o']
        if key in wheel_keys:
            if key == 'o':
                self.current_state = 0x00 if self.current_state == 0x02 else 0x02
                print(f"State ändrat till: {hex(self.current_state)}")
            
            # Target = 0x00 (Hjul)
            pkt = protocol.build_command_packet(self.current_state, 0x00, key)
            self.udp_sock.sendto(pkt, (ROBOT_IP, UDP_PORT))

    def closeEvent(self, event):
        self.video_thread.terminate()
        self.tele_thread.running = False
        self.tele_thread.terminate()
        super().closeEvent(event)

if __name__ == "__main__":
    from PyQt6.QtWidgets import QApplication
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
