import cv2
import socket
from PyQt6.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
                             QLabel, QComboBox, QFrame, QGridLayout,
                             QSizePolicy, QPushButton, QListWidget)
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QImage, QPixmap

from threads import VideoThread, TelemetryThread
from map_widget import MapFrame
from protocol import build_command_packet, build_item_list_packet

IP_ADDRESS_SITE = "10.42.0.1"  # Ändra vid behov
PI_PORT = 5001

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PiCam Ground Control")
        self.pi_ip = IP_ADDRESS_SITE
        self.pi_port = PI_PORT
        
        self.setStyleSheet("""
            QMainWindow { background-color: #2c3e50; color: white; }
            QLabel { color: white; font-family: 'Segoe UI', sans-serif; }
            QFrame#Dashboard { 
                background-color: #34495e; 
                border: 2px solid #7f8c8d; 
                border-radius: 5px; 
            }
        """)
        
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QVBoxLayout(self.central_widget)
        
        # --- TOP LAYOUT (Video & Map) ---
        self.top_hlayout = QHBoxLayout()
        self.layout.addLayout(self.top_hlayout)

        # Video
        self.image_label = QLabel("Väntar på video...")
        self.image_label.setFixedSize(480, 360)
        self.image_label.setStyleSheet("background-color: black; border: 1px solid #7f8c8d;")
        self.image_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.top_hlayout.addWidget(self.image_label)

        # Karta (MapWidget)
        from pathfinding import NODES # Importera noder för referens om det behövs
        self.map_frame = MapFrame(range(26)) # Skickar med 26 noder (0-25)
        self.top_hlayout.addWidget(self.map_frame)

        # --- BOTTOM LAYOUT (Controls & Dashboard) ---
        self.bottom_hlayout = QHBoxLayout()
        self.layout.addLayout(self.bottom_hlayout)

        # Dashboard (Sensor Data)
        self._setup_dashboard()
        
        # Controls (Combo boxes)
        self._setup_controls()

        # --- NETWORKING & THREADS ---
        self.control_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # Video Thread
        self.video_thread = VideoThread()
        self.video_thread.change_pixmap_signal.connect(self.update_image)
        self.video_thread.start()

        # Telemetry Thread
        self.telemetry_thread = TelemetryThread(self.control_sock)
        self.telemetry_thread.telemetry_signal.connect(self.update_telemetry)
        self.telemetry_thread.route_signal.connect(self.map_frame.set_route)
        self.telemetry_thread.start()

        # State för ruttplanering
        self.item_edges = []
        self.map_frame.edge_clicked.connect(self.add_item_to_list)

    def _setup_dashboard(self):
        self.dashboard_frame = QFrame()
        self.dashboard_frame.setObjectName("Dashboard")
        self.dashboard_layout = QGridLayout(self.dashboard_frame)
        
        # Labels för telemetri
        self.phase_label = QLabel("Fas: IDLE")
        self.node_label  = QLabel("Nod: --")
        self.action_label = QLabel("Aktion: --")
        self.line_label = QLabel("Linje-var: --")
        
        # NYA LABELS: IR och Hinder
        self.ir_label = QLabel("IR Avstånd: --")
        self.hinder_label = QLabel("Väg: Klar")
        self.hinder_label.setStyleSheet("font-weight: bold; color: #2ecc71;")

        self.dashboard_layout.addWidget(self.phase_label, 0, 0)
        self.dashboard_layout.addWidget(self.node_label, 0, 1)
        self.dashboard_layout.addWidget(self.action_label, 1, 0)
        self.dashboard_layout.addWidget(self.line_label, 1, 1)
        self.dashboard_layout.addWidget(self.ir_label, 2, 0)
        self.dashboard_layout.addWidget(self.hinder_label, 2, 1)

        self.bottom_hlayout.addWidget(self.dashboard_frame)

    def _setup_controls(self):
        control_panel = QVBoxLayout()
        
        self.state_combo = QComboBox()
        self.state_combo.addItem("Autonomous (Normal)", 0x00)
        self.state_combo.addItem("Autonomous (No Reverse)", 0x01)
        self.state_combo.addItem("Manual (Wheel)", 0x02)
        self.state_combo.addItem("Manual (Arm)", 0x03)
        
        self.target_combo = QComboBox()
        self.target_combo.addItem("Wheels", "wheel")
        self.target_combo.addItem("Arm/Gripper", "arm")

        self.clear_items_btn = QPushButton("Rensa Varulista")
        self.clear_items_btn.clicked.connect(self.clear_items)

        control_panel.addWidget(QLabel("Körläge:"))
        control_panel.addWidget(self.state_combo)
        control_panel.addWidget(QLabel("Mål (Manuellt):"))
        control_panel.addWidget(self.target_combo)
        control_panel.addWidget(self.clear_items_btn)
        
        self.bottom_hlayout.addLayout(control_panel)

    # =================================================================
    # SLOTS & UPDATES
    # =================================================================
    def update_telemetry(self, data):
        """Hanterar inkommande 15-byte telemetri-dict från threads.py"""
        phases = ["IDLE", "TILL VARA", "UPPHÄMTNING", "HEM", "AVLÄMNING"]
        p_idx = data['phase']
        phase_str = phases[p_idx] if p_idx < len(phases) else f"UNKNOWN ({p_idx})"
        
        self.phase_label.setText(f"Fas: {phase_str}")
        self.node_label.setText(f"Nod: {data['current_node']}")
        self.action_label.setText(f"Nu: {data['action']} | Nästa: {data['next_action']}")
        self.line_label.setText(f"Linje-var: {data['line_var']}")
        
        # Uppdatera IR-avstånd (Index 13 i paketet)
        ir_val = data['ir_distance']
        self.ir_label.setText(f"IR Avstånd: {ir_val}")

        # Hantera Hinder-varning (Bit 4 i flags)
        hinder_flagga = (data['flags'] & 0x10) >> 4
        if hinder_flagga == 1:
            self.hinder_label.setText("HINDER UPPTÄCKT!")
            self.hinder_label.setStyleSheet("font-weight: bold; color: #e74c3c;") # Röd
        else:
            self.hinder_label.setText("Väg: Klar")
            self.hinder_label.setStyleSheet("font-weight: bold; color: #2ecc71;") # Grön

    def update_image(self, cv_img):
        rgb = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        qt_img = QImage(rgb.data, w, h, ch * w, QImage.Format.Format_RGB888)
        self.image_label.setPixmap(QPixmap.fromImage(qt_img))

    def add_item_to_list(self, edge):
        self.item_edges.append(edge)
        self.map_frame.set_item_edges(self.item_edges)

    def clear_items(self):
        self.item_edges = []
        self.map_frame.set_item_edges([])
        self.map_frame.set_route([])

    def keyPressEvent(self, event):
        """Hanterar manuell körning via tangentbordet"""
        key_map = {
            Qt.Key.Key_W: 'f',
            Qt.Key.Key_S: 'b',
            Qt.Key.Key_A: 'v',
            Qt.Key.Key_D: 'h',
            Qt.Key.Key_Space: 's',
            Qt.Key.Key_Enter: 'f', # Starta autonomt
            Qt.Key.Key_Return: 'f'
        }
        if event.key() in key_map:
            self.send_packet(key_map[event.key()])

    def send_packet(self, action_char):
        current_state = self.state_combo.currentData()
        current_target = self.target_combo.currentData()

        # Om vi startar autonomt ('f' i IDLE), skicka varulistan först
        if current_state in (0x00, 0x01) and action_char == 'f' and self.item_edges:
            packet = build_item_list_packet(self.item_edges)
            if packet:
                self.control_sock.sendto(packet, (self.pi_ip, self.pi_port))

        # Skicka huvudkommandot
        packet = build_command_packet(current_state, current_target, action_char)
        try:
            self.control_sock.sendto(packet, (self.pi_ip, self.pi_port))
        except Exception as e:
            print(f"Sändningsfel: {e}")
