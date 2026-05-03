import sys
import cv2
import numpy as np
import socket
import struct
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QComboBox, QFrame, QGridLayout,
                             QSizePolicy, QPushButton, QListWidget, QListWidgetItem)
from PyQt6.QtCore import QThread, pyqtSignal, Qt, QPoint
from PyQt6.QtGui import QImage, QPixmap, QPainter, QPen, QColor, QFont
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
                if len(data) == 13 and data[0] == 0x06 and data[12] == 0xFF:
                    unpacked = struct.unpack('13B', data)
                    
                    telemetry_data = {
                        'phase': unpacked[1],
                        'action': chr(unpacked[2]), 
                        'next_action': chr(unpacked[3]),
                        'line_var': unpacked[4],
                        'gyro1': unpacked[5],
                        'gyro2': unpacked[6],
                        'flags': unpacked[7],
                        'current_node': unpacked[8],
                        'current_item': unpacked[9],
                        'item_count': unpacked[10],
                        'direction': chr(unpacked[11])
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
        self.highlighted_edges = []  # List of (node_a, node_b) tuples for item locations
        self.setStyleSheet("background-color: #1a252f; border: 2px solid #7f8c8d; border-radius: 5px;")
        self.setFixedSize(480, 480)

    def set_item_edges(self, edges_list):
        """Set the highlighted edges (item locations) and repaint."""
        self.highlighted_edges = list(edges_list)
        self.update()

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

        # Draw highlighted edges (item locations) on top in orange with order numbers
        if self.highlighted_edges:
            highlight_pen = QPen(QColor("#e67e22"), 8)
            highlight_pen.setCapStyle(Qt.PenCapStyle.RoundCap)
            number_font = QFont("Arial", 14, QFont.Weight.Bold)
            painter.setFont(number_font)
            
            for idx, (a, b) in enumerate(self.highlighted_edges):
                if a in self.grid_nodes and b in self.grid_nodes:
                    p1 = get_center(a)
                    p2 = get_center(b)
                    painter.setPen(highlight_pen)
                    painter.drawLine(p1, p2)
                    
                    # Draw order number at midpoint
                    mid = QPoint((p1.x() + p2.x()) // 2, (p1.y() + p2.y()) // 2)
                    painter.setPen(QPen(QColor("#ffffff")))
                    painter.drawText(mid.x() - 6, mid.y() + 5, str(idx + 1))

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
        self.layout.setSpacing(5)
        self.layout.setContentsMargins(10, 5, 10, 5)

        self.top_hlayout = QHBoxLayout()
        self.top_hlayout.setSpacing(10)
        self.layout.addLayout(self.top_hlayout)

        self.image_label = QLabel(self)
        self.image_label.setFixedSize(480, 360)
        self.image_label.setStyleSheet("border: 2px solid #34495e; background-color: black;")
        self.image_label.setScaledContents(True)
        self.top_hlayout.addWidget(self.image_label, alignment=Qt.AlignmentFlag.AlignTop)

        # --- RIGHT SIDE: Map + Items ---
        right_vlayout = QVBoxLayout()
        right_vlayout.setSpacing(5)

        # --- MAP SETUP ---
        self.grid_nodes = {} 
        self.current_active_node = 25 
        
        self.map_frame = MapFrame(self.grid_nodes)
        self.map_frame.setFixedSize(360, 360)
        self.map_layout = QGridLayout(self.map_frame)
        self.map_layout.setSpacing(10) 

        self.node_style_idle = """
            background-color: #95a5a6; 
            color: black; 
            font-weight: bold; 
            border-radius: 20px; 
            font-size: 13px;
        """
        
        self.node_style_active = """
            background-color: #e74c3c; 
            color: white; 
            font-weight: bold; 
            border-radius: 20px; 
            font-size: 14px;
            border: 2px solid #f1c40f;
        """

        self.lbl_start = QLabel("25")
        self.lbl_start.setFixedSize(40, 40)
        self.lbl_start.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_start.setStyleSheet(self.node_style_active) 
        self.map_layout.addWidget(self.lbl_start, 0, 0, alignment=Qt.AlignmentFlag.AlignCenter)
        self.grid_nodes[25] = self.lbl_start

        for i in range(25):
            rad = (i // 5) + 1 
            kol = i % 5
            lbl = QLabel(str(i))
            lbl.setFixedSize(40, 40)
            lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            lbl.setStyleSheet(self.node_style_idle)
            self.map_layout.addWidget(lbl, rad, kol, alignment=Qt.AlignmentFlag.AlignCenter)
            self.grid_nodes[i] = lbl

        right_vlayout.addWidget(self.map_frame, alignment=Qt.AlignmentFlag.AlignCenter)

        # --- TELEMETRY SETUP ---
        self.pi_ip = "192.168.1.50"
        self.pi_port = 5001
        self.control_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.control_sock.bind(("0.0.0.0", 0)) 

        # Gemensam stil
        control_label_style = "font-size: 13px; font-weight: bold; color: #ecf0f1;"
        combo_style = "background-color: #ecf0f1; color: #2c3e50; padding: 3px 6px; font-size: 12px; font-weight: bold; border-radius: 3px;"
        btn_style = "background-color: #27ae60; color: white; font-weight: bold; padding: 3px 8px; border-radius: 3px; font-size: 12px;"
        btn_remove_style = "background-color: #c0392b; color: white; font-weight: bold; padding: 3px 8px; border-radius: 3px; font-size: 12px;"

        # --- ITEM PANEL (under the map, inside right column) ---
        item_hlayout = QHBoxLayout()
        item_hlayout.setSpacing(6)

        self.edge_combo = QComboBox()
        self.edge_combo.setStyleSheet(combo_style)
        self.edge_combo.setMinimumWidth(80)
        for i in range(25):
            kol = i % 5
            rad = i // 5
            if kol < 4:
                self.edge_combo.addItem(f"{i}↔{i+1}", (i, i+1))
            if rad < 4:
                self.edge_combo.addItem(f"{i}↔{i+5}", (i, i+5))
        self.edge_combo.currentIndexChanged.connect(lambda: self.setFocus())

        self.btn_add = QPushButton("+")
        self.btn_add.setStyleSheet(btn_style)
        self.btn_add.setFixedWidth(30)
        self.btn_add.clicked.connect(self.add_item_edge)

        self.item_list_widget = QListWidget()
        self.item_list_widget.setStyleSheet(
            "background-color: #34495e; color: #ecf0f1; font-size: 12px; "
            "font-weight: bold; border-radius: 3px; padding: 2px;"
        )
        self.item_list_widget.setMaximumHeight(55)
        self.item_list_widget.setMinimumWidth(160)

        self.btn_remove = QPushButton("−")
        self.btn_remove.setStyleSheet(btn_remove_style)
        self.btn_remove.setFixedWidth(30)
        self.btn_remove.clicked.connect(self.remove_selected_item)

        self.btn_clear = QPushButton("Clr")
        self.btn_clear.setStyleSheet(btn_remove_style)
        self.btn_clear.setFixedWidth(35)
        self.btn_clear.clicked.connect(self.clear_item_list)

        item_hlayout.addWidget(self.edge_combo)
        item_hlayout.addWidget(self.btn_add)
        item_hlayout.addWidget(self.item_list_widget, 1)
        item_hlayout.addWidget(self.btn_remove)
        item_hlayout.addWidget(self.btn_clear)

        right_vlayout.addLayout(item_hlayout)
        self.top_hlayout.addLayout(right_vlayout)

        # Internal item edge list
        self.item_edges = []

        # --- CONTROL PANEL (single compact row) ---
        self.control_layout = QHBoxLayout()
        self.control_layout.setSpacing(8)
        self.control_layout.setContentsMargins(20, 3, 20, 3)
        self.layout.addLayout(self.control_layout)

        lbl_state = QLabel("State:")
        lbl_state.setStyleSheet(control_label_style)
        lbl_state.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)
        
        self.state_combo = QComboBox()
        self.state_combo.setStyleSheet(combo_style)
        self.state_combo.setMinimumWidth(160)
        self.state_combo.addItem("1: (Auto, Auto)", 0)
        self.state_combo.addItem("2: (Auto, Manual)", 1)
        self.state_combo.addItem("3: (Manual, Auto)", 2)
        self.state_combo.addItem("4: (Manual, Manual)", 3)
        self.state_combo.setCurrentIndex(3) 
        self.state_combo.currentIndexChanged.connect(lambda: self.setFocus()) 
        
        self.control_layout.addWidget(lbl_state)
        self.control_layout.addWidget(self.state_combo)
        self.control_layout.addStretch()

        lbl_target = QLabel("Target:")
        lbl_target.setStyleSheet(control_label_style)
        lbl_target.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)

        self.target_combo = QComboBox()
        self.target_combo.setStyleSheet(combo_style)
        self.target_combo.setMinimumWidth(100)
        self.target_combo.addItem("Wheel", "wheel")
        self.target_combo.addItem("Arm", "arm")
        self.target_combo.currentIndexChanged.connect(lambda: self.setFocus())
        
        self.control_layout.addWidget(lbl_target)
        self.control_layout.addWidget(self.target_combo)
        self.control_layout.addStretch()

        lbl_keys = QLabel("Keys: [←1234] State | [WA] Target | [↑↓←→SEOU] Wheel | [VH] Arm")
        lbl_keys.setStyleSheet("color: #7f8c8d; font-size: 11px;")
        self.control_layout.addWidget(lbl_keys)

        # --- LIVE TELEMETRY DASHBOARD ---
        self.dashboard_frame = QFrame()
        self.dashboard_frame.setStyleSheet("QFrame { background-color: #34495e; border-radius: 4px; }")
        self.dashboard_layout = QHBoxLayout(self.dashboard_frame)
        self.dashboard_layout.setContentsMargins(8, 4, 8, 4)
        
        self.lbl_phase = QLabel("Phase: IDLE")
        self.lbl_action = QLabel("Last Action: -")
        self.lbl_next_action = QLabel("Next Action: -")
        self.lbl_line = QLabel("Line: 0")
        self.lbl_gyro = QLabel("Gyro: (0, 0)")
        self.lbl_flags = QLabel("Flags: 0")
        self.lbl_items_progress = QLabel("Items: -")
        
        for lbl in [self.lbl_phase, self.lbl_action, self.lbl_next_action, self.lbl_line, self.lbl_gyro, self.lbl_flags, self.lbl_items_progress]:
            lbl.setStyleSheet("font-size: 12px; font-weight: bold; color: #ecf0f1; padding: 3px;")
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

        # Set initial map state
        self.update_map_highlights()

    # --- ITEM LIST MANAGEMENT ---
    def add_item_edge(self):
        edge = self.edge_combo.currentData()
        if edge and edge not in self.item_edges:
            self.item_edges.append(edge)
            self.refresh_item_list_widget()
            self.update_map_highlights()
        self.setFocus()

    def remove_selected_item(self):
        row = self.item_list_widget.currentRow()
        if 0 <= row < len(self.item_edges):
            self.item_edges.pop(row)
            self.refresh_item_list_widget()
            self.update_map_highlights()
        self.setFocus()

    def clear_item_list(self):
        self.item_edges.clear()
        self.refresh_item_list_widget()
        self.update_map_highlights()
        self.setFocus()

    def refresh_item_list_widget(self):
        self.item_list_widget.clear()
        for idx, (u, v) in enumerate(self.item_edges):
            self.item_list_widget.addItem(f"  {idx+1}. Edge {u} ↔ {v}")

    def update_map_highlights(self):
        self.map_frame.set_item_edges(self.item_edges)

    def update_telemetry_dashboard(self, data):
        phase_str = self.phase_names.get(data['phase'], "UNKNOWN")
        self.lbl_phase.setText(f"Phase: {phase_str}")
        self.lbl_action.setText(f"Action: '{data['action']}'")
        self.lbl_next_action.setText(f"Next Action: '{data['next_action']}'")
        self.lbl_line.setText(f"Line: {data['line_var']}")
        self.lbl_gyro.setText(f"Gyro: ({data['gyro1']}, {data['gyro2']})")
        self.lbl_flags.setText(f"Flags: {data['flags']}")

        # Item progress
        item_idx = data.get('current_item', 0)
        item_total = data.get('item_count', 0)
        if item_total > 0:
            self.lbl_items_progress.setText(f"Items: {min(item_idx+1, item_total)}/{item_total}")
        else:
            self.lbl_items_progress.setText("Items: -")

        # Uppdatera aktiv nod och riktningspil på kartan
        node = data.get('current_node', 25)
        direction = data.get('direction', 's')
        if node != self.current_active_node or direction != getattr(self, 'current_dir', 's'):
            self.current_dir = direction
            self.set_active_node(node, direction)


    def set_active_node(self, node_id, direction='s'):
        dir_arrows = {'n': '↑', 's': '↓', 'e': '→', 'w': '←'}
        arrow = dir_arrows.get(direction, '')

        if self.current_active_node in self.grid_nodes:
            old_lbl = self.grid_nodes[self.current_active_node]
            old_lbl.setStyleSheet(self.node_style_idle)
            old_lbl.setText(str(self.current_active_node))
        
        if node_id in self.grid_nodes:
            lbl = self.grid_nodes[node_id]
            lbl.setStyleSheet(self.node_style_active)
            lbl.setText(f"{arrow}")
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

    def send_item_list_packet(self):
        """Send 0x07 item-list packet with all configured items."""
        if not self.item_edges:
            print("[!] No items to send")
            return
        
        n = len(self.item_edges)
        # Header (0x07) + count (N) + N*(u, v) + footer (0xFF)
        fmt = 'B' * (3 + 2 * n)
        values = [0x07, n]
        for u, v in self.item_edges:
            values.extend([u, v])
        values.append(0xFF)
        
        packet = struct.pack(fmt, *values)
        try:
            self.control_sock.sendto(packet, (self.pi_ip, self.pi_port))
            print(f"[ITEMS] Sent {n} item(s) to robot")
        except Exception as e:
            print(f"Error sending item list: {e}")

    def send_packet(self, action_char):
        current_state = self.state_combo.currentData()
        current_target = self.target_combo.currentData()

        target_byte = 0x00 if current_target == "wheel" else 0x01
        action_byte = ord(str(action_char)[0])

        # In auto modes, send item list before start command
        if current_state in (0x00, 0x01) and action_char == 'f' and self.item_edges:
            self.send_item_list_packet()

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