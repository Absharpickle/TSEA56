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

IP_ADDRESS_HOME = "192.168.1.50"
IP_ADDRESS_SITE = "10.42.0.1"



class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PiCam Ground Control")
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

        # --- VIDEO ---
        self.image_label = QLabel(self)
        self.image_label.setFixedSize(480, 360)
        self.image_label.setStyleSheet("border: 2px solid #34495e; background-color: black;")
        self.image_label.setScaledContents(True)
        self.top_hlayout.addWidget(self.image_label, alignment=Qt.AlignmentFlag.AlignTop)

        # --- RIGHT SIDE: Map + Items ---
        right_vlayout = QVBoxLayout()
        right_vlayout.setSpacing(5)

        self._setup_map(right_vlayout)
        self._setup_item_panel(right_vlayout)

        self.top_hlayout.addLayout(right_vlayout)

        # --- NETWORKING ---
        #self.pi_ip = IP_ADDRESS_HOME # Testing at home
        self.pi_ip = IP_ADDRESS_SITE # Testing at site
        self.pi_port = 5001
        self.control_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.control_sock.bind(("0.0.0.0", 0))

        # Internal item edge list
        self.item_edges = []

        self._setup_controls()
        self._setup_dashboard()
        self._start_threads()

        self.phase_names = {0: "IDLE", 1: "TO ITEM", 2: "PICKUP", 3: "TO HOME", 4: "DROP"}
        self.update_map_highlights()

    # =================================================================
    # SETUP HELPERS
    # =================================================================
    def _setup_map(self, parent_layout):
        """Create the 5x5 node map grid."""
        self.grid_nodes = {}
        self.current_active_node = 25

        self.map_frame = MapFrame(self.grid_nodes)
        self.map_frame.edge_clicked.connect(self._on_edge_clicked)
        self.map_layout = QGridLayout(self.map_frame)
        self.map_layout.setSpacing(10)

        self.node_style_idle = """
            background-color: #95a5a6; color: black; font-weight: bold;
            border-radius: 20px; font-size: 13px;
        """
        self.node_style_active = """
            background-color: #e74c3c; color: white; font-weight: bold;
            border-radius: 20px; font-size: 14px; border: 2px solid #f1c40f;
        """

        # Start node (25)
        self.lbl_start = QLabel("25")
        self.lbl_start.setFixedSize(40, 40)
        self.lbl_start.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_start.setStyleSheet(self.node_style_active)
        self.map_layout.addWidget(self.lbl_start, 0, 0, alignment=Qt.AlignmentFlag.AlignCenter)
        self.grid_nodes[25] = self.lbl_start

        # Grid nodes 0-24
        for i in range(25):
            rad = (i // 5) + 1
            kol = i % 5
            lbl = QLabel(str(i))
            lbl.setFixedSize(40, 40)
            lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            lbl.setStyleSheet(self.node_style_idle)
            self.map_layout.addWidget(lbl, rad, kol, alignment=Qt.AlignmentFlag.AlignCenter)
            self.grid_nodes[i] = lbl

        parent_layout.addWidget(self.map_frame, alignment=Qt.AlignmentFlag.AlignCenter)

    def _setup_item_panel(self, parent_layout):
        """Create the item edge selector and list below the map."""
        combo_style = "background-color: #ecf0f1; color: #2c3e50; padding: 3px 6px; font-size: 12px; font-weight: bold; border-radius: 3px;"
        btn_style = "background-color: #27ae60; color: white; font-weight: bold; padding: 3px 8px; border-radius: 3px; font-size: 12px;"
        btn_remove_style = "background-color: #c0392b; color: white; font-weight: bold; padding: 3px 8px; border-radius: 3px; font-size: 12px;"

        item_hlayout = QHBoxLayout()
        item_hlayout.setSpacing(6)

        self.edge_combo = QComboBox()
        self.edge_combo.setStyleSheet(combo_style)
        self.edge_combo.setMinimumWidth(120)
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

        parent_layout.addLayout(item_hlayout)

    def _setup_controls(self):
        """Create the control panel row (state, target, hotkey hint)."""
        control_label_style = "font-size: 13px; font-weight: bold; color: #ecf0f1;"
        combo_style = "background-color: #ecf0f1; color: #2c3e50; padding: 3px 6px; font-size: 12px; font-weight: bold; border-radius: 3px;"

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

        lbl_keys = QLabel("Keys: [1234] State | [WA] Target | [↑↓←→SEOB] Wheel | [VH] Arm")
        lbl_keys.setStyleSheet("color: #7f8c8d; font-size: 11px;")
        self.control_layout.addWidget(lbl_keys)

    def _setup_dashboard(self):
        """Create the live telemetry dashboard row."""
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
        self.lbl_action_done = QLabel("Done: ●")
        self.lbl_action_done.setStyleSheet("font-size: 13px; font-weight: bold; color: #e74c3c; padding: 3px;")

        for lbl in [self.lbl_phase, self.lbl_action, self.lbl_next_action,
                     self.lbl_line, self.lbl_gyro, self.lbl_flags,
                     self.lbl_items_progress]:
            lbl.setStyleSheet("font-size: 12px; font-weight: bold; color: #ecf0f1; padding: 3px;")
            self.dashboard_layout.addWidget(lbl)

        self.dashboard_layout.addWidget(self.lbl_action_done)

        self.layout.addWidget(self.dashboard_frame)

    def _start_threads(self):
        """Start the video and telemetry background threads."""
        self.video_thread = VideoThread()
        self.video_thread.change_pixmap_signal.connect(self.update_image)
        self.video_thread.start()

        self.telemetry_thread = TelemetryThread(self.control_sock)
        self.telemetry_thread.telemetry_signal.connect(self.update_telemetry_dashboard)
        self.telemetry_thread.route_signal.connect(self.update_route)
        self.telemetry_thread.start()

    # =================================================================
    # ITEM LIST MANAGEMENT
    # =================================================================
    def add_item_edge(self, edge=None):
        if edge is None:
            edge = self.edge_combo.currentData()
        if edge and edge not in self.item_edges:
            self.item_edges.append(edge)
            self.refresh_item_list_widget()
            self.update_map_highlights()
        self.setFocus()

    def _on_edge_clicked(self, edge):
        """Handle click on a map edge — toggle: add or remove item."""
        if edge in self.item_edges:
            self.item_edges.remove(edge)
            self.refresh_item_list_widget()
            self.update_map_highlights()
        else:
            self.add_item_edge(edge)
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

    # =================================================================
    # TELEMETRY
    # =================================================================
    def update_telemetry_dashboard(self, data):
        phase_str = self.phase_names.get(data['phase'], "UNKNOWN")
        self.lbl_phase.setText(f"Phase: {phase_str}")
        self.lbl_action.setText(f"Action: '{data['action']}'")
        self.lbl_next_action.setText(f"Next Action: '{data['next_action']}'")
        self.lbl_line.setText(f"Line: {data['line_var']}")
        self.lbl_gyro.setText(f"Gyro: ({data['gyro1']}, {data['gyro2']})")
        self.lbl_flags.setText(f"Flags: {data['flags']}")

        # Action done indicator
        done = data.get('action_done', 0)
        if done:
            self.lbl_action_done.setStyleSheet("font-size: 13px; font-weight: bold; color: #2ecc71; padding: 3px;")
            self.lbl_action_done.setText("Done: ● YES")
        else:
            self.lbl_action_done.setStyleSheet("font-size: 13px; font-weight: bold; color: #e74c3c; padding: 3px;")
            self.lbl_action_done.setText("Done: ● NO")

        item_idx = data.get('current_item', 0)
        item_total = data.get('item_count', 0)
        if item_total > 0:
            self.lbl_items_progress.setText(f"Items: {min(item_idx+1, item_total)}/{item_total}")
        else:
            self.lbl_items_progress.setText("Items: -")

        # Clear route display when robot goes idle
        if data['phase'] == 0 and self.map_frame.route_nodes:
            self.map_frame.set_route([])

        node = data.get('current_node', 25)
        direction = data.get('direction', 's')
        if node != self.current_active_node or direction != getattr(self, 'current_dir', 's'):
            self.current_dir = direction
            self.set_active_node(node, direction)

    def update_route(self, route_nodes):
        """Called when the robot sends a 0x08 route packet."""
        self.map_frame.set_route(route_nodes)

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

    # =================================================================
    # KEYBOARD INPUT
    # =================================================================
    def keyPressEvent(self, event):
        key = event.key()

        # State hotkeys
        if key == Qt.Key.Key_1: self.state_combo.setCurrentIndex(0); return
        elif key == Qt.Key.Key_2: self.state_combo.setCurrentIndex(1); return
        elif key == Qt.Key.Key_3: self.state_combo.setCurrentIndex(2); return
        elif key == Qt.Key.Key_4: self.state_combo.setCurrentIndex(3); return

        # Target hotkeys
        elif key == Qt.Key.Key_T: self.target_combo.setCurrentIndex(0); return
        elif key == Qt.Key.Key_A: self.target_combo.setCurrentIndex(1); return

        # Action keys
        action_char = None
        target = self.target_combo.currentData()

        if target == "wheel":
            key_map = {
                Qt.Key.Key_Up: 'f', Qt.Key.Key_Down: 'b',
                Qt.Key.Key_Right: 'r', Qt.Key.Key_Left: 'l',
                Qt.Key.Key_S: 's', Qt.Key.Key_E: 'e',
                Qt.Key.Key_O: 'o', Qt.Key.Key_B: 'b',
            }
            action_char = key_map.get(key)
        elif target == "arm":
            key_map = {Qt.Key.Key_V: 'v', Qt.Key.Key_H: 'h', Qt.Key.Key_W: 'w'}
            action_char = key_map.get(key)

        if action_char:
            self.send_packet(action_char)

    # =================================================================
    # NETWORKING
    # =================================================================
    def send_packet(self, action_char):
        current_state = self.state_combo.currentData()
        current_target = self.target_combo.currentData()

        # In auto modes, send item list before start command
        if current_state in (0x00, 0x01) and action_char == 'f' and self.item_edges:
            packet = build_item_list_packet(self.item_edges)
            if packet:
                try:
                    self.control_sock.sendto(packet, (self.pi_ip, self.pi_port))
                    print(f"[ITEMS] Sent {len(self.item_edges)} item(s) to robot")
                except Exception as e:
                    print(f"Error sending item list: {e}")

        packet = build_command_packet(current_state, current_target, action_char)
        try:
            self.control_sock.sendto(packet, (self.pi_ip, self.pi_port))
        except Exception as e:
            print(f"Error sending packet: {e}")

    # =================================================================
    # VIDEO
    # =================================================================
    def update_image(self, cv_img):
        rgb = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        qt_img = QImage(rgb.data, w, h, ch * w, QImage.Format.Format_RGB888)
        self.image_label.setPixmap(QPixmap.fromImage(qt_img))

    def closeEvent(self, event):
        self.telemetry_thread.stop()
        self.telemetry_thread.wait()
        super().closeEvent(event)
