import sys
import socket
from PyQt6.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
                             QLabel, QFrame, QPushButton)
from PyQt6.QtCore import Qt, pyqtSlot
from PyQt6.QtGui import QImage, QPixmap

import protocol
from threads import VideoThread, TelemetryThread
from map_widget import MapFrame

# ── Network settings ────────────────────────────────────────────────
ROBOT_IP = "192.168.1.100"
UDP_PORT  = 5001


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Ground Control")
        self.resize(1100, 800)

        # Arm-control state
        self.selected_joint  = 0      # 0-5 = J1-J6, 6 = camera
        self.current_state   = 0x02   # Start in manual mode

        # Item edges selected on the map
        self.item_edges = []

        # UDP socket — shared for TX (commands) and RX (telemetry)
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self._init_ui()
        self._start_threads()

    # ──────────────────────────────────────────────────────────────────
    # UI setup
    # ──────────────────────────────────────────────────────────────────

    def _init_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QHBoxLayout(central)
        root.setSpacing(12)
        root.setContentsMargins(10, 10, 10, 10)

        # ── Left column: video + help text ───────────────────────────
        left = QVBoxLayout()

        self.video_label = QLabel("Waiting for video stream…")
        self.video_label.setFixedSize(640, 480)
        self.video_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.video_label.setStyleSheet("background-color: #111; color: #aaa;")
        left.addWidget(self.video_label)

        help_label = QLabel(
            "<b>Hjul:</b> W A S D &nbsp;|&nbsp; "
            "<b>Joint:</b> 5-0 &nbsp;|&nbsp; "
            "<b>Kamera:</b> K &nbsp;|&nbsp; "
            "<b>Arm:</b> + &minus; . &nbsp;|&nbsp; "
            "<b>Klicka karta:</b> lägg till vara"
        )
        help_label.setWordWrap(True)
        left.addWidget(help_label)

        # Clear items button
        clear_btn = QPushButton("Rensa varor")
        clear_btn.clicked.connect(self._clear_items)
        left.addWidget(clear_btn)

        left.addStretch()
        root.addLayout(left)

        # ── Right column: map + telemetry ────────────────────────────
        right = QVBoxLayout()
        right.setSpacing(8)

        # MapFrame is now self-contained — no grid_nodes dict needed
        self.map_widget = MapFrame()
        self.map_widget.edge_clicked.connect(self._handle_map_click)
        right.addWidget(self.map_widget)

        # Telemetry display
        self.lbl_node      = QLabel("Nod: –")
        self.lbl_phase     = QLabel("Fas: –")
        self.lbl_action    = QLabel("Åtgärd: –")
        self.lbl_joint     = QLabel("Vald joint: J1")
        self.lbl_items     = QLabel("Varor: 0 st")
        self.lbl_direction = QLabel("Riktning: –")

        for lbl in (self.lbl_node, self.lbl_phase, self.lbl_action,
                    self.lbl_joint, self.lbl_items, self.lbl_direction):
            lbl.setStyleSheet("font-family: monospace; font-size: 12px;")
            right.addWidget(lbl)

        right.addStretch()
        root.addLayout(right)

    # ──────────────────────────────────────────────────────────────────
    # Thread management
    # ──────────────────────────────────────────────────────────────────

    def _start_threads(self):
        self.video_thread = VideoThread()
        self.video_thread.change_pixmap_signal.connect(self._update_image)
        self.video_thread.start()

        self.tele_thread = TelemetryThread(self.udp_sock)
        self.tele_thread.telemetry_signal.connect(self._update_telemetry)
        self.tele_thread.route_signal.connect(self.map_widget.set_route)
        self.tele_thread.start()

    # ──────────────────────────────────────────────────────────────────
    # Map interaction
    # ──────────────────────────────────────────────────────────────────

    def _handle_map_click(self, edge: tuple):
        """Toggle an edge in the item list and (re-)send the 0x07 packet."""
        if edge in self.item_edges:
            self.item_edges.remove(edge)
        else:
            self.item_edges.append(edge)

        self.map_widget.set_item_edges(self.item_edges)
        self.lbl_items.setText(f"Varor: {len(self.item_edges)} st")

        pkt = protocol.build_item_list_packet(self.item_edges)
        if pkt:
            self.udp_sock.sendto(pkt, (ROBOT_IP, UDP_PORT))
        print(f"[MAP] Varor: {self.item_edges}")

    def _clear_items(self):
        """Remove all item edges from the map."""
        self.item_edges.clear()
        self.map_widget.set_item_edges([])
        self.lbl_items.setText("Varor: 0 st")

    # ──────────────────────────────────────────────────────────────────
    # Slots
    # ──────────────────────────────────────────────────────────────────

    @pyqtSlot(dict)
    def _update_telemetry(self, data: dict):
        phase_names = {0: "IDLE", 1: "→ VARA", 2: "PICKUP", 3: "→ HEM", 4: "AVLÄMNING"}
        self.lbl_node.setText(f"Nod: {data['node']}")
        self.lbl_phase.setText(f"Fas: {phase_names.get(data['phase'], str(data['phase']))}")
        self.lbl_action.setText(
            f"Åtgärd: {data['action']}  →  {data['next_action']}  "
            f"({data['item_idx']+1}/{data['item_count']})"
        )
        self.lbl_direction.setText(f"Riktning: {data['direction']}")

        # Update robot marker on the map
        self.map_widget.set_robot_node(data['node'])

    @pyqtSlot(object)
    def _update_image(self, cv_img):
        h, w, ch = cv_img.shape
        q_img = QImage(
            cv_img.data, w, h, ch * w, QImage.Format.Format_RGB888
        ).rgbSwapped()
        self.video_label.setPixmap(QPixmap.fromImage(q_img))

    # ──────────────────────────────────────────────────────────────────
    # Keyboard control
    # ──────────────────────────────────────────────────────────────────

    def keyPressEvent(self, event):
        key = event.text().lower()

        # Joint selection (5-0 keys)
        joint_map = {'5': 0, '6': 1, '7': 2, '8': 3, '9': 4, '0': 5}
        if key in joint_map:
            self.selected_joint = joint_map[key]
            self.lbl_joint.setText(f"Vald joint: J{self.selected_joint + 1}")
            return

        # Camera selection
        if key == 'k':
            self.selected_joint = 6
            self.lbl_joint.setText("Vald joint: KAMERA")
            return

        # Arm movement (+, -, .)
        arm_moves = {'+': 'inc', '-': 'dec', '.': 'stop'}
        if key in arm_moves:
            action_byte = protocol.encode_arm_action(self.selected_joint, arm_moves[key])
            pkt = protocol.build_command_packet(self.current_state, 0x01, action_byte)
            self.udp_sock.sendto(pkt, (ROBOT_IP, UDP_PORT))
            return

        # Wheel control (WASD + diagonals)
        if key in ('w', 'a', 's', 'd', 'q', 'e'):
            pkt = protocol.build_command_packet(self.current_state, 0x00, key)
            self.udp_sock.sendto(pkt, (ROBOT_IP, UDP_PORT))

    # ──────────────────────────────────────────────────────────────────
    # Clean shutdown
    # ──────────────────────────────────────────────────────────────────

    def closeEvent(self, event):
        self.tele_thread.stop()
        self.tele_thread.wait(2000)
        self.video_thread.terminate()
        self.udp_sock.close()
        super().closeEvent(event)


if __name__ == "__main__":
    from PyQt6.QtWidgets import QApplication
    app = QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec())