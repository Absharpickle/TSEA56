import sys
import json
import socket
import threading
import urllib.request

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QHBoxLayout, QVBoxLayout,
    QGridLayout, QPushButton, QLabel, QGroupBox, QLineEdit,
    QSpinBox, QFrame
)
from PyQt5.QtCore import Qt, pyqtSignal, QObject
from PyQt5.QtGui import QPainter, QColor, QBrush, QPen, QFont, QPixmap

# ── Configuration ──────────────────────────────────────────────
ROBOT_IP   = "192.168.1.100"   # Change to Pi's IP
TCP_PORT   = 5000
MJPEG_PORT = 8080


# ── Grid Map Widget ─────────────────────────────────────────────
class GridMapWidget(QWidget):
    """Draws the 5x5 node map (nodes 1-25 + S/E) with robot, route, goods."""

    def __init__(self):
        super().__init__()
        self.setMinimumSize(320, 320)
        self.robot_pos = 1
        self.route: list = []
        self.goods: list = []

    def set_state(self, robot_pos: int, route: list, goods: list):
        self.robot_pos = robot_pos
        self.route     = route
        self.goods     = goods
        self.update()

    def _node_positions(self, cell: int, margin: int) -> dict:
        """
        From design spec fig 4:
          row 5 (top): 25 24 23 22 21
          row 4:       20 19 18 17 16
          row 3:       15 14 13 12 11
          row 2:       10  9  8  7  6
          row 1 (bot):  5  4  3  2  1
        """
        pos = {}
        for r in range(5):
            for c in range(5):
                node = r * 5 + c + 1
                x = margin + c * cell + cell // 2
                y = margin + (4 - r) * cell + cell // 2
                pos[node] = (x, y)
        return pos

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        cell   = min(self.width(), self.height()) // 6
        margin = cell // 2
        npos   = self._node_positions(cell, margin)

        # Draw edges
        painter.setPen(QPen(QColor(70, 71, 90), 2))
        for node, (x, y) in npos.items():
            r = (node - 1) // 5
            c = (node - 1) % 5
            if c < 4:
                nx, ny = npos[node + 1]
                painter.drawLine(x, y, nx, ny)
            if r < 4:
                nx, ny = npos[node + 5]
                painter.drawLine(x, y, nx, ny)

        # Draw route highlight
        if len(self.route) > 1:
            painter.setPen(QPen(QColor(100, 180, 255, 180), 4))
            for i in range(len(self.route) - 1):
                a, b = self.route[i], self.route[i + 1]
                if a in npos and b in npos:
                    painter.drawLine(*npos[a], *npos[b])

        # Draw nodes
        for node, (x, y) in npos.items():
            if node == self.robot_pos:
                color = QColor(0, 200, 100)
            elif node in self.route:
                color = QColor(100, 180, 255)
            elif node in self.goods:
                color = QColor(255, 180, 0)
            else:
                color = QColor(80, 80, 100)

            painter.setBrush(QBrush(color))
            painter.setPen(QPen(QColor(30, 30, 46), 1))
            painter.drawEllipse(x - 13, y - 13, 26, 26)
            painter.setPen(Qt.white)
            painter.setFont(QFont("Arial", 8, QFont.Bold))
            painter.drawText(x - 10, y + 4, str(node))

        # S / E labels
        sx, sy = margin - cell // 2, margin + 4 * cell + cell // 2
        ex, ey = margin + 5 * cell,  margin
        for lbl, lx, ly in [("S", sx, sy), ("E", ex, ey)]:
            painter.setPen(QColor(250, 179, 135))
            painter.setFont(QFont("Arial", 10, QFont.Bold))
            painter.drawText(lx - 6, ly + 5, lbl)


# ── TCP Worker ──────────────────────────────────────────────────
class TcpWorker(QObject):
    telemetry_received = pyqtSignal(dict)
    connection_status  = pyqtSignal(bool)

    def __init__(self, ip: str, port: int):
        super().__init__()
        self.ip      = ip
        self.port    = port
        self.sock    = None
        self.running = False

    def connect(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.ip, self.port))
            self.sock.settimeout(1.0)
            self.running = True
            self.connection_status.emit(True)
            threading.Thread(target=self._receive_loop, daemon=True).start()
        except Exception as e:
            print(f"[TCP] Connection failed: {e}")
            self.connection_status.emit(False)

    def send_command(self, cmd: dict):
        if self.sock and self.running:
            try:
                self.sock.sendall((json.dumps(cmd) + "\n").encode())
            except Exception:
                self.connection_status.emit(False)

    def _receive_loop(self):
        buf = ""
        while self.running:
            try:
                data = self.sock.recv(1024).decode()
                buf += data
                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    if line.strip():
                        try:
                            self.telemetry_received.emit(json.loads(line))
                        except Exception:
                            pass
            except socket.timeout:
                continue
            except Exception:
                self.connection_status.emit(False)
                break

    def disconnect(self):
        self.running = False
        if self.sock:
            self.sock.close()


# ── Camera Worker ───────────────────────────────────────────────
class CameraWorker(QObject):
    frame_ready = pyqtSignal(QPixmap)

    def __init__(self, url: str):
        super().__init__()
        self.url     = url
        self.running = False

    def start(self):
        self.running = True
        threading.Thread(target=self._stream_loop, daemon=True).start()

    def stop(self):
        self.running = False

    def _stream_loop(self):
        try:
            stream = urllib.request.urlopen(self.url, timeout=5)
            buf    = b""
            while self.running:
                buf += stream.read(1024)
                s = buf.find(b'\xff\xd8')
                e = buf.find(b'\xff\xd9')
                if s != -1 and e != -1:
                    jpg = buf[s:e + 2]
                    buf = buf[e + 2:]
                    px  = QPixmap()
                    px.loadFromData(jpg)
                    if not px.isNull():
                        self.frame_ready.emit(px)
        except Exception as ex:
            print(f"[Camera] Stream error: {ex}")


# ── Stylesheet ──────────────────────────────────────────────────
STYLE = """
QMainWindow, QWidget  { background-color: #1e1e2e; color: #cdd6f4; font-family: Arial; font-size: 13px; }
QPushButton           { background-color: #313244; color: #cdd6f4; border: 1px solid #45475a;
                        border-radius: 6px; padding: 8px 12px; }
QPushButton:hover     { background-color: #45475a; }
QPushButton:pressed   { background-color: #89b4fa; color: #1e1e2e; }
QPushButton:disabled  { background-color: #181825; color: #585b70; }
QGroupBox             { border: 1px solid #45475a; border-radius: 8px; margin-top: 10px;
                        padding: 8px; color: #89b4fa; font-weight: bold; }
QGroupBox::title      { subcontrol-origin: margin; left: 10px; padding: 0 4px; }
QLineEdit, QSpinBox   { background-color: #313244; border: 1px solid #45475a;
                        border-radius: 4px; padding: 4px; color: #cdd6f4; }
QFrame[frameShape="4"]{ color: #45475a; }
"""


# ── Main Window ─────────────────────────────────────────────────
class RobotGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Lagerhållningsrobot – Control Panel")
        self.setMinimumSize(1150, 780)
        self.setStyleSheet(STYLE)

        self.tcp    = TcpWorker(ROBOT_IP, TCP_PORT)
        self.camera = CameraWorker(f"http://{ROBOT_IP}:{MJPEG_PORT}/stream")

        self.tcp.telemetry_received.connect(self._on_telemetry)
        self.tcp.connection_status.connect(self._on_connection_status)
        self.camera.frame_ready.connect(self._on_frame)

        self._build_ui()

    # ── UI Construction ─────────────────────────────────────────
    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QHBoxLayout(central)
        root.setSpacing(12)
        root.setContentsMargins(12, 12, 12, 12)

        left = QVBoxLayout()
        left.setSpacing(10)
        left.addWidget(self._panel_connection())
        left.addWidget(self._panel_mode())
        left.addWidget(self._panel_drive())
        left.addWidget(self._panel_arm())
        left.addStretch()

        right = QVBoxLayout()
        right.setSpacing(10)
        right.addWidget(self._panel_camera())
        right.addWidget(self._panel_map())

        root.addLayout(left, 38)
        root.addLayout(right, 62)

    def _panel_connection(self) -> QGroupBox:
        g  = QGroupBox("Connection")
        lo = QHBoxLayout(g)
        self.ip_input     = QLineEdit(ROBOT_IP)
        self.connect_btn  = QPushButton("Connect")
        self.status_label = QLabel("● Disconnected")
        self.status_label.setStyleSheet("color: #f38ba8;")
        self.connect_btn.clicked.connect(self._on_connect)
        lo.addWidget(QLabel("IP:"))
        lo.addWidget(self.ip_input)
        lo.addWidget(self.connect_btn)
        lo.addWidget(self.status_label)
        return g

    def _panel_mode(self) -> QGroupBox:
        g  = QGroupBox("Mode")
        lo = QHBoxLayout(g)
        m  = QPushButton("🕹  Manual")
        a  = QPushButton("🤖  Autonomous")
        m.clicked.connect(lambda: self._send({"type": "mode", "cmd": "manual"}))
        a.clicked.connect(lambda: self._send({"type": "mode", "cmd": "autonomous"}))
        lo.addWidget(m)
        lo.addWidget(a)
        return g

    def _panel_drive(self) -> QGroupBox:
        g    = QGroupBox("Drive Control")
        grid = QGridLayout(g)
        grid.setSpacing(5)
        btns = [
            ("↖", 0, 0, "fwd_left"),  ("↑ Fwd",  0, 1, "fwd"),  ("↗", 0, 2, "fwd_right"),
            ("← Left", 1, 0, "left"), ("■ Stop",  1, 1, "stop"), ("→ Right", 1, 2, "right"),
            ("↙", 2, 0, "bwd_left"),  ("↓ Bwd",  2, 1, "bwd"),  ("↘", 2, 2, "bwd_right"),
        ]
        for label, row, col, cmd in btns:
            btn = QPushButton(label)
            if cmd == "stop":
                btn.setStyleSheet(
                    "background-color:#f38ba8;color:#1e1e2e;font-weight:bold;border-radius:6px;padding:8px;")
            btn.clicked.connect(lambda _, c=cmd: self._send({"type": "drive", "cmd": c}))
            grid.addWidget(btn, row, col)
        return g

    def _panel_arm(self) -> QGroupBox:
        g  = QGroupBox("Arm Control")
        lo = QVBoxLayout(g)

        xyz = QGridLayout()
        for i, axis in enumerate(["X", "Y", "Z"]):
            minus = QPushButton("−")
            plus  = QPushButton("+")
            minus.setFixedWidth(38)
            plus.setFixedWidth(38)
            minus.clicked.connect(lambda _, a=axis: self._send({"type": "arm", "cmd": f"{a.lower()}-"}))
            plus.clicked.connect( lambda _, a=axis: self._send({"type": "arm", "cmd": f"{a.lower()}+"}))
            xyz.addWidget(QLabel(f"{axis}:"), i, 0)
            xyz.addWidget(minus, i, 1)
            xyz.addWidget(plus,  i, 2)
        lo.addLayout(xyz)
        lo.addWidget(self._separator())

        actions = [
            ("Base ↺", "base_ccw"), ("Base ↻", "base_cw"),
            ("Claw ↺", "claw_ccw"), ("Claw ↻", "claw_cw"),
            ("Tilt ↑",  "tilt_up"), ("Tilt ↓", "tilt_down"),
            ("🤏 Open", "claw_open"), ("✊ Close", "claw_close"),
        ]
        ag = QGridLayout()
        for i, (lbl, cmd) in enumerate(actions):
            btn = QPushButton(lbl)
            btn.clicked.connect(lambda _, c=cmd: self._send({"type": "arm", "cmd": c}))
            ag.addWidget(btn, i // 2, i % 2)
        lo.addLayout(ag)
        lo.addWidget(self._separator())

        mission = QHBoxLayout()
        self.target_spin = QSpinBox()
        self.target_spin.setRange(1, 25)
        self.target_spin.setPrefix("Node: ")
        go_btn = QPushButton("▶  Start Mission")
        go_btn.setStyleSheet(
            "background-color:#a6e3a1;color:#1e1e2e;font-weight:bold;border-radius:6px;padding:8px;")
        go_btn.clicked.connect(self._start_mission)
        mission.addWidget(self.target_spin)
        mission.addWidget(go_btn)
        lo.addLayout(mission)
        return g

    def _panel_camera(self) -> QGroupBox:
        g  = QGroupBox("Camera Feed")
        lo = QVBoxLayout(g)
        self.camera_label = QLabel("No signal")
        self.camera_label.setAlignment(Qt.AlignCenter)
        self.camera_label.setMinimumHeight(240)
        self.camera_label.setStyleSheet(
            "background-color:#11111b;border-radius:6px;color:#585b70;")
        lo.addWidget(self.camera_label)
        return g

    def _panel_map(self) -> QGroupBox:
        g  = QGroupBox("Grid Map")
        lo = QVBoxLayout(g)

        legend = QHBoxLayout()
        for color, text in [("#00c864","Robot"),("#64b4ff","Route"),("#ffb400","Goods"),("#505064","Node")]:
            dot = QLabel("●")
            dot.setStyleSheet(f"color:{color};font-size:15px;")
            legend.addWidget(dot)
            legend.addWidget(QLabel(text))
            legend.addSpacing(10)
        legend.addStretch()
        lo.addLayout(legend)

        self.grid_map = GridMapWidget()
        lo.addWidget(self.grid_map)
        return g

    @staticmethod
    def _separator() -> QFrame:
        f = QFrame()
        f.setFrameShape(QFrame.HLine)
        return f

    # ── Slots ────────────────────────────────────────────────────
    def _on_connect(self):
        ip = self.ip_input.text().strip()
        self.tcp.ip        = ip
        self.camera.url    = f"http://{ip}:{MJPEG_PORT}/stream"
        self.tcp.connect()
        self.camera.start()

    def _send(self, cmd: dict):
        self.tcp.send_command(cmd)

    def _start_mission(self):
        target = self.target_spin.value()
        self._send({"type": "mode",    "cmd": "autonomous"})
        self._send({"type": "mission", "target": target})

    def _on_telemetry(self, data: dict):
        if data.get("type") == "telemetry":
            self.grid_map.set_state(
                data.get("position", self.grid_map.robot_pos),
                data.get("route",    self.grid_map.route),
                data.get("goods",    self.grid_map.goods),
            )

    def _on_connection_status(self, connected: bool):
        if connected:
            self.status_label.setText("● Connected")
            self.status_label.setStyleSheet("color: #a6e3a1;")
        else:
            self.status_label.setText("● Disconnected")
            self.status_label.setStyleSheet("color: #f38ba8;")

    def _on_frame(self, pixmap: QPixmap):
        scaled = pixmap.scaled(
            self.camera_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.camera_label.setPixmap(scaled)

    def closeEvent(self, event):
        self.tcp.disconnect()
        self.camera.stop()
        event.accept()


# ── Entry point ─────────────────────────────────────────────────
if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = RobotGUI()
    win.show()
    sys.exit(app.exec_())
