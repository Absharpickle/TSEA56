import cv2
import numpy as np
import socket
import struct
from PyQt6.QtCore import QThread, pyqtSignal
from protocol import parse_telemetry


class VideoThread(QThread):
    """Receives and emits video frames from the Pi's UDP stream."""
    change_pixmap_signal = pyqtSignal(np.ndarray)

    def run(self):
        cap = cv2.VideoCapture("udp://0.0.0.0:5000", cv2.CAP_FFMPEG)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        while True:
            ret, frame = cap.read()
            if ret:
                self.change_pixmap_signal.emit(frame)


class TelemetryThread(QThread):
    """Listens for telemetry (0x06) and route (0x08) packets from the robot."""
    telemetry_signal = pyqtSignal(dict)
    route_signal = pyqtSignal(list)  # Emits list of node IDs for the planned route

    def __init__(self, sock):
        super().__init__()
        self.sock = sock
        self.running = True

    def run(self):
        self.sock.settimeout(1.0)
        
        while self.running:
            try:
                data, addr = self.sock.recvfrom(1024)

                # 0x06 Telemetry packet (14 or 18 bytes)
                telemetry_data = parse_telemetry(data)
                if telemetry_data:
                    self.telemetry_signal.emit(telemetry_data)

                # 0x08 Route packet: [0x08, count, node0, node1, ..., 0xFF]
                elif len(data) >= 3 and data[0] == 0x08 and data[-1] == 0xFF:
                    count = data[1]
                    if len(data) == count + 3:  # header + count + N nodes + footer
                        route = list(data[2:2 + count])
                        self.route_signal.emit(route)

            except socket.timeout:
                continue
            except Exception as e:
                print(f"[!] Telemetry Error: {e}") 

    def stop(self):
        self.running = False
