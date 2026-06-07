# ------------------------------------------------------
# Markus Hellers, Joel Eberhardsson - 28 maj 2026 - V1.0
# ------------------------------------------------------

## Trådklasser för att hantera videoströmning och telemetri i GUI:t ##

import cv2
import numpy as np
import socket
import struct
from PyQt6.QtCore import QThread, pyqtSignal
from protocol import parse_telemetry


# Tar och skickar videoframes från Pi:ns UDP-strömning
class VideoThread(QThread):
    change_pixmap_signal = pyqtSignal(np.ndarray)       # Signal som skickar videoframes som numpy-arrays

    def run(self):
        cap = cv2.VideoCapture("udp://0.0.0.0:5000", cv2.CAP_FFMPEG)    # Öppna UDP-strömmen
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)                             # Minska buffertstorleken för att minska latens
        
        while True:
            ret, frame = cap.read()                     # Läs en frame från strömmen
            if ret:                                     # Om en frame lästes korrekt...
                self.change_pixmap_signal.emit(frame)   # ...skicka frame till GUI:t via signalen


# Lyssnar på telemetri- och ruttdata från roboten via UDP
class TelemetryThread(QThread):
    telemetry_signal = pyqtSignal(dict) # Signal som skickar telemetridata som en dictionary
    route_signal = pyqtSignal(list)     # Signal som skickar ruttdata som en lista

    def __init__(self, sock):
        super().__init__()
        self.sock = sock    # UDP-socket som redan är bunden till rätt port
        self.running = True # Flagga för att stoppa tråden när GUI:t stängs

    def run(self):
        self.sock.settimeout(1.0) # Timeout för att kolla om tråden ska stoppas varje sekund
        
        while self.running:
            try:
                data, addr = self.sock.recvfrom(1024) # Ta emot data från roboten

                # 0x06 telemetripaket (14 or 18 bytes)
                telemetry_data = parse_telemetry(data)          # Parsa telemetridata
                if telemetry_data:                              # Om datat kunde parsas korrekt...
                    self.telemetry_signal.emit(telemetry_data)  # ...skicka telemetridata till GUI:t via signalen

                # 0x08 ruttpaket (olika längder beroende på antal noder)
                elif len(data) >= 3 and data[0] == 0x08 and data[-1] == 0xFF:   # Kontrollera att inkommande är ett ruttpaket
                    count = data[1]                                             # Antal noder i rutten
                    if len(data) == count + 3:                                  # Kontrollera att paketet har rätt längd baserat på antal noder
                        route = list(data[2:2 + count])                         # Ta ut ruttdata från paketet
                        self.route_signal.emit(route)                           # Skicka ruttdata till GUI:t via signalen

            except socket.timeout:  # Timeout triggat, kolla om tråden ska fortsätta köra
                continue            # Fortsätt loopa och vänta på nästa paket
            except Exception as e:  # Logga andra eventuella fel
                print(f"[!] Telemetry Error: {e}") 

    def stop(self):
        self.running = False # Sätt flaggan för att stoppa tråden
