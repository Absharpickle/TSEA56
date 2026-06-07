# ------------------------------------------------------
# Markus Hellers, Joel Eberhardsson - 28 maj 2026 - V1.0
# ------------------------------------------------------

## Huvudfönstret för GUI:t som innehåller videoflödet, nodkartan, kontrollpanelen och telemetridashboarden ##

import cv2
import socket
from PyQt6.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
                             QLabel, QComboBox, QFrame, QGridLayout,
                             QSizePolicy, QPushButton, QListWidget)
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QImage, QPixmap

from threads import VideoThread, TelemetryThread
from map_widget import MapFrame
from protocol import build_command_packet, build_item_list_packet, build_arm_command_packet, build_reset_packet

IP_ADDRESS_HOME = "192.168.1.50"    # För testning hemma
IP_ADDRESS_SITE = "10.42.0.1"       # För testning på plats


# MainWindow är det centrala fönstret i GUI:t som hanterar alla widgets, trådar och nätverkskommunikation
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PiCam Ground Control")
        self.setStyleSheet("""
            QMainWindow { background-color: #2c3e50; color: white; }
            QLabel { color: white; }
        """)
        
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus) # För att kunna fånga keyPressEvent även när andra widgets är fokuserade
        self.current_joint = 1                          # Aktiv joint (1-6)

        # -- LAYOUT OCH WIDGETS ---
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

        # --- KARTA + OBJEKT ---
        right_vlayout = QVBoxLayout()
        right_vlayout.setSpacing(5)

        self._setup_map(right_vlayout)
        self._setup_item_panel(right_vlayout)

        self.top_hlayout.addLayout(right_vlayout)

        # --- NÄTVERK ---
        #self.pi_ip = IP_ADDRESS_HOME   # Testning hemma
        self.pi_ip = IP_ADDRESS_SITE    # Testning på plats
        self.pi_port = 5001             # Port för att skicka kommandon till roboten
        self.control_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.control_sock.bind(("0.0.0.0", 0))

        # -- INTERNA DATA OCH TILLSTÅND ---
        self.item_edges = []

        self._setup_controls()   # Kontrollpanelen med dropdowns och knappar
        self._setup_dashboard()  # Telemetridashboard som visar live-data från roboten
        self._start_threads()    # Starta bakgrundstrådarna för video och telemetri

        self.phase_names = {0: "IDLE", 1: "TO ITEM", 2: "PICKUP", 3: "TO HOME", 4: "DROP"}  # För att översätta phase-nummer till namn i dashboarden
        self.update_map_highlights()                                                        # Uppdatera kartan med eventuella varor som redan finns i listan

    # =================================================================
    # SETUP-FUNKTIONER FÖR OLIKA DELAR AV GUI:T
    # =================================================================
    
    # Skapa kartan med nodwidgets
    def _setup_map(self, parent_layout):
        self.grid_nodes = {}
        self.current_active_node = 25 # Startnoden aktiv från början

        self.map_frame = MapFrame(self.grid_nodes)                  # Skapa MapFrame och skicka referens till grid_nodes så den kan rita ut kanterna korrekt
        self.map_frame.edge_clicked.connect(self._on_edge_clicked)  # Koppla signalen för klick på kanter i kartan till en hanterare i MainWindow
        self.map_layout = QGridLayout(self.map_frame)               # Använd en QGridLayout för att placera nodwidgets i nodkartan
        self.map_layout.setSpacing(10)

        self.node_style_idle = """
            background-color: #95a5a6; color: black; font-weight: bold;
            border-radius: 20px; font-size: 13px;
        """ # Stil för inaktiva nodwidgets
        self.node_style_active = """
            background-color: #e74c3c; color: white; font-weight: bold;
            border-radius: 20px; font-size: 14px; border: 2px solid #f1c40f;
        """ # Stil för aktiv nodwidget

        # Startnoden (25) som är utanför 5x5-griden
        self.lbl_start = QLabel("25")
        self.lbl_start.setFixedSize(40, 40)
        self.lbl_start.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_start.setStyleSheet(self.node_style_active)
        self.map_layout.addWidget(self.lbl_start, 0, 0, alignment=Qt.AlignmentFlag.AlignCenter)
        self.grid_nodes[25] = self.lbl_start

        # Övriga noder i 5x5-griden (0-24)
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

    # Skapa panelen för att lägga till och ta bort varor i kartan
    def _setup_item_panel(self, parent_layout):
        combo_style = "background-color: #ecf0f1; color: #2c3e50; padding: 3px 6px; font-size: 12px; font-weight: bold; border-radius: 3px;"
        btn_style = "background-color: #27ae60; color: white; font-weight: bold; padding: 3px 8px; border-radius: 3px; font-size: 12px;"
        btn_remove_style = "background-color: #c0392b; color: white; font-weight: bold; padding: 3px 8px; border-radius: 3px; font-size: 12px;"

        item_hlayout = QHBoxLayout()    # Horisontell layout för varupanelen
        item_hlayout.setSpacing(6)      # Mellanrum mellan widgets i varupanelen

        self.edge_combo = QComboBox()               # Dropdown för att välja kant (varuplats) att lägga till
        self.edge_combo.setStyleSheet(combo_style)  # Stil för dropdown-menyn
        self.edge_combo.setMinimumWidth(120)        # Minsta bredd för dropdown-menyn
        for i in range(25):                         # Loop över alla noder i 5x5-griden för att lägga till deras kanter i dropdown-menyn
            kol = i % 5
            rad = i // 5
            if kol < 4:
                self.edge_combo.addItem(f"{i}↔{i+1}", (i, i+1))                 # Kant till höger
            if rad < 4:
                self.edge_combo.addItem(f"{i}↔{i+5}", (i, i+5))                 # Kant nedåt
        self.edge_combo.currentIndexChanged.connect(lambda: self.setFocus())    # Se till att dropdown-menyn inte tar fokus från keyPressEvent i MainWindow

        self.btn_add = QPushButton("+")                     # Knapp för att lägga till vald kant som en vara i kartan
        self.btn_add.setStyleSheet(btn_style)               # Stil för "lägg till"-knappen
        self.btn_add.setFixedWidth(30)                      # Bredd för "lägg till"-knappen
        self.btn_add.clicked.connect(self.add_item_edge)    # Koppla klick på "lägg till"-knappen till funktionen add_item_edge som lägger till vald kant i item_edges och uppdaterar kartan

        self.item_list_widget = QListWidget()               # Widget som visar listan över aktuella varor i kartan
        self.item_list_widget.setStyleSheet(
            "background-color: #34495e; color: #ecf0f1; font-size: 12px; "
            "font-weight: bold; border-radius: 3px; padding: 2px;"
        )                                                   # Stil för listan över varor
        self.item_list_widget.setMaximumHeight(55)
        self.item_list_widget.setMinimumWidth(160)

        self.btn_remove = QPushButton("−")                          # Knapp för att ta bort den valda varan från kartan
        self.btn_remove.setStyleSheet(btn_remove_style)             # Stil för "ta bort"-knappen
        self.btn_remove.setFixedWidth(30)                           # Bredd för "ta bort"-knappen
        self.btn_remove.clicked.connect(self.remove_selected_item)  # Koppla klick på "ta bort"-knappen till funktionen remove_selected_item som tar bort den valda varan från item_edges och uppdaterar kartan

        self.btn_clear = QPushButton("Clr")                         # Knapp för att rensa alla varor från kartan
        self.btn_clear.setStyleSheet(btn_remove_style)              # Stil för "rensa"-knappen
        self.btn_clear.setFixedWidth(35)                            # Bredd för "rensa"-knappen
        self.btn_clear.clicked.connect(self.clear_item_list)        # Koppla klick på "rensa"-knappen till funktionen clear_item_list som tömmer item_edges och uppdaterar kartan

        item_hlayout.addWidget(self.edge_combo)                     # Dropdown-menyn för att välja kant i varupanelen
        item_hlayout.addWidget(self.btn_add)                        # Lägg till-knappen
        item_hlayout.addWidget(self.item_list_widget, 1)            # Listan med varor som ska plockas
        item_hlayout.addWidget(self.btn_remove)                     # Ta bort-knappen
        item_hlayout.addWidget(self.btn_clear)                      # Rensa-knappen

        parent_layout.addLayout(item_hlayout)                       # Varupanel

    # Skapa kontrollpanelen för att styra roboten och se dess tillstånd
    def _setup_controls(self):
        control_label_style = "font-size: 13px; font-weight: bold; color: #ecf0f1;"
        combo_style = "background-color: #ecf0f1; color: #2c3e50; padding: 3px 6px; font-size: 12px; font-weight: bold; border-radius: 3px;"

        self.control_layout = QHBoxLayout()
        self.control_layout.setSpacing(8)
        self.control_layout.setContentsMargins(20, 3, 20, 3)
        self.layout.addLayout(self.control_layout)

        lbl_state = QLabel("State:")                        # Label för att visa robotens tillstånd för styrning och feedback
        lbl_state.setStyleSheet(control_label_style)
        lbl_state.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)

        self.state_combo = QComboBox()                      # Dropdown-meny för att välja robotens tillstånd
        self.state_combo.setStyleSheet(combo_style)
        self.state_combo.setMinimumWidth(160)
        self.state_combo.addItem("1: (Auto, Auto)", 0)
        self.state_combo.addItem("2: (Auto, Manual)", 1)
        self.state_combo.addItem("3: (Manual, Auto)", 2)
        self.state_combo.addItem("4: (Manual, Manual)", 3)
        self.state_combo.setCurrentIndex(3)
        self.state_combo.currentIndexChanged.connect(lambda: self.setFocus())

        self.control_layout.addWidget(lbl_state)            # State label
        self.control_layout.addWidget(self.state_combo)     # State dropdown
        self.control_layout.addStretch()

        lbl_target = QLabel("Target:")                      # Target label
        lbl_target.setStyleSheet(control_label_style)
        lbl_target.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)

        self.target_combo = QComboBox()                     # Dropdown-meny för att välja target
        self.target_combo.setStyleSheet(combo_style)
        self.target_combo.setMinimumWidth(100)
        self.target_combo.addItem("Wheel", "wheel")        # Wheel target
        self.target_combo.addItem("Arm", "arm")            # Arm target
        self.target_combo.currentIndexChanged.connect(lambda: self.setFocus())

        self.control_layout.addWidget(lbl_target)           # Target label
        self.control_layout.addWidget(self.target_combo)    # Target dropdown
        self.control_layout.addStretch()

        lbl_keys = QLabel("Keys: [1234] State | [TA] Target | [↑↓←→SEOB] Wheel | [VH] Arm | [SPACE] Reset") # Legend för keybindings
        lbl_keys.setStyleSheet("color: #7f8c8d; font-size: 11px;")
        self.control_layout.addWidget(lbl_keys)

    # Telemetri dashboard som visar robotens aktuella data
    def _setup_dashboard(self):
        self.dashboard_frame = QFrame()
        self.dashboard_frame.setStyleSheet("QFrame { background-color: #34495e; border-radius: 4px; }")
        self.dashboard_layout = QHBoxLayout(self.dashboard_frame)
        self.dashboard_layout.setContentsMargins(8, 4, 8, 4)

        # Labels för aktuell data
        self.lbl_phase = QLabel("Phase: IDLE")
        self.lbl_action = QLabel("Last Action: -")
        self.lbl_next_action = QLabel("Next Action: -")
        self.lbl_line = QLabel("Line: 0")
        self.lbl_gyro = QLabel("Gyro: (0, 0)")
        self.lbl_flags = QLabel("Flags: 0")
        self.lbl_items_progress = QLabel("Items: -")
        self.lbl_action_done = QLabel("Done: ●")
        self.lbl_action_done.setStyleSheet("font-size: 13px; font-weight: bold; color: #e74c3c; padding: 3px;")
        self.lbl_gas = QLabel("Gas: R0 L0")
        self.lbl_claw = QLabel("Claw: R0 Z0")

        # Lägger till alla labels till dashboard layouten
        for lbl in [self.lbl_phase, self.lbl_action, self.lbl_next_action,
                     self.lbl_line, self.lbl_gyro, self.lbl_flags,
                     self.lbl_items_progress, self.lbl_gas, self.lbl_claw]:
            lbl.setStyleSheet("font-size: 12px; font-weight: bold; color: #ecf0f1; padding: 3px;")
            self.dashboard_layout.addWidget(lbl)

        self.dashboard_layout.addWidget(self.lbl_action_done)

        self.layout.addWidget(self.dashboard_frame)

    # Startar video och telemetri trådar
    def _start_threads(self):
        # Videotråd för att visa bild i realtid från roboten
        self.video_thread = VideoThread()
        self.video_thread.change_pixmap_signal.connect(self.update_image)
        self.video_thread.start()

        # Telemetritråd för att ta emot data från roboten
        self.telemetry_thread = TelemetryThread(self.control_sock)
        self.telemetry_thread.telemetry_signal.connect(self.update_telemetry_dashboard)
        self.telemetry_thread.route_signal.connect(self.update_route)
        self.telemetry_thread.start()

    # Lägger till kanter i listan över varor som ska plockas från dropdown-menyn
    def add_item_edge(self, edge=None):
        if edge is None:
            edge = self.edge_combo.currentData()
        if edge and edge not in self.item_edges:
            self.item_edges.append(edge)
            self.refresh_item_list_widget()
            self.update_map_highlights()
        self.setFocus()

    # Lägger till kanter i listan över varor som ska plockas från klick på kartan
    def _on_edge_clicked(self, edge):
        if edge in self.item_edges:
            self.item_edges.remove(edge)
            self.refresh_item_list_widget()
            self.update_map_highlights()
        else:
            self.add_item_edge(edge)
        self.setFocus()

    # Tar bort markerad kant från listan över varor som ska plockas
    def remove_selected_item(self):
        row = self.item_list_widget.currentRow()
        if 0 <= row < len(self.item_edges):
            self.item_edges.pop(row)
            self.refresh_item_list_widget()
            self.update_map_highlights()
        self.setFocus()

    # Rensar listan över varor som ska plockas
    def clear_item_list(self):
        self.item_edges.clear()
        self.refresh_item_list_widget()
        self.update_map_highlights()
        self.setFocus()

    # Uppdaterar listan över varor som ska plockas
    def refresh_item_list_widget(self):
        self.item_list_widget.clear()
        for idx, (u, v) in enumerate(self.item_edges):
            self.item_list_widget.addItem(f"  {idx+1}. Edge {u} ↔ {v}")

    # Uppdaterar markerade kanter på kartan baserat på listan över varor som ska plockas
    def update_map_highlights(self):
        self.map_frame.set_item_edges(self.item_edges)

    # Uppdaterar gränssnittets variabler med aktuell data från roboten
    def update_telemetry_dashboard(self, data):
        phase_str = self.phase_names.get(data['phase'], "UNKNOWN")
        self.lbl_phase.setText(f"Phase: {phase_str}")
        self.lbl_action.setText(f"Action: '{data['action']}'")
        self.lbl_next_action.setText(f"Next Action: '{data['next_action']}'")
        self.lbl_line.setText(f"Line: {data['line_var']}")
        self.lbl_gyro.setText(f"Gyro: ({data['gyro1']}, {data['gyro2']})")
        self.lbl_flags.setText(f"Flags: {data['flags']}")

        # Action done från styrmodulen
        done = data.get('action_done', 0)
        if done:
            self.lbl_action_done.setStyleSheet("font-size: 13px; font-weight: bold; color: #2ecc71; padding: 3px;")
            self.lbl_action_done.setText("Done: ● YES")
        else:
            self.lbl_action_done.setStyleSheet("font-size: 13px; font-weight: bold; color: #e74c3c; padding: 3px;")
            self.lbl_action_done.setText("Done: ● NO")

        item_idx = data.get('current_item', 0)        # Nuvarande vara som ska plockas
        item_total = data.get('item_count', 0)        # Totalt antal varor
        if item_total > 0:                            # Visar hur många varor som ska plockas och vilken som är nuvarande
            self.lbl_items_progress.setText(f"Items: {min(item_idx+1, item_total)}/{item_total}")
        else:
            self.lbl_items_progress.setText("Items: -")

        # Rensar ruttvisningen när roboten går in i IDLE-fasen
        if data['phase'] == 0 and self.map_frame.route_nodes:
            self.map_frame.set_route([])

        # Data från styrmodulen
        self.lbl_gas.setText(f"Gas: R{data.get('gas_right', 0)} L{data.get('gas_left', 0)}")
        self.lbl_claw.setText(f"Claw: R{data.get('claw_pos_r', 0)} Z{data.get('claw_pos_z', 0)}")

        node = data.get('current_node', 25)        # Nuvarande nod
        direction = data.get('direction', 's')     # Nuvarande riktning
        if node != self.current_active_node or direction != getattr(self, 'current_dir', 's'): 
            self.current_dir = direction
            self.set_active_node(node, direction)

    # Uppdaterar ruttvisningen när roboten skickar en rutt i ett 0x08-paket
    def update_route(self, route_nodes):
        self.map_frame.set_route(route_nodes)

    # Markera noden som roboten befinner sig vid med pil i aktuell riktning
    def set_active_node(self, node_id, direction='s'):
        dir_arrows = {'n': '↑', 's': '↓', 'e': '→', 'w': '←'}
        arrow = dir_arrows.get(direction, '')

        if self.current_active_node in self.grid_nodes:     # Återställer utseendet på den tidigare aktiva noden
            old_lbl = self.grid_nodes[self.current_active_node]
            old_lbl.setStyleSheet(self.node_style_idle)
            old_lbl.setText(str(self.current_active_node))

        if node_id in self.grid_nodes:                      # Sätter utseendet på den nya aktiva noden
            lbl = self.grid_nodes[node_id]
            lbl.setStyleSheet(self.node_style_active)
            lbl.setText(f"{arrow}")
            self.current_active_node = node_id

    # Tangentkommandon
    def keyPressEvent(self, event):
        key = event.key()

        # Global reset med mellanslag
        if key == Qt.Key.Key_Space:
            self.perform_reset()
            return

        target = self.target_combo.currentData()

        # När armen är target, 1-6 för val av led, upp/ner för att öka/minska gradvis
        if target == "arm":
            joint_keys = {
                Qt.Key.Key_1: 1, Qt.Key.Key_2: 2, Qt.Key.Key_3: 3,
                Qt.Key.Key_4: 4, Qt.Key.Key_5: 5, Qt.Key.Key_6: 6,
            }
            if key in joint_keys:
                self.current_joint = joint_keys[key]
                print(f"[ARM] Led {self.current_joint} vald")   # visa vald led i terminalen
                return
            if key == Qt.Key.Key_Up:
                self.send_arm_packet(1)  # öka
                return
            elif key == Qt.Key.Key_Down:
                self.send_arm_packet(2)  # minska
                return
        else:
            # State hotkeys (endast när wheel är target)
            if key == Qt.Key.Key_1: self.state_combo.setCurrentIndex(0); return
            elif key == Qt.Key.Key_2: self.state_combo.setCurrentIndex(1); return
            elif key == Qt.Key.Key_3: self.state_combo.setCurrentIndex(2); return
            elif key == Qt.Key.Key_4: self.state_combo.setCurrentIndex(3); return

        # Target hotkeys (endast när wheel är target)
        if key == Qt.Key.Key_T: self.target_combo.setCurrentIndex(0); return
        elif key == Qt.Key.Key_A: self.target_combo.setCurrentIndex(1); return

        # Wheel action hotkeys
        action_char = None
        if target == "wheel":
            key_map = {
                Qt.Key.Key_Up: 'f', Qt.Key.Key_Down: 'b',
                Qt.Key.Key_Right: 'r', Qt.Key.Key_Left: 'l',
                Qt.Key.Key_S: 's', Qt.Key.Key_E: 'e',
                Qt.Key.Key_O: 'o', Qt.Key.Key_B: 'b',
            }
            action_char = key_map.get(key)  # plocka fram action-karaktär baserat på tangent

        if action_char:
            self.send_packet(action_char)   # skicka action till styrmodulen

    def keyReleaseEvent(self, event):
        key = event.key()
        target = self.target_combo.currentData()

        if target == "arm" and key in (Qt.Key.Key_Up, Qt.Key.Key_Down):
            if not event.isAutoRepeat():
                self.send_arm_packet(0)  # stop

    # Skickar standardpaket till styrmodulen 
    def send_packet(self, action_char):
        current_state = self.state_combo.currentData()
        current_target = self.target_combo.currentData()

        # I autoläge, skicka varulista innan startkommandot
        if current_state in (0x00, 0x01) and action_char == 'f' and self.item_edges:
            packet = build_item_list_packet(self.item_edges)
            if packet:
                try:
                    self.control_sock.sendto(packet, (self.pi_ip, self.pi_port))
                    print(f"[ITEMS] Sent {len(self.item_edges)} item(s) to robot")  # Skriver ut i terminalen hur många objekt som skickats
                except Exception as e:
                    print(f"Error sending item list: {e}")                          # Felhantering om det ej går att skicka paket

        packet = build_command_packet(current_state, current_target, action_char)   # Bygger själva kommando-paketet (state, target och action)
        try:
            self.control_sock.sendto(packet, (self.pi_ip, self.pi_port))            # Skickar kommandot till styrmodulen
        except Exception as e:
            print(f"Error sending packet: {e}")                                     # Felhantering om det ej går att skicka ett paket

    # Skickar armpaket till styrmodulen
    def send_arm_packet(self, direction):
        current_state = self.state_combo.currentData() 
        packet = build_arm_command_packet(current_state, self.current_joint, direction)   # Bygger arm-paketet (state, joint och direction)
        try:
            self.control_sock.sendto(packet, (self.pi_ip, self.pi_port))                  # Skickar arm-kommandot till styrmodulen
        except Exception as e:
            print(f"Error sending arm packet: {e}")                                       # Felhantering om det ej går att skicka arm-kommandot

    # Reset för GUI och roboten
    def perform_reset(self):
        # Skickar reset packet till roboten
        packet = build_reset_packet()
        try:
            self.control_sock.sendto(packet, (self.pi_ip, self.pi_port))
            print("[RESET] Sent reset packet to robot")
        except Exception as e:
            print(f"Error sending reset packet: {e}")

        # Återställer varulistan
        self.item_edges.clear()
        self.refresh_item_list_widget()

        # Återställer kartan
        self.map_frame.set_route([])
        self.map_frame.set_item_edges([])
        self.set_active_node(25, 's')

        # Återställer combo-rutorna
        self.state_combo.setCurrentIndex(3)   # Manual, Manual
        self.target_combo.setCurrentIndex(0)  # Wheel
        self.current_joint = 1

        # Återställer dashboard-etiketterna
        self.lbl_phase.setText("Phase: IDLE")
        self.lbl_action.setText("Last Action: -")
        self.lbl_next_action.setText("Next Action: -")
        self.lbl_line.setText("Line: 0")
        self.lbl_gyro.setText("Gyro: (0, 0)")
        self.lbl_flags.setText("Flags: 0")
        self.lbl_items_progress.setText("Items: -")
        self.lbl_action_done.setStyleSheet("font-size: 13px; font-weight: bold; color: #e74c3c; padding: 3px;")
        self.lbl_action_done.setText("Done: ●")
        self.lbl_gas.setText("Gas: R0 L0")
        self.lbl_claw.setText("Claw: R0 Z0")

        self.setFocus()
        print("[RESET] GUI state cleared")    # Skriver ut i terminalen att GUI:et har återställts

    # Uppdaterar kameraströmmen
    def update_image(self, cv_img):
        rgb = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        qt_img = QImage(rgb.data, w, h, ch * w, QImage.Format.Format_RGB888)
        self.image_label.setPixmap(QPixmap.fromImage(qt_img))

    # Avslutar programmet
    def closeEvent(self, event):
        self.telemetry_thread.stop()
        self.telemetry_thread.wait()
        super().closeEvent(event)
