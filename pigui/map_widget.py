# ------------------------------------------------------
# Markus Hellers, Joel Eberhardsson - 28 maj 2026 - V1.0
# ------------------------------------------------------

## Widget som ritar ut nodkartan i GUI:t ##

from PyQt6.QtWidgets import QFrame
from PyQt6.QtCore import Qt, QPoint, pyqtSignal, QLineF, QPointF
from PyQt6.QtGui import QPainter, QPen, QColor, QFont


# MapFrame är en specialiserad QFrame som ritar ut nodkartan, markerar varor och visar den planerade rutten.
class MapFrame(QFrame):

    edge_clicked = pyqtSignal(tuple)     # Signal som skickar (node_a, node_b) när en kant klickas

    def __init__(self, grid_nodes_ref, parent=None):
        super().__init__(parent)         # Initiera QFrame
        self.grid_nodes = grid_nodes_ref # Referens till nodwidgets i MainWindow (för att få deras positioner)
        self.highlighted_edges = []      # Lista över vilka kanter som ska markeras som varor
        self.route_nodes = []            # Planerad rutt
        self.setStyleSheet("background-color: #1a252f; border: 2px solid #7f8c8d; border-radius: 5px;")
        self.setFixedSize(360, 360)

    # Uppdatera och måla om kartan när varor sätts
    def set_item_edges(self, edges_list):
        self.highlighted_edges = list(edges_list) # Kopiera listan
        self.update()                             # Måla om kartan med de nya varorna markerade

    # Uppdatera och måla om kartan när en ny rutt planeras
    def set_route(self, node_list):
        self.route_nodes = list(node_list) # Kopiera listan
        self.update()                      # Måla om kartan med den nya rutten

    # Hjälpfunktion för att få alla kanter i 5x5-griden plus startnoden (25-0)
    def _get_all_edges(self):
        edges = []                       # Lista över alla kanter i 5x5-griden plus startnoden
        for i in range(25):
            kol = i % 5                  # Kolumnindex (0-4)
            rad = i // 5                 # Radindex (0-4)
            if kol < 4:
                edges.append((i, i + 1)) # Kant till höger
            if rad < 4:
                edges.append((i, i + 5)) # Kant nedåt
        edges.append((25, 0))            # Kant från startnoden till nod 0
        return edges

    # Hjälpfunktion för att få centrumkoordinaterna för en nodwidget
    def _get_center(self, node_id):
        if node_id not in self.grid_nodes:                                      # Om noden inte finns i grid_nodes (ex startnod 25)...
            return QPoint(0, 0)                                                 # ...returnera (0, 0) som en fallback (bör inte hända för giltiga noder)
        widget = self.grid_nodes[node_id]                                       # Hämta nodwidgeten från grid_nodes
        return widget.pos() + QPoint(widget.width() // 2, widget.height() // 2) # Beräkna centrumkoordinaterna för nodwidgeten

    # Hantera klick på kartan
    def mousePressEvent(self, event):
        if event.button() != Qt.MouseButton.LeftButton or len(self.grid_nodes) < 26: # Endast hantera vänsterklick och se till att alla noder är laddade
            super().mousePressEvent(event)                                           # Anropa standardhanteringen (för att inte blockera andra event) och returnera utan att göra något mer
            return

        click = QPointF(event.pos()) # Klickets position
        best_edge = None
        best_dist = 20               # Maxavstånd för att registrera ett klick

        for a, b in self._get_all_edges():    # Loop över alla kanter i 5x5-griden plus startnoden
            p1 = QPointF(self._get_center(a)) # Få centrumkoordinaterna för nod a
            p2 = QPointF(self._get_center(b)) # Få centrumkoordinaterna för nod b
            line = QLineF(p1, p2)             # Linje för kanten mellan nod a och nod b

            # Beräkna avståndet från klicket till linjen (kanten) och hitta den närmaste kanten inom ett visst avstånd
            length = line.length()
            if length < 1:
                continue
            
            # Projektion av klicket på linjen för att hitta den närmaste punkten på kanten
            dx, dy = p2.x() - p1.x(), p2.y() - p1.y()                                       # Riktning av linjen
            t = ((click.x() - p1.x()) * dx + (click.y() - p1.y()) * dy) / (length * length) # Projektionens t-värde längs linjen
            t = max(0.1, min(0.9, t))                                                       # Begränsa t så att den inte är för nära noderna
            
            # Närmaste punkten på linjen från klicket
            closest = QPointF(p1.x() + t * dx, p1.y() + t * dy)
            dist = QLineF(click, closest).length()
            
            if dist < best_dist:    # Om detta är den närmaste kanten hittills inom det tillåtna avståndet...
                best_dist = dist    # ...uppdatera bästa avståndet...
                best_edge = (a, b)  # ...och spara den bästa kanten (a, b)

        if best_edge:                         # Om en giltig kant hittades inom det tillåtna avståndet...
            self.edge_clicked.emit(best_edge) # ...skicka signalen edge_clicked med den klickade kanten (a, b)

        super().mousePressEvent(event)

    # Rita ut kartan, den planerade rutten och markerade varor
    def paintEvent(self, event):
        super().paintEvent(event)
        
        if not self.grid_nodes or len(self.grid_nodes) < 26:
            return

        painter = QPainter(self) # Skapa en QPainter för att rita på widgeten
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        # --- Lager 1 - Nodkartan ---
        pen = QPen(QColor("#2c3e50"), 6)
        pen.setCapStyle(Qt.PenCapStyle.RoundCap)
        painter.setPen(pen)

        for i in range(25):
            if i % 5 < 4:
                painter.drawLine(self._get_center(i), self._get_center(i + 1))
        for i in range(25):
            if i // 5 < 4:
                painter.drawLine(self._get_center(i), self._get_center(i + 5))
        painter.drawLine(self._get_center(25), self._get_center(0))

        # --- Lager 2 - Planerad rutt ---
        if self.route_nodes and len(self.route_nodes) >= 2:
            route_pen = QPen(QColor("#00bcd4"), 5, Qt.PenStyle.SolidLine)
            route_pen.setCapStyle(Qt.PenCapStyle.RoundCap)
            painter.setPen(route_pen)
            for i in range(len(self.route_nodes) - 1):
                a, b = self.route_nodes[i], self.route_nodes[i + 1]
                if a in self.grid_nodes and b in self.grid_nodes:
                    painter.drawLine(self._get_center(a), self._get_center(b))

        # --- Lager 3 - Markerade varor ---
        if self.highlighted_edges:
            highlight_pen = QPen(QColor("#e67e22"), 8)
            highlight_pen.setCapStyle(Qt.PenCapStyle.RoundCap)
            number_font = QFont("Arial", 14, QFont.Weight.Bold)
            painter.setFont(number_font)
            
            for idx, (a, b) in enumerate(self.highlighted_edges):
                if a in self.grid_nodes and b in self.grid_nodes:
                    p1 = self._get_center(a)
                    p2 = self._get_center(b)
                    painter.setPen(highlight_pen)
                    painter.drawLine(p1, p2)
                    
                    mid = QPoint((p1.x() + p2.x()) // 2, (p1.y() + p2.y()) // 2)
                    painter.setPen(QPen(QColor("#ffffff")))
                    painter.drawText(mid.x() - 6, mid.y() + 5, str(idx + 1))
