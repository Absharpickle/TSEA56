from PyQt6.QtWidgets import QFrame
from PyQt6.QtCore import Qt, QPoint, pyqtSignal, QLineF, QPointF
from PyQt6.QtGui import QPainter, QPen, QColor, QFont


class MapFrame(QFrame):
    """Custom widget that draws the 5x5 node grid with edges, item highlights,
    route visualization, and supports click-to-add-edge."""

    edge_clicked = pyqtSignal(tuple)  # Emits (node_a, node_b) when an edge is clicked

    def __init__(self, grid_nodes_ref, parent=None):
        super().__init__(parent)
        self.grid_nodes = grid_nodes_ref
        self.highlighted_edges = []  # Item location edges: [(a, b), ...]
        self.route_nodes = []        # Planned route node sequence: [25, 0, 1, ...]
        self.setStyleSheet("background-color: #1a252f; border: 2px solid #7f8c8d; border-radius: 5px;")
        self.setFixedSize(360, 360)

    def set_item_edges(self, edges_list):
        """Set the highlighted edges (item locations) and repaint."""
        self.highlighted_edges = list(edges_list)
        self.update()

    def set_route(self, node_list):
        """Set the planned route node sequence and repaint."""
        self.route_nodes = list(node_list)
        self.update()

    def _get_all_edges(self):
        """Return all valid edges in the grid as (node_a, node_b) tuples."""
        edges = []
        for i in range(25):
            kol = i % 5
            rad = i // 5
            if kol < 4:
                edges.append((i, i + 1))
            if rad < 4:
                edges.append((i, i + 5))
        edges.append((25, 0))  # Start node connection
        return edges

    def _get_center(self, node_id):
        """Get the center point of a node widget."""
        if node_id not in self.grid_nodes:
            return QPoint(0, 0)
        widget = self.grid_nodes[node_id]
        return widget.pos() + QPoint(widget.width() // 2, widget.height() // 2)

    def mousePressEvent(self, event):
        """Detect clicks near edges and emit edge_clicked signal."""
        if event.button() != Qt.MouseButton.LeftButton or len(self.grid_nodes) < 26:
            super().mousePressEvent(event)
            return

        click = QPointF(event.pos())
        best_edge = None
        best_dist = 20  # Max pixel distance from edge line to register a click

        for a, b in self._get_all_edges():
            p1 = QPointF(self._get_center(a))
            p2 = QPointF(self._get_center(b))
            line = QLineF(p1, p2)

            # Project click onto the line segment, reject if outside endpoints
            length = line.length()
            if length < 1:
                continue
            
            # Parametric position along line (0..1)
            dx, dy = p2.x() - p1.x(), p2.y() - p1.y()
            t = ((click.x() - p1.x()) * dx + (click.y() - p1.y()) * dy) / (length * length)
            t = max(0.1, min(0.9, t))  # Exclude near-node clicks (avoid ambiguity)
            
            # Closest point on segment
            closest = QPointF(p1.x() + t * dx, p1.y() + t * dy)
            dist = QLineF(click, closest).length()
            
            if dist < best_dist:
                best_dist = dist
                best_edge = (a, b)

        if best_edge:
            self.edge_clicked.emit(best_edge)

        super().mousePressEvent(event)

    def paintEvent(self, event):
        super().paintEvent(event)
        
        if not self.grid_nodes or len(self.grid_nodes) < 26:
            return

        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        # --- Layer 1: Base grid edges (dark) ---
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

        # --- Layer 2: Planned route (cyan, drawn under items) ---
        if self.route_nodes and len(self.route_nodes) >= 2:
            route_pen = QPen(QColor("#00bcd4"), 5, Qt.PenStyle.SolidLine)
            route_pen.setCapStyle(Qt.PenCapStyle.RoundCap)
            painter.setPen(route_pen)
            for i in range(len(self.route_nodes) - 1):
                a, b = self.route_nodes[i], self.route_nodes[i + 1]
                if a in self.grid_nodes and b in self.grid_nodes:
                    painter.drawLine(self._get_center(a), self._get_center(b))

        # --- Layer 3: Item edges (orange with order numbers) ---
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
