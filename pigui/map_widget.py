from PyQt6.QtWidgets import QFrame
from PyQt6.QtCore import Qt, QPoint, QPointF, QLineF, pyqtSignal
from PyQt6.QtGui import QPainter, QPen, QColor, QFont, QBrush


# Layout constants — tweak these to adjust the visual appearance
_MARGIN   = 30   # px from widget edge to first/last node
_NODE_R   = 8    # node circle radius
_START_R  = 10   # start-node circle radius (node 25 is larger)


class MapFrame(QFrame):
    """
    Custom widget that draws the 5x5 node grid (nodes 0-24) plus the
    start/home node (node 25), with:
      - Orange highlighted edges for item locations (numbered)
      - Cyan route overlay
      - Click-to-toggle-edge support
      - Robot position marker

    Node layout
    -----------
    Nodes 0-24 form a 5×5 grid:
        0  1  2  3  4
        5  6  7  8  9
       10 11 12 13 14
       15 16 17 18 19
       20 21 22 23 24
    Node 25 (START) sits above node 0 (connected by a single edge).
    """

    edge_clicked = pyqtSignal(tuple)   # Emits (node_a, node_b) when an edge is clicked

    def __init__(self, parent=None):
        super().__init__(parent)
        self.highlighted_edges = []   # Item-location edges: [(a, b), ...]
        self.route_nodes       = []   # Planned route: [25, 0, 1, ...]
        self.robot_node        = None # Current robot position (node id or None)

        self.setStyleSheet(
            "background-color: #1a252f; border: 2px solid #7f8c8d; border-radius: 5px;"
        )
        self.setFixedSize(360, 420)   # Slightly taller to fit start node above grid

    # ------------------------------------------------------------------
    # Public API (same as before so main_window.py needs no extra changes)
    # ------------------------------------------------------------------

    def set_item_edges(self, edges_list):
        """Set the highlighted edges (item locations) and repaint."""
        self.highlighted_edges = list(edges_list)
        self.update()

    def set_route(self, node_list):
        """Set the planned route node sequence and repaint."""
        self.route_nodes = list(node_list)
        self.update()

    def set_robot_node(self, node_id):
        """Update the robot's current node marker."""
        self.robot_node = node_id
        self.update()

    # ------------------------------------------------------------------
    # Internal geometry helpers
    # ------------------------------------------------------------------

    def _node_pos(self, node_id: int) -> QPoint:
        """
        Return the centre pixel position of a node inside this widget.

        The 5×5 grid occupies a square region with _MARGIN padding on all
        sides.  Node 25 (start) sits one grid-step above node 0.
        """
        w = self.width()
        h = self.height()

        # Grid cell size (same horizontal and vertical spacing)
        grid_w = w - 2 * _MARGIN
        grid_h = h - 2 * _MARGIN

        step_x = grid_w / 4   # 4 gaps between 5 columns
        step_y = grid_h / 5   # 5 gaps: 4 between grid rows + 1 for start node

        if node_id == 25:
            # Start node: same x as column 0, one step above row 0
            x = _MARGIN
            y = _MARGIN
            return QPoint(int(x), int(y))

        col = node_id % 5
        row = node_id // 5
        x = _MARGIN + col * step_x
        y = _MARGIN + step_y + row * step_y   # +step_y to leave room for node 25
        return QPoint(int(x), int(y))

    def _all_edges(self):
        """Yield every valid (node_a, node_b) edge pair (a < b)."""
        for i in range(25):
            if i % 5 < 4:
                yield (i, i + 1)   # horizontal
            if i // 5 < 4:
                yield (i, i + 5)   # vertical
        yield (25, 0)               # start-node connection

    # ------------------------------------------------------------------
    # Mouse interaction
    # ------------------------------------------------------------------

    def mousePressEvent(self, event):
        if event.button() != Qt.MouseButton.LeftButton:
            super().mousePressEvent(event)
            return

        click      = QPointF(event.pos())
        best_edge  = None
        best_dist  = 20   # max pixel distance from edge centre-line

        for a, b in self._all_edges():
            p1 = QPointF(self._node_pos(a))
            p2 = QPointF(self._node_pos(b))
            dx = p2.x() - p1.x()
            dy = p2.y() - p1.y()
            length_sq = dx * dx + dy * dy
            if length_sq < 1:
                continue

            # Parametric projection onto the segment
            t = ((click.x() - p1.x()) * dx + (click.y() - p1.y()) * dy) / length_sq
            t = max(0.1, min(0.9, t))   # exclude near-node clicks

            closest = QPointF(p1.x() + t * dx, p1.y() + t * dy)
            dist    = QLineF(click, closest).length()

            if dist < best_dist:
                best_dist = dist
                best_edge = (a, b)

        if best_edge:
            self.edge_clicked.emit(best_edge)

        super().mousePressEvent(event)

    # ------------------------------------------------------------------
    # Painting
    # ------------------------------------------------------------------

    def paintEvent(self, event):
        super().paintEvent(event)
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        # Layer 1 — base grid edges (dark)
        base_pen = QPen(QColor("#2c3e50"), 6)
        base_pen.setCapStyle(Qt.PenCapStyle.RoundCap)
        painter.setPen(base_pen)
        for a, b in self._all_edges():
            painter.drawLine(self._node_pos(a), self._node_pos(b))

        # Layer 2 — planned route (cyan)
        if len(self.route_nodes) >= 2:
            route_pen = QPen(QColor("#00bcd4"), 5)
            route_pen.setCapStyle(Qt.PenCapStyle.RoundCap)
            painter.setPen(route_pen)
            for i in range(len(self.route_nodes) - 1):
                painter.drawLine(
                    self._node_pos(self.route_nodes[i]),
                    self._node_pos(self.route_nodes[i + 1])
                )

        # Layer 3 — item edges (orange, numbered)
        if self.highlighted_edges:
            hi_pen = QPen(QColor("#e67e22"), 8)
            hi_pen.setCapStyle(Qt.PenCapStyle.RoundCap)
            num_font = QFont("Arial", 12, QFont.Weight.Bold)
            painter.setFont(num_font)

            for idx, (a, b) in enumerate(self.highlighted_edges):
                p1 = self._node_pos(a)
                p2 = self._node_pos(b)
                painter.setPen(hi_pen)
                painter.drawLine(p1, p2)

                mid = QPoint((p1.x() + p2.x()) // 2, (p1.y() + p2.y()) // 2)
                painter.setPen(QPen(QColor("#ffffff")))
                painter.drawText(mid.x() - 5, mid.y() + 5, str(idx + 1))

        # Layer 4 — node circles
        for node_id in range(26):
            pos    = self._node_pos(node_id)
            radius = _START_R if node_id == 25 else _NODE_R

            if node_id == 25:
                color = QColor("#2ecc71")   # green for home/start
            elif node_id == self.robot_node:
                color = QColor("#e74c3c")   # red for robot position
            else:
                color = QColor("#ecf0f1")   # light grey for normal nodes

            painter.setPen(QPen(QColor("#1a252f"), 2))
            painter.setBrush(QBrush(color))
            painter.drawEllipse(pos, radius, radius)

        # Layer 5 — node labels (small, shown on normal nodes)
        label_font = QFont("Arial", 7)
        painter.setFont(label_font)
        painter.setPen(QPen(QColor("#7f8c8d")))
        for node_id in range(25):
            pos = self._node_pos(node_id)
            painter.drawText(pos.x() + _NODE_R + 1, pos.y() + 4, str(node_id))

        # Start-node label
        pos = self._node_pos(25)
        bold_font = QFont("Arial", 8, QFont.Weight.Bold)
        painter.setFont(bold_font)
        painter.setPen(QPen(QColor("#2ecc71")))
        painter.drawText(pos.x() + _START_R + 2, pos.y() + 4, "S")

        painter.end()