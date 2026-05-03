from PyQt6.QtWidgets import QFrame
from PyQt6.QtCore import Qt, QPoint
from PyQt6.QtGui import QPainter, QPen, QColor, QFont


class MapFrame(QFrame):
    """Custom widget that draws the 5x5 node grid with edges and item highlights."""

    def __init__(self, grid_nodes_ref, parent=None):
        super().__init__(parent)
        self.grid_nodes = grid_nodes_ref
        self.highlighted_edges = []  # List of (node_a, node_b) tuples
        self.setStyleSheet("background-color: #1a252f; border: 2px solid #7f8c8d; border-radius: 5px;")
        self.setFixedSize(360, 360)

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

        # Horizontal edges
        for i in range(25):
            kol = i % 5
            if kol < 4:  
                p1 = get_center(i)
                p2 = get_center(i + 1)
                painter.drawLine(p1, p2)

        # Vertical edges
        for i in range(25):
            rad = i // 5
            if rad < 4:  
                p1 = get_center(i)
                p2 = get_center(i + 5)
                painter.drawLine(p1, p2)

        # Start node connection
        p_start = get_center(25)
        p_0 = get_center(0)
        painter.drawLine(p_start, p_0)

        # Draw highlighted edges (item locations) in orange with order numbers
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
