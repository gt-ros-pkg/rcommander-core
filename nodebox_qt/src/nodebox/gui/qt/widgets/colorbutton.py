from PyQt4.QtGui import QToolButton, QPainter, QPixmap, QPen, QColor, QColorDialog, QIcon
from PyQt4.QtCore import SIGNAL, QRect

class ColorButton(QToolButton):
    def __init__(self, *args):
        QToolButton.__init__(self, *args)
        self._color = QColor()
        self.connect(self, SIGNAL("clicked()"), self.selectColor)
    
    def color(self):
        return self._color

    def setColor(self, color):
        self._color = color
        self.updateColor()

    def updateColor(self):
        iconSize = self.iconSize()
        width = iconSize.width()
        height = iconSize.height()
        pixmap = QPixmap(iconSize)
        pixmap.fill(self._color)
        painter = QPainter()
        painter.begin(pixmap)
        painter.setPen(QPen(QColor("#777777")))
        painter.drawRect(QRect(0, 0, width - 1, height - 1))
        painter.end()
        self.setIcon(QIcon(pixmap))
    
    def selectColor(self):
        self.setChecked(True)
        color = QColorDialog.getColor(self._color)
        self.setChecked(False)
        if color != self._color:
            self.setColor(color)
            self.emit(SIGNAL("colorChanged()"))