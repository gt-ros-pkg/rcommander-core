from PyQt4.QtGui import QLabel, QPainter, QPainterPath, QBrush, QColor, QPalette
from PyQt4.QtCore import Qt, SIGNAL, QTimer

def coordinates(x0, y0, distance, angle):
    from math import radians, sin, cos
    x1 = x0 + cos(radians(angle)) * distance
    y1 = y0 + sin(radians(angle)) * distance
    return x1, y1

class AnimationSpinner(QLabel):
    def __init__(self, parent=None):
        QLabel.__init__(self, parent)
        brightness = parent.palette().color(QPalette.Window).valueF()
        self._bw = brightness < 0.5 and 1 or 0
        self._steps = 12
        self._setup()
        self._isRunning = False
        self.animationTimer = None
        
    def _setup(self):
        steps = self._steps
        anglestep = 360. / steps
        fillstep = 0.6 / (steps - 1)
        self._fillsteps = [0.71 - i * fillstep for i in range(steps)]
        self._coords = [coordinates(8, 8, 6, anglestep*i) for i in range(steps)]
        self._path = QPainterPath()
        self._path.addRoundedRect(0, 0, 4, 2, 1, 1)
    
    def start(self):
        self.animationTimer = QTimer(self)
        self.connect(self.animationTimer, SIGNAL("timeout()"), self.run)
        self.animationTimer.start(35)
        self._isRunning = True
    
    def stop(self):
        if self.animationTimer is not None:
            self.animationTimer.stop()
            self.animationTimer = None
        self._isRunning = False
        self.repaint()
    
    def run(self):
        self.repaint()
        self._fillsteps = self._fillsteps[1:] + [self._fillsteps[0]]
        
    def paintEvent(self, event):
        if self._isRunning:
            anglestep = 360. / self._steps
            fillsteps = self._fillsteps
            factor = min(self.width(), self.height()) / 16.
            bw = self._bw

            p = QPainter(self)
            p.setRenderHint(QPainter.Antialiasing, True)
            p.scale(factor, factor)
            p.setPen(Qt.NoPen)

            for i in range(self._steps):
                x1, y1 = self._coords[i]
                c = fillsteps[self._steps - 1 - i]
                a = anglestep * i
                p.setBrush(QBrush(QColor.fromRgbF(bw, bw, bw, c)))
                p.save()
                p.translate(x1 - 2, y1 - 1)
                p.translate(2, 1)
                p.rotate(a)
                p.translate(-2, -1)
                p.drawPath(self._path)
                p.restore()
