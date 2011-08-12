from PyQt4.QtGui import *
from PyQt4.QtCore import *

class NodeBoxGraphicsView(QGraphicsWidget):
    zoomLevels = [0.1, 0.25, 0.5, 0.75]
    zoom = 1.0
    zoom = .5
    while zoom <= 20.0:
        zoomLevels.append(zoom)
        zoom += 1.0
            
    def __init__(self, parent=None):
        QGraphicsWidget.__init__(self, parent)
        self.setFlag(QGraphicsItem.ItemClipsToShape, True)
        self.setFocusPolicy(Qt.ClickFocus)
        self._rect = QRectF(0, 0, 1000, 1000)
        self._shape = QPainterPath()
        self._shape.addRect(self._rect)
        self._canvas = None
        self._dirty = True
        self.mousedown = False
        self.rightdown = False
        self.mousePosition = QPointF(0, 0)
        self.mouseDCPosition = QPointF(0, 0)
        self.mousedoubleclick = False
        self.keydown = False
        self.key = None
        self.keycode = None        
        self.scrollwheel = False
        self.wheeldelta = 0.0
        self._zoom = 1.0
        #self.setMouseTracking(True)
        #print self.hasMouseTracking()

    def boundingRect(self):
        return self._rect

    def shape(self):
        return self._shape

    def _check_cache(self):
        cacheMode = self.cacheMode()
        DeviceCoordinateCache = QGraphicsItem.DeviceCoordinateCache
        NoCache = QGraphicsItem.NoCache
        if self.canvas is not None:
            if self.document.animationTimer is not None:
                cache = False
            elif len(self.canvas) > 400:
                x, y, width, height = self._rect.getRect()
                cache = True
                if width > 1000 or height > 1000 or \
                    not (width * self.zoom <= 1200 and height * self.zoom <= 1200):
                    cache = False
            else:
                cache = False
            if cache and cacheMode != DeviceCoordinateCache:
                self.setCacheMode(DeviceCoordinateCache)
            elif not cache and cacheMode != NoCache:
                self.setCacheMode(NoCache)
        elif cacheMode != NoCache:
            self.setCacheMode(QNoCache)
        
    def _get_canvas(self):
        return self._canvas

    def _set_canvas(self, canvas):
        self._canvas = canvas
        if canvas is not None:
            x, y, width, height = self._rect.getRect()
            size = int(width), int(height)
            if size != self.canvas.size:
                width, height = self.canvas.size
                scene = self._scene
                self._rect = rect = QRectF(0, 0, width, height)
                scene.setSceneRect(rect)
                self._shape = shape = QPainterPath()
                shape.addRect(rect)
        self._check_cache()
        self.markDirty()
    canvas = property(_get_canvas, _set_canvas)

    def _get_zoom(self):
        return self._zoom

    def _set_zoom(self, zoom):
        self._zoom = zoom
        self.document.zoomLevel.setText("%i%%" % (self._zoom * 100.0))
        self.document.zoomSlider.setValue(self._zoom * 100.0)
        self._check_cache()
        transform = QTransform()
        transform.scale(self.zoom, self.zoom)
        self.superView.setTransform(transform)
    zoom = property(_get_zoom, _set_zoom)

    def findNearestZoomIndex(self, zoom):
        """Returns the nearest zoom level, and whether we found a direct, exact
        match or a fuzzy match."""
        try: # Search for a direct hit first.
            idx = self.zoomLevels.index(zoom)
            return idx, True
        except ValueError: # Can't find the zoom level, try looking at the indexes.
            idx = 0
            try:
                while self.zoomLevels[idx] < zoom:
                    idx += 1
            except KeyError: # End of the list
                idx = len(self.zoomLevels) - 1 # Just return the last index.
            return idx, False

    def zoomIn_(self):
        idx, direct = self.findNearestZoomIndex(self.zoom)
        # Direct hits are perfect, but indirect hits require a bit of help.
        # Because of the way indirect hits are calculated, they are already
        # rounded up to the upper zoom level; this means we don't need to add 1.
        if direct:
            idx += 1
        idx = max(min(idx, len(self.zoomLevels)-1), 0)
        self.zoom = self.zoomLevels[idx]

    def zoomOut_(self):
        idx, direct = self.findNearestZoomIndex(self.zoom)
        idx -= 1
        idx = max(min(idx, len(self.zoomLevels)-1), 0)
        self.zoom = self.zoomLevels[idx]

    def zoomTo_(self, value):
        self.zoom = value

    def zoomToFit_(self):
        w, h = self.canvas.size
        factor = min(self._viewPort.width() / float(w), self._viewPort.height() / float(h))
        self.zoom = factor        

    def dragZoom_(self, value):
        self.zoom = value / 100.0

    def markDirty(self, redraw=True):
        self._dirty = True
        if redraw:
            self._viewPort.update()
            
    def _updateImage(self, painter):
        if self.canvas is None: return
        painter._screen = True
        try:
            painter.save()
            self.canvas.draw(painter)
        except:
            # A lot of code just to display the error in the output view.
            etype, value, tb = sys.exc_info()
            if tb.tb_next is not None:
                tb = tb.tb_next  # skip the frame doing the exec
            traceback.print_exception(etype, value, tb)
            data = "".join(traceback.format_exception(etype, value, tb))
            outputView = self.document.outputView
            outputView.setTextColor(QColor(255, 0, 0)) # TODO: Refactor color
            outputView.insertPlainText(data)
        finally:
            painter.restore()

    def paint(self, painter, item, widget):
        if self._dirty:
            self._updateImage(painter)

    def mousePressEvent(self, event):
        self.mousePosition = event.scenePos()

        if event.button() == Qt.LeftButton:
            self.mousedown = True 

        if event.button() == Qt.RightButton:
            self.rightdown = True

        self.setFocus()

    def mouseDoubleClickEvent(self, event):
        if event.button() == Qt.LeftButton:
            #self.mouseDCPosition = event.scenePos()
            self.mousedoubleclick = True
            self.setFocus()

    def mouseMoveEvent(self, event):
        self.mousePosition = event.scenePos()

    def mouseReleaseEvent(self, event): 
        self.mousePosition = event.scenePos()
        if event.button() == Qt.LeftButton: 
           	self.mousedown = False 
        if event.button() == Qt.RightButton: 
            self.rightdown = False

    def keyPressEvent(self, event): 
        self.keydown = True 
        self.key = event.text() 
        self.keycode = event.key() 

    def keyReleaseEvent(self, event): 
        self.keydown = False 
        self.key = event.text() 
        self.keycode = event.key() 

    def wheelEvent(self, event): 
        self.scrollwheel = True 
        self.wheeldelta = event.delta() / 120.
        print 'event delta', event.delta()
        

