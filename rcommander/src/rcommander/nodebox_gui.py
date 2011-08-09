import roslib; roslib.load_manifest('rcommander')
from PyQt4.QtGui import *
from PyQt4.QtCore import *
from nodebox import graphics

class NodeBoxGUI:

    def __init__(self, graphics_view):

        #add scene to QGraphicsView
        scene = QGraphicsScene()
        self.drawing_widget = NodeBoxGUIHelper(graphics_view.viewport(), scene)
        graphics_view.setScene(scene)
        # TODO: graphics_view._scene = scene
        scene.setItemIndexMethod(QGraphicsScene.NoIndex)
        scene.addItem(self.drawing_widget)

        #Add NB to scene
        self.namespace = {}
        self.canvas = graphics.Canvas()
        self.canvas._setTextScaleFactor(QPixmap(1, 1).logicalDpiX() / 72.0)
        self.context = graphics.Context(self.canvas, self.namespace)

        #Setup the scene
        self.setup()
        self.animationTimer = QTimer(self)
        self.connect(self.animationTimer, SIGNAL("timeout()"), self._draw)
        self.animationTimer.start(1000.0 / self.canvas.speed)

    def properties(self):
        properties = self.namespace

        pos = self.drawing_widget.mousePosition
        properties["MOUSEX"], properties["MOUSEY"] = pos.x(), pos.y()
        properties["mousedoubleclick"]   = self.drawing_widget.mousedoubleclick
        self.drawing_widget.mousedoubleclick = False
        properties["mousedown"]   = self.drawing_widget.mousedown
        properties["rightdown"]   = self.drawing_widget.rightdown
        properties["keydown"]     = self.drawing_widget.keydown
        properties["key"]         = self.drawing_widget.key
        properties["keycode"]     = self.drawing_widget.keycode
        properties["scrollwheel"] = self.drawing_widget.scrollwheel
        properties["wheeldelta"]  = self.drawing_widget.wheeldelta
        return properties

    def _draw(self):
        self.canvas.clear()
        self.draw(self.properties())
        self.drawing_widget.set_canvas(self.canvas)
        self.context._resetContext()

    def draw(self, properties):
        pass

    def setup(self):
        pass

class NodeBoxGUIHelper(QGraphicsWidget):

    def __init__(self, viewport, scene, parent=None):
        QGraphicsWidget.__init__(self, parent)
        self.setFlag(QGraphicsItem.ItemClipsToShape, True)
        self.setFocusPolicy(Qt.StrongFocus)

        self.mousedown = False
        self.rightdown = False
        self.mousePosition = QPointF(0, 0)
        self.mousedoubleclick = False
        self.keydown = False
        self.key = None
        self.keycode = None        
        self.scrollwheel = False
        self.wheeldelta = 0.0

        self._dirty = True
        self._canvas = None
        self._viewPort = viewport
        self._scene = scene
        self._rect = QRectF(0, 0, 1000, 1000)
        self._shape = QPainterPath()
        self._shape.addRect(self._rect)

    def mousePressEvent(self, event):
        self.mousePosition = event.scenePos()

        if event.button() == Qt.LeftButton:
            self.mousedown = True 

        if event.button() == Qt.RightButton:
            self.rightdown = True

        self.setFocus()

    def mouseDoubleClickEvent(self, event):
        if event.button() == Qt.LeftButton:
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

    ######################################################################
    # Magical drawing functions
    ######################################################################

    def get_canvas(self):
        return self._canvas

    def set_canvas(self, canvas):
        self._canvas = canvas
        if canvas is not None:
            x, y, width, height = self._rect.getRect()
            size = int(width), int(height)
            if size != self.get_canvas().size:
                width, height = self.get_canvas().size
                scene = self._scene
                self._rect = rect = QRectF(0, 0, width, height)
                scene.setSceneRect(rect)
                self._shape = shape = QPainterPath()
                shape.addRect(rect)
        self._check_cache()
        self._dirty = True      #signal that we want to be redrawn to paint
        self._viewPort.update() #tell QT that we want to be redrawn

    ##########################################################################################
    # Must be implemented for mouse events
    ##########################################################################################
    def boundingRect(self):
        return self._rect

    def shape(self):
        return self._shape

    def paint(self, painter, item, widget):
        if self._dirty:
            if self.get_canvas() is None: 
                return
            painter.save()
            self.get_canvas().draw(painter)
            painter.restore()

    def _check_cache(self):
        cacheMode = self.cacheMode()
        DeviceCoordinateCache = QGraphicsItem.DeviceCoordinateCache
        NoCache = QGraphicsItem.NoCache
        if self.get_canvas() is not None:
            #if self.document.animationTimer is not None:
            #    cache = False
            #elif len(self.get_canvas()) > 400:
            if len(self.get_canvas()) > 400:
                x, y, width, height = self._rect.getRect()
                cache = True
                if width > 1000 or height > 1000: #or \
                    #not (width * self.zoom <= 1200 and height * self.zoom <= 1200):
                    cache = False
            else:
                cache = False
            if cache and cacheMode != DeviceCoordinateCache:
                self.setCacheMode(DeviceCoordinateCache)
            elif not cache and cacheMode != NoCache:
                self.setCacheMode(NoCache)
        elif cacheMode != NoCache:
            self.setCacheMode(QNoCache)

