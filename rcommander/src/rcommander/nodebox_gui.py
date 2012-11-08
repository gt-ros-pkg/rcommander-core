#import roslib; roslib.load_manifest('rcommander_core')
import PyQt4.QtGui as qtg
import PyQt4.QtCore as qtc
import PyQt4.QtOpenGL as qtl
from nodebox import graphics

import time
import rospy
import threading

## Sets up the NodeBox graphics widget.
class NodeBoxGUI:

    ## Constructor
    # @param graphics_view a QGraphicsView to turn into a Nodebox viewport.
    def __init__(self, graphics_view):

        #add scene to QGraphicsView
        scene = qtg.QGraphicsScene()
        graphics_view.setViewport(qtl.QGLWidget())
        self.drawing_widget = NodeBoxGUIHelper(graphics_view.viewport(), scene)
        graphics_view.setScene(scene)
        scene.setItemIndexMethod(qtg.QGraphicsScene.NoIndex)
        scene.addItem(self.drawing_widget)

        #Add NB to scene
        self.namespace = {}
        self.canvas = graphics.Canvas()
        self.canvas._setTextScaleFactor(qtg.QPixmap(1, 1).logicalDpiX() / 72.0)
        self.context = graphics.Context(self.canvas, self.namespace)

        #Setup the scene
        self.setup()
        self.animationTimer = qtc.QTimer(self)
        self.connect(self.animationTimer, qtc.SIGNAL("timeout()"), self._draw)
        self.start_drawing()

    ## Starts animation timer callbacks
    def start_drawing(self):
        self.animationTimer.start(1000.0 / self.canvas.speed)

    ## Stops animation timer callbacks
    def stop_drawing(self):
        self.animationTimer.stop()

    ## Create the magic variables used by NodeBox libraries
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

    ## Setup and tear down stuff needed before calling client drawing functions
    def _draw(self):
        self.canvas.clear()
        self.draw(self.properties())
        self.drawing_widget.set_canvas(self.canvas)
        self.context._resetContext()

    ## Inherit from this class and implement this method to draw on the Nodebox canvas
    def draw(self, properties):
        pass

    ## Setup function (called before drawing the first time) for inherited from this class
    def setup(self):
        pass

## Helper class that setups all the user events and deal with QT drawing
class NodeBoxGUIHelper(qtg.QGraphicsWidget):

    ## Constructor
    # @param viewport QT viewport
    # @param scene QT scene
    # @param parent parent QT object
    def __init__(self, viewport, scene, parent=None):
        qtg.QGraphicsWidget.__init__(self, parent)
        self.setFlag(qtg.QGraphicsItem.ItemClipsToShape, True)
        self.setFocusPolicy(qtc.Qt.StrongFocus)

        self.mousedown = False
        self.rightdown = False
        self.mousePosition = qtc.QPointF(0, 0)
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
        self._rect = qtc.QRectF(0, 0, 1000, 1000)
        self._shape = qtg.QPainterPath()
        self._shape.addRect(self._rect)

    ## Mouse pressed QT callback 
    def mousePressEvent(self, event):
        self.mousePosition = event.scenePos()

        if event.button() == qtc.Qt.LeftButton:
            self.mousedown = True 

        if event.button() == qtc.Qt.RightButton:
            self.rightdown = True

        self.setFocus()

    ## Mouse double clicked QT callback 
    def mouseDoubleClickEvent(self, event):
        if event.button() == qtc.Qt.LeftButton:
            self.mousedoubleclick = True
            self.setFocus()

    ## Mouse move QT callback 
    def mouseMoveEvent(self, event):
        self.mousePosition = event.scenePos()

    ## Mouse released QT callback 
    def mouseReleaseEvent(self, event): 
        self.mousePosition = event.scenePos()
        if event.button() == qtc.Qt.LeftButton: 
           	self.mousedown = False 
        if event.button() == qtc.Qt.RightButton: 
            self.rightdown = False

    ## Key pressed QT callback
    def keyPressEvent(self, event): 
        self.keydown = True 
        self.key = event.text() 
        self.keycode = event.key() 

    ## Key release QT callback
    def keyReleaseEvent(self, event): 
        self.keydown = False 
        self.key = event.text() 
        self.keycode = event.key() 

    ## Mouse wheele QT callback
    def wheelEvent(self, event): 
        self.scrollwheel = True 
        self.wheeldelta = event.delta() / 120.

    ######################################################################
    # Magical drawing functions
    ######################################################################

    ## Getter for canvas object
    def get_canvas(self):
        return self._canvas

    ## Setter for canvas object
    def set_canvas(self, canvas):
        self._canvas = canvas
        if canvas is not None:
            x, y, width, height = self._rect.getRect()
            size = int(width), int(height)
            if size != self.get_canvas().size:
                width, height = self.get_canvas().size
                scene = self._scene
                self._rect = rect = qtc.QRectF(0, 0, width, height)
                scene.setSceneRect(rect)
                self._shape = shape = qtg.QPainterPath()
                shape.addRect(rect)
        self._dirty = True      #signal that we want to be redrawn to paint
        self._viewPort.update() #tell QT that we want to be redrawn

    ## Gets the bounding rect of canvas
    def boundingRect(self):
        return self._rect

    ## Getter for shape (QPainterPath object)
    def shape(self):
        return self._shape

    def paint(self, painter, item, widget):
        if self._dirty:
            if self.get_canvas() is None: 
                return
            painter.save()
            self.get_canvas().draw(painter)
            painter.restore()

