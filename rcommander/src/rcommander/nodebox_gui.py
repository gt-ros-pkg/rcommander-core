#import roslib; roslib.load_manifest('rcommander_core')
import PyQt4.QtGui as qtg
import PyQt4.QtCore as qtc
import PyQt4.QtOpenGL as qtl
from nodebox import graphics

import time
import rospy
import threading

class AnimationRunner(threading.Thread):

    def __init__(self, f):
        threading.Thread.__init__(self)    
        self.f = f
        self.should_stop = False

    def run(self):
        time.sleep(2.)
        r = rospy.Rate(30)
        while not rospy.is_shutdown() and not self.should_stop:
            self.f()
            r.sleep()


class NodeBoxGUI:

    def __init__(self, graphics_view):

        #print 'GUI IS SHUTDOWN??1', rospy.is_shutdown()
        #add scene to QGraphicsView
        scene = qtg.QGraphicsScene()
        graphics_view.setViewport(qtl.QGLWidget())
        #print 'GUI IS SHUTDOWN??11', rospy.is_shutdown()
        self.drawing_widget = NodeBoxGUIHelper(graphics_view.viewport(), scene)
        #print 'GUI IS SHUTDOWN??111', rospy.is_shutdown()
        graphics_view.setScene(scene)
        # TODO: graphics_view._scene = scene
        #print 'GUI IS SHUTDOWN??112', rospy.is_shutdown()
        scene.setItemIndexMethod(qtg.QGraphicsScene.NoIndex)
        #print 'GUI IS SHUTDOWN??113', rospy.is_shutdown()
        scene.addItem(self.drawing_widget)
        #print 'GUI IS SHUTDOWN??2', rospy.is_shutdown()

        #Add NB to scene
        self.namespace = {}
        self.canvas = graphics.Canvas()
        #print 'text scale', QPixmap(1, 1).logicalDpiX() / 72.0
        self.canvas._setTextScaleFactor(qtg.QPixmap(1, 1).logicalDpiX() / 72.0)
        self.context = graphics.Context(self.canvas, self.namespace)

        #Setup the scene
        self.setup()
        self.animationTimer = qtc.QTimer(self)
        self.connect(self.animationTimer, qtc.SIGNAL("timeout()"), self._draw)
        self.start_drawing()
        #print 'GUI IS SHUTDOWN??3', rospy.is_shutdown()

        #self.animation_runner = AnimationRunner(self._draw)
        #self.animation_runner.start()

    def start_drawing(self):
        self.animationTimer.start(1000.0 / self.canvas.speed)

    def stop_drawing(self):
        self.animationTimer.stop()

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

class NodeBoxGUIHelper(qtg.QGraphicsWidget):

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

    def mousePressEvent(self, event):
        self.mousePosition = event.scenePos()

        if event.button() == qtc.Qt.LeftButton:
            self.mousedown = True 

        if event.button() == qtc.Qt.RightButton:
            self.rightdown = True

        self.setFocus()

    def mouseDoubleClickEvent(self, event):
        if event.button() == qtc.Qt.LeftButton:
            self.mousedoubleclick = True
            self.setFocus()

    def mouseMoveEvent(self, event):
        self.mousePosition = event.scenePos()

    def mouseReleaseEvent(self, event): 
        self.mousePosition = event.scenePos()
        if event.button() == qtc.Qt.LeftButton: 
           	self.mousedown = False 
        if event.button() == qtc.Qt.RightButton: 
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
                self._rect = rect = qtc.QRectF(0, 0, width, height)
                scene.setSceneRect(rect)
                self._shape = shape = qtg.QPainterPath()
                shape.addRect(rect)
        #self._check_cache()
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

    #def _check_cache(self):
    #    cacheMode = self.cacheMode()
    #    DeviceCoordinateCache = QGraphicsItem.DeviceCoordinateCache
    #    NoCache = QGraphicsItem.NoCache
    #    if self.get_canvas() is not None:
    #        #if self.document.animationTimer is not None:
    #        #    cache = False
    #        #elif len(self.get_canvas()) > 400:
    #        if len(self.get_canvas()) > 400:
    #            x, y, width, height = self._rect.getRect()
    #            cache = True
    #            if width > 1000 or height > 1000: #or \
    #                #not (width * self.zoom <= 1200 and height * self.zoom <= 1200):
    #                cache = False
    #        else:
    #            cache = False
    #        if cache and cacheMode != DeviceCoordinateCache:
    #            self.setCacheMode(DeviceCoordinateCache)
    #        elif not cache and cacheMode != NoCache:
    #            self.setCacheMode(NoCache)
    #    elif cacheMode != NoCache:
    #        self.setCacheMode(QNoCache)

