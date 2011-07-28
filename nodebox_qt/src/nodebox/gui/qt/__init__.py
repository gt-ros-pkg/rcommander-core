import codecs
import sys
import os
import traceback, linecache
import re
import time
import random

from PyQt4.QtGui import *
from PyQt4.QtCore import *

from nodebox.gui.qt.baseclasses import NodeBoxDocumentBaseClass, NodeBoxApplicationBaseClass, ExportMoviePanel
from nodebox.gui.qt.document import DocumentController
from nodebox.gui.qt.ValueLadder import MAGICVAR
from nodebox.gui.qt.util import errorAlert
from nodebox.gui.qt.progressbar import ProgressBarController
from nodebox.gui.qt.findreplace import FindReplaceController
from nodebox.gui.qt.dashboard import DashboardController

MAGICVAR = "__magic_var__"

from nodebox import graphics
from nodebox import util

class OutputFile(object):

    def __init__(self, data, isErr=False):
        self.data = data
        self.isErr = isErr

    def write(self, data):
        if isinstance(data, str):
            try:
                data = unicode(data, "utf_8", "replace")
            except UnicodeDecodeError:
                data = "XXX " + repr(data)
        self.data.append((self.isErr, data))


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
        self.keydown = False
        self.key = None
        self.keycode = None        
        self.scrollwheel = False
        self.wheeldelta = 0.0
        self._zoom = 1.0
        
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
            self.setFocus()

        if event.button() == Qt.RightButton:
            self.rightdown = True
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
        
class NodeBoxDocument(NodeBoxDocumentBaseClass):
    
    path = None
    exportDir = None
    magicvar = None # Used for value ladders.
    _code = None
    vars = []
    
    def __init__(self, *args):
        NodeBoxDocumentBaseClass.__init__(self, *args)
        
        self.namespace = {}
        self.canvas = graphics.Canvas()
        textScaleFactor = QPixmap(1, 1).logicalDpiX() / 72.0
        self.canvas._setTextScaleFactor(textScaleFactor)
        self.context = graphics.Context(self.canvas, self.namespace)
        self.animationTimer = None
        self.__doc__ = {}
        self._pageNumber = 1
        self._frame = 150
        self.fullScreen = None
        self._seed = time.time()
        self.currentView = self.graphicsView
        self._frame_stream = None
        self._recordTimer = QBasicTimer()
        self._paddedSize = None
        self._pb = None
        
    def close(self):
        self.stop()
        return NodeBoxDocumentBaseClass.close(self)

    def readFromFile(self, path):
        return self.readFromUTF8(path)

    def readFromUTF8(self, path):
        err = False
        f = None
        try:
            try:
                f = file(path)
                text = unicode(f.read(), "utf_8")
                self.textView.setPlainText(text)
            except (IOError, OSError), e:
                errorAlert(None, "Failed to load %s: %s" % (path, e))
                err = True
            except UnicodeError, e:
                errorAlert(None, "(%s)" % e)
                err = True
        finally:
            if f is not None:
                f.close()
            success = not err
            if success:
                self.textView.moveCursor(QTextCursor.End, QTextCursor.MoveAnchor)
            return success
    
    def writeToFile(self, path):
        return self.writeToUTF8(path)
    
    def writeToUTF8(self, path):
        err = False
        f = None
        try:
            try:
                f = file(path, "w")
                text = unicode(self.textView.toPlainText())
                f.write(text.encode("utf_8"))
            except (IOError, OSError), e:
                errorAlert(None, "Failed to save %s: %s" % (path, e))
                err = True
        finally:
            if f is not None:
                f.close()
            success = not err
            if success:
                self.textView.document().setModified(False)
            return success
            
    def zoomToTag_(self):
        value, exists = self.sender().property("tag").toInt()
        self.graphicsView.zoomTo_(value / 100.)

    def dragZoom(self):
        if self.fullScreen is not None: return
        self.graphicsView.dragZoom_(self.zoomSlider.value())
    
    def exportAsImage(self):
        if self.canvas is None: return 
        fileName = QFileDialog.getSaveFileName(self, self.tr("Save SVG File"), app.home, "*.svg,*.pdf,*.png,*.tiff,*.jpg,*.jpeg") 
        if fileName is not None and len(fileName) > 0: 
            self.canvas.save(str(fileName))

    def exportAsMovie(self):
        exportPanel = ExportMoviePanel(self)
        exportPanel.allowedFileTypes = "Movie Files (*.mov, *.avi, *.mp4)"
        exportPanel.exportMovieFrames.setText("150")
        exportPanel.exportMovieFps.setText("30")
        path = self.fileName
        if path is not None:
            dirName, fileName = os.path.split(path)
            fileName, ext = os.path.splitext(fileName)
            fileName += ".avi"
        else:
            dirName, fileName = unicode(app.documentController.lastDir), "Untitled.avi"
        # If a file was already exported, use that folder as the default.
        if self.exportDir is not None:
            dirName = self.exportDir
        exportPanel.fileName = os.path.join(dirName, fileName)
        exportPanel.show()
        self.connect(exportPanel, SIGNAL("accepted()"), self.exportAsMovie_)
    	
    def exportAsMovie_(self):
        panel = self.sender()
        if panel is not None:
            fname = panel.fileName
            self.exportDir = os.path.split(fname)[0] # Save the directory we exported to.
            frames, ok = panel.exportMovieFrames.text().toInt()
            if not ok: return
            fps, ok = panel.exportMovieFps.text().toDouble()
            if not ok: return
            format = panel.format

            if frames <= 0 or fps <= 0: return
            self.doExportAsMovie(fname, format, frames, fps)

    def doExportAsMovie(self, fname, format, frames=60, fps=30):
        try:
            if not self.cleanRun(self._execScript): return
            cmd = None
            if sys.platform == "darwin":
                cmd = "Resources/ffmpeg"
            elif "linux" in sys.platform:
                cmd = "ffmpeg"
            if cmd is not None:
                x, y, z = os.popen3(cmd)
                err = z.read().lower()
                if "command not found" in err or "no such file or directory" in err:
                    cmd = None
            if cmd is None:
                return
            self._pb = ProgressBarController(self)
            self.connect(self._pb, SIGNAL("progressAborted()"), self._stopRecording)
            self._pb.begin("Generating %s frames..." % frames, frames)
            self._pageNumber = 1
            self._frame = 1
            self._frame_stream = QProcess(self)
            self._frame_stream.frames = frames
            self.connect(self._frame_stream, SIGNAL("finished(int)"), self.recordingFinished)
            self._recordTimer.start(20, self)
            proc = self._frame_stream
            inputOptions = ["-y", "-f", "rawvideo", "-pix_fmt", "rgb24", "-r", str(fps), "-s", "%sx%s" % (self.canvas.width, self.canvas.height)]
            if format == ExportMoviePanel.MPEG4:
                proc.start(cmd, inputOptions + ["-i", "-", "-vcodec", "mpeg4", "-sameq", "-b", "1250kb", fname])
            elif format == ExportMoviePanel.XVID:
                proc.start(cmd, inputOptions + ["-i", "-", "-vcodec", "libxvid", "-pix_fmt", "yuv420p", "-sameq", "-aspect", str(float(self.canvas.width) / self.canvas.height), "-flags", "+4mv", "-trellis", "-aic", "-cmp", "2", "-subcmp", "2", "-g", "300", fname]) # xvid
            elif format == ExportMoviePanel.X264:
                proc.start(cmd, inputOptions + ["-aspect", "1:1",  "-i", "-", "-vcodec", "libx264", "-b", "1250kb", "-coder", "1", "-cmp", "+chroma", "-partitions", "+parti8x8+parti4x4+partp8x8+partb8x8", "-me_method", "hex", "-subq", "6", "-me_range", "16", "-g", "250", "-keyint_min", "25", "-sc_threshold", "40", "-i_qfactor", "0.71", "-b_strategy", "1", "-qcomp", "0.6", "-qmin", "10", "-qmax", "51", "-qdiff", "4", "-directpred", "1", "-flags2", "+fastpskip", "-threads", "0", "-an", fname])
            elif format == ExportMoviePanel.MPEG1:
                proc.start(cmd, inputOptions + ["-i", "-", "-sameq", "-vcodec", "mpeg1video", fname])
            elif format == ExportMoviePanel.FLV:
                proc.start(cmd, inputOptions + ["-aspect", "1:1",  "-i", "-", "-f", "flv", "-vcodec", "flv", "-nr", "500", "-b", "300kb", "-me_range", "25", "-i_qfactor", "0.9", "-qmin", "8", "-qmax", "8", "-g", "500", "-an", fname])
            elif format == ExportMoviePanel.HUFFYUV:
                proc.start(cmd, inputOptions + ["-i", "-", "-sameq", "-vcodec", "huffyuv", fname])

        except KeyboardInterrupt:
            pass

    def recordingFinished(self):
        sender = self.sender()
        if sender is not None:
            self._pb.end()
            self._pb = None
            if self._recordTimer.isActive():
                self._recordTimer.stop()
            if sender.exitCode() != 0:
                err = unicode(self._frame_stream.readAllStandardError())
                errorAlert("An unexpected error occurred.", err)
            self._frame_stream.close()
            self._frame_stream = None

    def _addFrame(self):
        img = self.canvas._qImage
        img = img.convertToFormat(QImage.Format_RGB888)
        if self._paddedSize is not None:
            width, height = self._paddedSize
            im = img
            img = QImage(width, height, QImage.Format_RGB888)
            p = QPainter(img)
            p.fillRect(img.rect(), Qt.black)
            p.drawImage(0, 0, im)
            p.end()
        self._frame_stream.writeData(img.bits().asstring(img.numBytes()))
        self.repaint()
        self._pb.inc()
        self._pageNumber += 1
        self._frame += 1
        
    def recordingEvent(self, event):
        if self._frame_stream is not None and self._frame_stream.state() == QProcess.Running:
            # If the speed is set, we are dealing with animation
            if self.canvas.speed is None:
                if self._frame > 1: # Run has already happened first time
                    self.fastRun(self._execScript, newSeed=True)
                self._addFrame()
            else:
                if self._frame == 1:
                    self.fastRun(self.namespace["setup"])
                self.fastRun(self.namespace["draw"], newSeed=True)
                self._addFrame()
            if self._frame == self._frame_stream.frames + 1:
                self._stopRecording()
    timerEvent = recordingEvent
    
    def _stopRecording(self):
        self._recordTimer.stop()
        self._frame_stream.closeWriteChannel()
        self._pageNumber = 1
        self._frame = 1

    def buildInterface_(self): 
        self.dashboardController.buildInterface_(self.vars)
        
    def print_(self):
        pass

    def run(self):
        if self.fullScreen is not None: return
        self.currentView = self.graphicsView
        self._runScript()

    def runFullscreen(self):
        pass

    def stop(self):
        if self.animationTimer is not None:
            self.animationTimer.stop()
            self.animationTimer = None
        if self.fullScreen is not None:
#            self.currentView.close()
            self.currentView = self.graphicsView
            self.fullScreen = None
            # TODO: Menu
            #NSMenu.setMenuBarVisible_(True)
        QApplication.restoreOverrideCursor()
        #self.textView.hideValueLadder()
        self.textView.setFocus()
        self.animationSpinner.stop()

    def zoomIn(self):
        if self.fullScreen is not None: return        
        self.graphicsView.zoomIn_()        

    def zoomOut(self):
        if self.fullScreen is not None: return
        self.graphicsView.zoomOut_()        

    def zoomToFit(self):
        if self.fullScreen is not None: return
        self.graphicsView.zoomToFit_()

    def _get_source(self):
        return self.textView.toPlainText()
    def _set_source(self, source):
        self.textView.setPlainText(source)
    source = property(_get_source, _set_source)

    def _runScript(self, compile=True, newSeed=True):
        if not self.cleanRun(self._execScript, newSeed):
            pass

        # Check whether we are dealing with animation
        if self.canvas.speed is not None:
            if not self.namespace.has_key("draw"):
                errorAlert("Not a proper NodeBox animation",
                    "NodeBox animations should have at least a draw() method.")
                return

            # Check if animationTimer is already running
            if self.animationTimer is not None:
                self.stop()

            self.speed = self.canvas.speed

            self.animationSpinner.start()

            # Run setup routine
            if self.namespace.has_key("setup"):
                self.fastRun(self.namespace["setup"])
            self.currentView.setFocus()

            # Start the timer
            self.animationTimer = QTimer(self)
            self.connect(self.animationTimer, SIGNAL("timeout()"), self.doFrame)
            self.animationTimer.start(1000.0 / self.speed)

    def runScriptFast_(self):        
        if self.animationTimer is None:
            self.fastRun(self._execScript)
        else:
            # XXX: This can be sped up. We just run _execScript to get the
            # method with __MAGICVAR__ into the namespace, and execute
            # that, so it should only be called once for animations.
            self.fastRun(self._execScript)
            self.fastRun(self.namespace["draw"])

    def cleanRun(self, fn, newSeed = True):
        # Prepare everything for running the script
#        self.animationSpinner.start()
        self.prepareRun()

        success = self.fastRun(fn, newSeed)
#        self.animationSpinner.stop()

        # Run the actual script
        if success:

            # Build the interface
            self.vars = self.namespace["_ctx"]._vars
            if newSeed and len(self.vars) > 0:
                self.buildInterface_()

            return True

        return False

    def prepareRun(self):

        # Compile the script
        success, output = self._boxedRun(self._compileScript)
        self._flushOutput(output)
        if not success:
            return False

        # Initialize the namespace
        self._initNamespace()

        # Reset the pagenum
        self._pageNum = 1

        # Reset the frame
        self._frame = 1

        self.speed = self.canvas.speed = None

    def fastRun(self, fn, newSeed = False):

        # Check if there is code to run
        if self._code is None:
            return False

        # Clear the canvas
        self.canvas.clear()

        # Generate a new seed, if needed
        if newSeed:
            self._seed = time.time()
        random.seed(self._seed)

        # Set the mouse position
        pos = self.currentView.mousePosition
        mx, my = pos.x(), pos.y()

        self.namespace["MOUSEX"], self.namespace["MOUSEY"] = mx, my
        self.namespace["mousedown"] = self.currentView.mousedown
        self.namespace["keydown"] = self.currentView.keydown
        self.namespace["key"] = self.currentView.key
        self.namespace["keycode"] = self.currentView.keycode
        self.namespace["scrollwheel"] = self.currentView.scrollwheel
        self.namespace["wheeldelta"] = self.currentView.wheeldelta

        # Reset the context
        self.context._resetContext()

        # Initalize the magicvar
        self.namespace[MAGICVAR] = self.magicvar

        # Set the pagenum
        self.namespace['PAGENUM'] = self._pageNumber

        # Set the frame
        self.namespace['FRAME'] = self._frame

        # Run the script
        success, output = self._boxedRun(fn)
        self._flushOutput(output)
        if not success:
            return False

        # Display the output of the script
        self.currentView.canvas = self.canvas

        return True

    def doFrame(self):
        self.fastRun(self.namespace["draw"], newSeed=True)
        self._frame += 1

    def _boxedRun(self, method, args=[]):
        """
        Runs the given method in a boxed environment.
        Boxed environments:
         - Have their current directory set to the directory of the file
         - Have their argument set to the filename
         - Have their outputs redirect to an output stream.
        Returns:
           A tuple containing:
             - A boolean indicating whether the run was successful
             - The OutputFile
        """
        self.scriptName = self.fileName
        libDir = app.lib
        if not self.scriptName:
            curDir = app.home
            self.scriptName = "<untitled>"
        else:
            curDir = os.path.dirname(self.scriptName)
        save = sys.stdout, sys.stderr
        saveDir = os.getcwd()
        saveArgv = sys.argv
        sys.argv = [self.scriptName]
        if os.path.exists(libDir):
            sys.path.insert(0, libDir)
        os.chdir(curDir)
        sys.path.insert(0, curDir)
        output = []
        sys.stdout = OutputFile(output, False)
        sys.stderr = OutputFile(output, True)
        self._scriptDone = False
        try:
            if self.animationTimer is None:
                pass
                # Creating a thread is a heavy operation,
                # don't install it when animating, where speed is crucial
#                t = Thread(target=self.animationSpinner.start, name="UserCancelledMonitor")
#                t.start()
            try:
                method(*args)
            except KeyboardInterrupt:
                self.stop()
            except:
                etype, value, tb = sys.exc_info()
                if tb.tb_next is not None:
                    tb = tb.tb_next  # skip the frame doing the exec
                traceback.print_exception(etype, value, tb)
                etype = value = tb = None
                return False, output
        finally:
            self._scriptDone = True
            sys.stdout, sys.stderr = save
            os.chdir(saveDir)
            sys.path.remove(curDir)
            try:
                sys.path.remove(libDir)
            except ValueError:
                pass
            sys.argv = saveArgv
        return True, output

    def _flushOutput(self, output):
        outputView = self.outputView
        lastErr = None
        outColor = QColor(0, 0, 0)
        errColor = QColor(255, 0, 0)
        for isErr, data in output:
            if isErr != lastErr:
                if isErr:
                    outputView.setTextColor(errColor)
                else:
                    outputView.setTextColor(outColor)
                lastErr = isErr
            outputView.insertPlainText(data)
        outputView.ensureCursorVisible()

    def _compileScript(self, source=None):
        if source is None:
            source = self.textView.toPlainText()
        source = unicode(source)
        self._code = None
        self._code = compile(source + "\n\n", self.scriptName.encode('ascii', 'ignore'), "exec")

    def _initNamespace(self):
        self.namespace.clear()
        # Add everything from the namespace
        for name in graphics.__all__:
            self.namespace[name] = getattr(graphics, name)
        for name in util.__all__:
            self.namespace[name] = getattr(util, name)
        # Add everything from the context object
        self.namespace["_ctx"] = self.context
        for attrName in dir(self.context):
            self.namespace[attrName] = getattr(self.context, attrName)
        # Add the document global
        self.namespace["__doc__"] = self.__doc__
        # Add the page number
        self.namespace["PAGENUM"] = self._pageNumber
        # Add the frame number
        self.namespace["FRAME"] = self._frame
        # Add the magic var
        self.namespace[MAGICVAR] = self.magicvar
        # XXX: will be empty after reset.
        #for var in self.vars:
        #    self.namespace[var.name] = var.value

    def _execScript(self):
        exec self._code in self.namespace
        self.__doc__ = self.namespace.get("__doc__", self.__doc__)

class NodeBoxApplication(NodeBoxApplicationBaseClass):
    def __init__(self, args):
        NodeBoxApplicationBaseClass.__init__(self, args)
        self.setObjectName("NodeBoxApplication")
        self.home = os.getenv("USERPROFILE") or os.getenv("HOME")
        
        if sys.platform == "win32":
            self.lib = os.path.join(os.getenv("SYSTEMDRIVE") + os.path.sep, "NodeBox")
        elif sys.platform == "darwin":
            self.lib = os.path.join(unicode(QDesktopServices.storageLocation(QDesktopServices.DataLocation)), "NodeBox")
        else:
            self.lib = os.path.join(self.home, "NodeBox")
            
        self._prefsController = None
        self._findReplaceController = FindReplaceController()
        self.documentController = DocumentController(self, NodeBoxDocument, "Python Source Files (*.py)")
        self.connect(self, SIGNAL("focusChanged(QWidget *, QWidget *)"), self._focusChanged)

        QCoreApplication.setOrganizationName("NodeBox")
        QCoreApplication.setOrganizationDomain("nodebox.net")
        QCoreApplication.setApplicationName("NodeBox")
    
    def _focusChanged(self, oldWidget, newWidget):
        if self._findReplaceController is not None:
            try:
                if self._findReplaceController.textEdit == newWidget: return
                widget = newWidget
                while True:
                    widget = widget.parent()
                    if widget.isWindow():
                        break
                if not isinstance(newWidget, QTextEdit):
                    newWidget = None
                if isinstance(widget, NodeBoxDocument):
                    self._findReplaceController.textEdit = newWidget
            except:
                pass
                    
    def newNodeBoxGraphicsView(self):
        return NodeBoxGraphicsView()
        
    def generateCode(self):
        """Generate a piece of NodeBox code using OttoBot"""
        from nodebox.util.ottobot import genProgram
        doc = self.documentController.newDocument()
        doc.source = genProgram()
        doc.run()

    def showPreferencesPanel(self):
        if self._prefsController is None:
            from nodebox.gui.qt.preferences import NodeBoxPreferencesController
            self._prefsController = NodeBoxPreferencesController()
            self.connect(self._prefsController, SIGNAL("textFontChanged()"), NodeBoxDocument.updateFont)
        self._prefsController.show()
        self._prefsController.raise_()
        self._prefsController.activateWindow()

    def showFindReplacePanel(self):
        self._findReplaceController.show()
        self._findReplaceController.raise_()
        self._findReplaceController.activateWindow()
        self._findReplaceController.clearFeedback()

    def performFindPanelAction(self, action):
        if action == FindReplaceController.SHOW_PANEL:
            self.showFindReplacePanel()
        elif self._findReplaceController is not None:
            self._findReplaceController.performAction(action, self.sender())
        elif action in [FindReplaceController.FIND_NEXT, FindReplaceController.FIND_PREVIOUS]:
            self.beep()

    def showHelp(self):
        f = QFileInfo("Resources/NodeBox Help/index.html")
        if f.exists():
            url = QUrl.fromLocalFile(f.absoluteFilePath())
            QDesktopServices.openUrl(url)
        
    def showSite(self):
        url = QUrl("http://nodebox.net/")
        QDesktopServices.openUrl(url)

def qtmain():
    global app
    app = NodeBoxApplication(sys.argv)
    app.documentController.newDocument()
    sys.exit(app.exec_())

if __name__ == "__main__":
    qtmain()
