import sys
import os
import traceback, linecache
import re
import objc
import time
import random
import EasyDialogs
from PyObjCTools import NibClassBuilder, AppHelper
from Foundation import *
from AppKit import *
from threading import Thread

from nodebox.gui.mac.ValueLadder import MAGICVAR
from nodebox.gui.mac import PyDETextView
from nodebox.gui.mac.util import errorAlert
from nodebox import util
from nodebox.util import QTSupport
from nodebox import graphics

# AppleScript enumerator codes for PDF and Quicktime export
PDF = 0x70646678 # 'pdfx'
QUICKTIME = 0x71747878 # 'qt  '

VERY_LIGHT_GRAY = NSColor.blackColor().blendedColorWithFraction_ofColor_(
        0.95, NSColor.whiteColor())

NibClassBuilder.extractClasses("MainMenu")
NibClassBuilder.extractClasses("NodeBoxDocument")
NibClassBuilder.extractClasses("ExportImageAccessory")
NibClassBuilder.extractClasses("ExportMovieAccessory")
NibClassBuilder.extractClasses("ProgressBarSheet")
from nodebox.gui.mac.dashboard import *
from nodebox.gui.mac.progressbar import ProgressBarController

class ExportCommand(NSScriptCommand):
    pass    

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

# class defined in NodeBoxDocument.nib
class NodeBoxDocument(NibClassBuilder.AutoBaseClass):
    # the actual base class is NSDocument
    # The following outlets are added to the class:
    # graphicsView
    # outputView
    # textView
    # variablesController
    # dashboardController
    # The ExportImageAccessory adds:
    # exportImageAccessory
    # exportImageFormat
    # exportImagePageCount
    # The ExportMovieAccessory adds:
    # exportMovieAccessory
    # exportMovieFrame
    # exportMovieFps
    # When the PageCount accessory is loaded, we also add:
    # pageCount
    # pageCountAccessory
    # When the ExportSheet is loaded, we also add:
    # exportSheet
    # exportSheetIndicator

    path = None
    exportDir = None
    magicvar = None # Used for value ladders.
    _code = None
    vars = []
    movie = None

    def windowNibName(self):
        return "NodeBoxDocument"

    def init(self):
        self = super(NodeBoxDocument, self).init()
        nc = NSNotificationCenter.defaultCenter()
        nc.addObserver_selector_name_object_(self, "textFontChanged:", "PyDETextFontChanged", None)
        self.namespace = {}
        self.canvas = graphics.Canvas()
        self.context = graphics.Context(self.canvas, self.namespace)
        self.animationTimer = None
        self.__doc__ = {}
        self._pageNumber = 1
        self._frame = 150
        self.fullScreen = None
        self._seed = time.time()
        self.currentView = self.graphicsView
        return self

    def close(self):
        self.stopScript_()
        super(NodeBoxDocument, self).close()

    def __del__(self):
        nc = NSNotificationCenter.defaultCenter()
        nc.removeObserver_name_object_(self, "PyDETextFontChanged", None)
        # text view has a couple of circular refs, it can let go of them now
        self.textView._cleanup()

    def textFontChanged_(self, notification):
        font = PyDETextView.getBasicTextAttributes()[NSFontAttributeName]
        self.outputView.setFont_(font)

    def readFromFile_ofType_(self, path, tp):
        if self.textView is None:
            # we're not yet fully loaded
            self.path = path
        else:
            # "revert"
            self.readFromUTF8(path)
        return True

    def writeToFile_ofType_(self, path, tp):
        f = file(path, "w")
        text = self.textView.string()
        f.write(text.encode("utf8"))
        f.close()
        return True

    def windowControllerDidLoadNib_(self, controller):
        if self.path:
            self.readFromUTF8(self.path)
        font = PyDETextView.getBasicTextAttributes()[NSFontAttributeName]
        self.outputView.setFont_(font)
        self.textView.window().makeFirstResponder_(self.textView)
        self.windowControllers()[0].setWindowFrameAutosaveName_("NodeBoxDocumentWindow")

    def readFromUTF8(self, path):
        f = file(path)
        text = unicode(f.read(), "utf_8")
        f.close()
        self.textView.setString_(text)
        self.textView.usesTabs = "\t" in text

    def cleanRun(self, fn, newSeed = True):

        # Prepare everything for running the script
        self.prepareRun()

        # Run the actual script
        if self.fastRun(fn, newSeed):

            # Build the interface
            self.vars = self.namespace["_ctx"]._vars
            if len(self.vars) > 0:
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
        window = self.currentView.window()
        pt = window.mouseLocationOutsideOfEventStream()
        mx, my = window.contentView().convertPoint_toView_(pt, self.currentView)
        # Hack: mouse coordinates are flipped vertically in FullscreenView.
        # This flips them back.
        if isinstance(self.currentView, FullscreenView):
            my = self.currentView.bounds()[1][1] - my
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
        self.currentView.setCanvas(self.canvas)

        return True
        
    def handleRunScriptCommand_(self, command):
        self.runScript_(self)
        
    def runFullscreen_(self):
        if self.fullScreen is not None: return
        self.stopScript_()
        self.currentView = FullscreenView.alloc().init()
        self.currentView.canvas = None
        fullRect = NSScreen.mainScreen().frame()
        self.fullScreen = FullscreenWindow.alloc().init(fullRect)
        self.fullScreen.setContentView_(self.currentView)
        self.fullScreen.makeKeyAndOrderFront_(self)
        self.fullScreen.makeFirstResponder_(self.currentView)
        NSMenu.setMenuBarVisible_(False)
        NSCursor.hide()
        self._runScript()

    def runScript_(self, compile=True, newSeed=True):
        if self.fullScreen is not None: return
        self.currentView = self.graphicsView
        self._runScript(compile, newSeed)

    def _runScript(self, compile=True, newSeed=True):        
        if not self.cleanRun(self._execScript):
            pass

        # Check whether we are dealing with animation
        if self.canvas.speed is not None:
            if not self.namespace.has_key("draw"):
                errorAlert("Not a proper NodeBox animation",
                    "NodeBox animations should have at least a draw() method.")
                return

            # Check if animationTimer is already running
            if self.animationTimer is not None:
                self.stopScript_()

            self.speed = self.canvas.speed

            # Run setup routine
            if self.namespace.has_key("setup"):
                self.fastRun(self.namespace["setup"])
            window = self.currentView.window()
            window.makeFirstResponder_(self.currentView)

            # Start the timer
            self.animationTimer = NSTimer.scheduledTimerWithTimeInterval_target_selector_userInfo_repeats_(
                1.0 / self.speed, self, objc.selector(self.doFrame_, signature="v@:@"), None, True)

    def runScriptFast_(self):        
        if self.animationTimer is None:
            self.fastRun(self._execScript)
        else:
            # XXX: This can be sped up. We just run _execScript to get the
            # method with __MAGICVAR__ into the namespace, and execute
            # that, so it should only be called once for animations.
            self.fastRun(self._execScript)
            self.fastRun(self.namespace["draw"])

    def doFrame_(self):
        self.fastRun(self.namespace["draw"], newSeed=True)
        self._frame += 1
        
    def source(self):
        return self.textView.string()

    def setSource_(self, source):
        self.textView.setString_(source)

    def stopScript_(self):
        if self.animationTimer is not None:
            self.animationTimer.invalidate()
            self.animationTimer = None
        if self.fullScreen is not None:
            self.currentView = self.graphicsView
            self.fullScreen = None
            NSMenu.setMenuBarVisible_(True)
        NSCursor.unhide()
        self.textView.hideValueLadder()
        window = self.textView.window()
        window.makeFirstResponder_(self.textView)

    def _compileScript(self, source=None):
        if source is None:
            source = self.textView.string()
        #linecache.clearcache()
        #linecache.cache[fileName] = len(source), 0, source.splitlines(True), fileName
        self._code = None
        self._code = compile(source + "\n\n", self.scriptName, "exec")

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

        self.scriptName = self.fileName()
        libDir = os.path.join(os.getenv("HOME"), "Library", "Application Support", "NodeBox")
        if not self.scriptName:
            curDir = os.getenv("HOME")
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
                # Creating a thread is a heavy operation,
                # don't install it when animating, where speed is crucial
                t = Thread(target=self._userCancelledMonitor, name="UserCancelledMonitor")
                t.start()
            try:
                method(*args)
            except KeyboardInterrupt:
                self.stopScript_()
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
            #self._flushOutput()
        return True, output

    # from Mac/Tools/IDE/PyEdit.py
    def _userCancelledMonitor(self):
        import time
        from signal import SIGINT
        from Carbon import Evt
        while not self._scriptDone:
            if Evt.CheckEventQueueForUserCancel():
                # Send a SIGINT signal to ourselves.
                # This gets delivered to the main thread,
                # cancelling the running script.
                os.kill(os.getpid(), SIGINT)
                break
            time.sleep(0.25)

    def _flushOutput(self, output):
        outAttrs = PyDETextView.getBasicTextAttributes()
        errAttrs = outAttrs.copy()
        # XXX err color from user defaults...
        errAttrs[NSForegroundColorAttributeName] = NSColor.redColor()

        outputView = self.outputView
        outputView.setSelectedRange_((outputView.textStorage().length(), 0))
        lastErr = None
        for isErr, data in output:
            if isErr != lastErr:
                attrs = [outAttrs, errAttrs][isErr]
                outputView.setTypingAttributes_(attrs)
                lastErr = isErr
            outputView.insertText_(data)
        # del self.output

    def copyImageAsPDF_(self, sender):
        pboard = NSPasteboard.generalPasteboard()
        # graphicsView implements the pboard delegate method to provide the data
        pboard.declareTypes_owner_([NSPDFPboardType,NSPostScriptPboardType,NSTIFFPboardType], self.graphicsView)

    def exportAsImage_(self, sender):
        exportPanel = NSSavePanel.savePanel()
        exportPanel.setRequiredFileType_("pdf")
        exportPanel.setNameFieldLabel_("Export To:")
        exportPanel.setPrompt_("Export")
        exportPanel.setCanSelectHiddenExtension_(True)
        if not NSBundle.loadNibNamed_owner_("ExportImageAccessory", self):
            NSLog("Error -- could not load ExportImageAccessory.")
        self.exportImagePageCount.setIntValue_(1)
        exportPanel.setAccessoryView_(self.exportImageAccessory)
        path = self.fileName()
        if path:
            dirName, fileName = os.path.split(path)
            fileName, ext = os.path.splitext(fileName)
            fileName += ".pdf"
        else:
            dirName, fileName = None, "Untitled.pdf"
        # If a file was already exported, use that folder as the default.
        if self.exportDir is not None:
            dirName = self.exportDir
        exportPanel.beginSheetForDirectory_file_modalForWindow_modalDelegate_didEndSelector_contextInfo_(
            dirName, fileName, NSApp().mainWindow(), self,
            "exportPanelDidEnd:returnCode:contextInfo:", 0)

    def exportPanelDidEnd_returnCode_contextInfo_(self, panel, returnCode, context):
        if returnCode:
            fname = panel.filename()
            self.exportDir = os.path.split(fname)[0] # Save the directory we exported to.
            pages = self.exportImagePageCount.intValue()
            format = panel.requiredFileType()
            panel.close()
            self.doExportAsImage(fname, format, pages)
    exportPanelDidEnd_returnCode_contextInfo_ = objc.selector(exportPanelDidEnd_returnCode_contextInfo_,
            signature="v@:@ii")
            
    def exportImageFormatChanged_(self, sender):
        image_formats = ('pdf', 'png', 'tiff', 'jpg')
        panel = sender.window()
        panel.setRequiredFileType_(image_formats[sender.indexOfSelectedItem()])

    def doExportAsImage(self, fname, format, pages=1):
        basename, ext = os.path.splitext(fname)
        # When saving one page (the default), just save the current graphics
        # context. When generating multiple pages, we run the script again 
        # (so we don't use the current displayed view) for the first page, 
        # and then for every next page.
        if pages == 1:
            if not self.graphicsView.pdfData:
                self.runScript_()
            if format == 'pdf':
                pdfData = self.graphicsView.pdfData
                pdfData.writeToFile_atomically_(fname , False)
            else:
                self.canvas.save(fname, format)
        elif pages > 1:
            pb = ProgressBarController.alloc().init()
            pb.begin("Generating %s PDF files..." % pages, pages)
            try:
                if not self.cleanRun(self._execScript): return
                self._pageNumber = 1
                self._frame = 1

                # If the speed is set, we are dealing with animation
                if self.canvas.speed is None:
                    for i in range(pages):
                        if i > 0: # Run has already happened first time
                            self.fastRun(self._execScript, newSeed=True)
                        counterAsString = "-%5d" % self._pageNumber
                        counterAsString = counterAsString.replace(' ', '0')
                        exportName = basename + counterAsString + ext

                        if ext == '.pdf':
                            pdfData = self.graphicsView.pdfData
                            pdfData.writeToFile_atomically_(exportName, False)
                        else:
                            self.canvas.save(exportName, format)
                        self.graphicsView.setNeedsDisplay_(True)
                        self._pageNumber += 1
                        self._frame += 1
                        pb.inc()
                else:
                    if self.namespace.has_key("setup"):
                        self.fastRun(self.namespace["setup"])
                    for i in range(pages):
                        self.fastRun(self.namespace["draw"], newSeed=True)
                        counterAsString = "-%5d" % self._pageNumber
                        counterAsString = counterAsString.replace(' ', '0')
                        exportName = basename + counterAsString + ext

                        if ext == '.pdf':
                            pdfData = self.graphicsView.pdfData
                            pdfData.writeToFile_atomically_(exportName, False)
                        else:
                            self.canvas.save(exportName, format)
                        self.graphicsView.setNeedsDisplay_(True)
                        self._pageNumber += 1
                        self._frame += 1
                        pb.inc()
                        #self.exportSheetProgress.setDoubleValue_(i)
            except KeyboardInterrupt:
                pass
            pb.end()
            del pb
        self._pageNumber = 1
        self._frame = 1

    def handleExportScriptCommand_(self, command):
        print "ARGS"
        print command.arguments()
        print "FFF"
        try:
            print command.arguments()['File']
            print command.arguments()['File'].__class__.__name__
        except KeyError:
            print "F NOT FOUND"
        print "KEYS"
        print command.arguments().allKeys()
        print "VALUES"
        print command.arguments().allValues()
        print "CLASS"
        print command.__class__.__name__

        if command:
            args = command.arguments()
            ftype = PDF
            fcount = None
            fps = 30
            if args.has_key('ftype'):
                ftype = args['ftype']
            fname = None
            if args.has_key('fname'):
                f = args['fname']
                if f.isFileURL():
                    fname = f.path()
            if args.has_key('frames'):
                fcount = args['frames']
            if args.has_key('pages'):
                fcount = args['pages']
            if args.has_key('framerate'):
                fps = args['framerate']
            print fname
            if fname:
                if ftype == PDF:
                    if fcount is None: fcount = 1
                    self.doExportToPDF(fname, fcount)
                elif ftype == QUICKTIME:
                    if fcount is None: fcount = 60
                    self.doExportToQuickTime(fname, fcount, fps)

    def exportAsMovie_(self, sender):
        exportPanel = NSSavePanel.savePanel()
        exportPanel.setRequiredFileType_("pdf")
        exportPanel.setNameFieldLabel_("Export To:")
        exportPanel.setPrompt_("Export")
        exportPanel.setCanSelectHiddenExtension_(True)
        exportPanel.setAllowedFileTypes_(["mov"])
        if not NSBundle.loadNibNamed_owner_("ExportMovieAccessory", self):
            NSLog("Error -- could not load ExportMovieAccessory.")
        self.exportMovieFrames.setIntValue_(150)
        self.exportMovieFps.setIntValue_(30)
        exportPanel.setAccessoryView_(self.exportMovieAccessory)
        path = self.fileName()
        if path:
            dirName, fileName = os.path.split(path)
            fileName, ext = os.path.splitext(fileName)
            fileName += ".mov"
        else:
            dirName, fileName = None, "Untitled.mov"
        # If a file was already exported, use that folder as the default.
        if self.exportDir is not None:
            dirName = self.exportDir
        exportPanel.beginSheetForDirectory_file_modalForWindow_modalDelegate_didEndSelector_contextInfo_(
            dirName, fileName, NSApp().mainWindow(), self,
            "qtPanelDidEnd:returnCode:contextInfo:", 0)
                
    def qtPanelDidEnd_returnCode_contextInfo_(self, panel, returnCode, context):
        if returnCode:
            fname = panel.filename()
            self.exportDir = os.path.split(fname)[0] # Save the directory we exported to.
            frames = self.exportMovieFrames.intValue()
            fps = self.exportMovieFps.floatValue()
            panel.close()

            if frames <= 0 or fps <= 0: return
            self.doExportAsMovie(fname, frames, fps)
    qtPanelDidEnd_returnCode_contextInfo_ = objc.selector(qtPanelDidEnd_returnCode_contextInfo_,
            signature="v@:@ii")

    def doExportAsMovie(self, fname, frames=60, fps=30):
        try:
            os.unlink(fname)
        except:
            pass
        try:
            fp = open(fname, 'w')
            fp.close()
        except:
            errorAlert("File Error", "Could not create file '%s'. Perhaps it is locked or busy." % fname)
            return

        movie = None

        pb = ProgressBarController.alloc().init()
        pb.begin("Generating %s frames..." % frames, frames)
        try:
            if not self.cleanRun(self._execScript): return
            self._pageNumber = 1
            self._frame = 1

            movie = QTSupport.Movie(fname, fps)
            # If the speed is set, we are dealing with animation
            if self.canvas.speed is None:
                for i in range(frames):
                    if i > 0: # Run has already happened first time
                        self.fastRun(self._execScript, newSeed=True)
                    movie.add(self.canvas)
                    self.graphicsView.setNeedsDisplay_(True)
                    pb.inc()
                    self._pageNumber += 1
                    self._frame += 1
            else:
                if self.namespace.has_key("setup"):
                    self.fastRun(self.namespace["setup"])
                for i in range(frames):
                    self.fastRun(self.namespace["draw"], newSeed=True)
                    movie.add(self.canvas)
                    self.graphicsView.setNeedsDisplay_(True)
                    pb.inc()
                    self._pageNumber += 1
                    self._frame += 1

        except KeyboardInterrupt:
            pass
        pb.end()
        del pb
        movie.save()
        self._pageNumber = 1
        self._frame = 1

    def printDocument_(self, sender):
        op = NSPrintOperation.printOperationWithView_printInfo_(self.graphicsView, self.printInfo())
        op.runOperationModalForWindow_delegate_didRunSelector_contextInfo_(
            NSApp().mainWindow(), self, "printOperationDidRun:success:contextInfo:",
            0)

    def printOperationDidRun_success_contextInfo_(self, op, success, info):
        if success:
            self.setPrintInfo_(op.printInfo())

    printOperationDidRun_success_contextInfo_ = objc.selector(printOperationDidRun_success_contextInfo_,
            signature="v@:@ci")

    def buildInterface_(self):
        self.dashboardController.buildInterface_(self.vars)

    def validateMenuItem_(self, menuItem):
        if menuItem.action() in ("exportAsImage:", "exportAsMovie:"):
            return self.canvas is not None
        return True
        
class FullscreenWindow(NibClassBuilder.AutoBaseClass):
    
    def init(self, fullRect):
        super(FullscreenWindow, self).initWithContentRect_styleMask_backing_defer_(fullRect, NSBorderlessWindowMask, NSBackingStoreBuffered, True)
        return self
        
    def canBecomeKeyWindow(self):
        return True

class FullscreenView(NibClassBuilder.AutoBaseClass):
    
    def init(self):
        super(FullscreenView, self).init()
        self.mousedown = False
        self.keydown = False
        self.key = None
        self.keycode = None
        self.scrollwheel = False
        self.wheeldelta = 0.0
        return self
        
    def setCanvas(self, canvas):
        self.canvas = canvas
        self.setNeedsDisplay_(True)
        if not hasattr(self, "screenRect"):
            self.screenRect = NSScreen.mainScreen().frame()
            cw, ch = self.canvas.size
            sw, sh = self.screenRect[1]
            self.scalingFactor = calc_scaling_factor(cw, ch, sw, sh)
            nw, nh = cw * self.scalingFactor, ch * self.scalingFactor
            self.scaledSize = nw, nh
            self.dx = (sw - nw) / 2.0
            self.dy = (sh - nh) / 2.0

    def drawRect_(self, rect):
        NSGraphicsContext.currentContext().saveGraphicsState()        
        NSColor.blackColor().set()
        NSRectFill(rect)
        if self.canvas is not None:
            t = NSAffineTransform.transform()
            t.translateXBy_yBy_(self.dx, self.dy)
            t.scaleBy_(self.scalingFactor)
            t.concat()
            clip = NSBezierPath.bezierPathWithRect_( ((0, 0), (self.canvas.width, self.canvas.height)) )
            clip.addClip()
            self.canvas.draw()
        NSGraphicsContext.currentContext().restoreGraphicsState()

    def isFlipped(self):
        return True

    def mouseDown_(self, event):
        self.mousedown = True

    def mouseUp_(self, event):
        self.mousedown = False

    def keyDown_(self, event):
        self.keydown = True
        self.key = event.characters()
        self.keycode = event.keyCode()

    def keyUp_(self, event):
        self.keydown = False
        self.key = event.characters()
        self.keycode = event.keyCode()

    def scrollWheel_(self, event):
        self.scrollwheel = True
        self.wheeldelta = event.deltaY()

    def canBecomeKeyView(self):
        return True

    def acceptsFirstResponder(self):
        return True

def calc_scaling_factor(width, height, maxwidth, maxheight):
    if width > height:
        return float(maxwidth) / width
    else:
        return float(maxheight) / height

# class defined in NodeBoxGraphicsView.nib
class NodeBoxGraphicsView(NibClassBuilder.AutoBaseClass):
    # the actual base class is NSView
    # The following outlets are added to the class:
    # document
    
    def awakeFromNib(self):
        self.canvas = None
        self._image = None
        self._dirty = False
        self.mousedown = False
        self.keydown = False
        self.key = None
        self.keycode = None
        self.scrollwheel = False
        self.wheeldelta = 0.0
        self.setFrameSize_( (graphics.DEFAULT_WIDTH, graphics.DEFAULT_HEIGHT) )
        self.setFocusRingType_(NSFocusRingTypeExterior)
        if self.superview() is not None:
            self.superview().setBackgroundColor_(VERY_LIGHT_GRAY)

    def setCanvas(self, canvas):
        self.canvas = canvas
        if self.frame()[1] != self.canvas.size:
            self.setFrameSize_(self.canvas.size)
        self.markDirty()
    
    def markDirty(self, redraw=True):
        self._dirty = True
        if redraw:
            self.setNeedsDisplay_(True)

    def setFrameSize_(self, size):
        self._image = None
        NSView.setFrameSize_(self, size)

    def isOpaque(self):
        return False

    def isFlipped(self):
        return True
        
    def drawRect_(self, rect):
        if self.canvas is not None:
            NSGraphicsContext.currentContext().saveGraphicsState()
            try:
                self.canvas.draw()
            except:
                # A lot of code just to display the error in the output view.
                etype, value, tb = sys.exc_info()
                if tb.tb_next is not None:
                    tb = tb.tb_next  # skip the frame doing the exec
                traceback.print_exception(etype, value, tb)
                data = "".join(traceback.format_exception(etype, value, tb))
                attrs = PyDETextView.getBasicTextAttributes()
                attrs[NSForegroundColorAttributeName] = NSColor.redColor()
                outputView = self.document.outputView
                outputView.setSelectedRange_((outputView.textStorage().length(), 0))
                outputView.setTypingAttributes_(attrs)
                outputView.insertText_(data)
            NSGraphicsContext.currentContext().restoreGraphicsState()

    def _updateImage(self):
        if self._dirty:
            self._image = self.canvas._nsImage
            self._dirty = False

    # pasteboard delegate method
    def pasteboard_provideDataForType_(self, pboard, type):
        if NSPDFPboardType:
            pboard.setData_forType_(self.pdfData, NSPDFPboardType)
        elif NSPostScriptPboardType:
            pboard.setData_forType_(self.epsData, NSPostScriptPboardType)
        elif NSTIFFPboardType:
            pboard.setData_forType_(self.tiffData, NSTIFFPboardType)
            
    def _get_pdfData(self):
        if self.canvas:
            return self.dataWithPDFInsideRect_(((0, 0), self.canvas.size))        
    pdfData = property(_get_pdfData)
    
    def _get_epsData(self):
        if self.canvas:
            return self.dataWithEPSInsideRect_(((0, 0), self.canvas.size))
    epsData = property(_get_epsData)

    def _get_tiffData(self):
        return self.image.TIFFRepresentation()
    tiffData = property(_get_tiffData)
 
    def _get_pngData(self):
        return NSBitmapImageRep.imageRepWithData_(self.tiffData).representationUsingType_properties_(NSPNGFileType, None)
    pngData = property(_get_pngData)
    
    def _get_gifData(self):
        return NSBitmapImageRep.imageRepWithData_(self.tiffData).representationUsingType_properties_(NSGIFFileType, None)
    gifData = property(_get_gifData)

    def _get_jpegData(self):
        return NSBitmapImageRep.imageRepWithData_(self.tiffData).representationUsingType_properties_(NSJPEGFileType, None)
    jpegData = property(_get_jpegData)

    def _get_image(self):
        if self.canvas is not None:
            self._updateImage()
            return self._image
        else:
            return NSImage.alloc().initWithSize_(self.bounds[1])
    image = property(_get_image)
    
    def mouseDown_(self, event):
        self.mousedown = True
        
    def mouseUp_(self, event):
        self.mousedown = False
        
    def keyDown_(self, event):
        self.keydown = True
        self.key = event.characters()
        self.keycode = event.keyCode()
        
    def keyUp_(self, event):
        self.keydown = False
        self.key = event.characters()
        self.keycode = event.keyCode()
        
    def scrollWheel_(self, event):
        self.scrollwheel = True
        self.wheeldelta = event.deltaY()

    def canBecomeKeyView(self):
        return True

    def acceptsFirstResponder(self):
        return True

class NodeBoxAppDelegate(NibClassBuilder.AutoBaseClass):

    def awakeFromNib(self):
        self._prefsController = None
        libDir = os.path.join(os.getenv("HOME"), "Library", "Application Support", "NodeBox")
        try:
            if not os.path.exists(libDir):
                os.mkdir(libDir)
                f = open(os.path.join(libDir, "README"), "w")
                f.write("In this directory, you can put Python libraries to make them available to your scripts.\n")
                f.close()
        except OSError: pass
        except IOError: pass

    def showPreferencesPanel_(self, sender):
        if self._prefsController is None:
            from nodebox.gui.mac.preferences import NodeBoxPreferencesController
            self._prefsController = NodeBoxPreferencesController.alloc().init()
        self._prefsController.showWindow_(sender)

    def generateCode_(self, sender):
        """Generate a piece of NodeBox code using OttoBot"""
        from nodebox.util.ottobot import genProgram
        controller = NSDocumentController.sharedDocumentController()
        doc = controller.newDocument_(sender)
        doc = controller.currentDocument()
        doc.textView.setString_(genProgram())
        doc.runScript_()

    def showSite_(self, sender):
        url = NSURL.URLWithString_("http://nodebox.net/")
        NSWorkspace.sharedWorkspace().openURL_(url)

