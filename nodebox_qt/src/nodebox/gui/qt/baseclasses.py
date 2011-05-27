import sys
import os

from PyQt4.QtGui import QApplication, QMainWindow, QDialog, QColor, QPalette, QIcon, QSizePolicy, QDialogButtonBox, QFileDialog, QGraphicsScene
from PyQt4.QtCore import Qt, QSettings, QVariant, SIGNAL, QEvent, QPoint
from PyQt4 import uic

from nodebox.gui.qt.document import DocumentWindow
from nodebox.gui.qt.dashboard import DashboardController
from nodebox.gui.qt.util import chooseAction
import os.path as pt
import pdb

RESOURCEFOLDER = "Resources/qt/ui/"

def formClass(className, resourceFolder=RESOURCEFOLDER):
    resourceFolder = pt.join(os.sep + pt.join(*__file__.split(os.sep)[:-4]), resourceFolder)
    frm_class, base_class = uic.loadUiType("%s%s.ui" % (resourceFolder, className))
    return frm_class

def split(num, factor):
    num1 = int(round(num * factor))
    num2 = num - num1
    return [num1, num2]

class NodeBoxDocumentBaseClass(DocumentWindow, formClass("NodeBoxDocument")):

    styleName = None
    windowSize = None
    windowPosition = None
    windowMaximized = None
    frameXOffset = 21
    frameYOffset = 23
    
    def __init__(self, *args):
        DocumentWindow.__init__(self, *args)
        self.setAttribute(Qt.WA_DeleteOnClose)
        self.initStyleName()
        self.setProperty("style", QVariant(self.styleName))
        self.setupUi(self)
        self.setupActions()
        self.graphicsView.document = self
        self.textView._document = self
        self.dashboardController = DashboardController(self) 
        
    def show_(self):
        if self.windowMaximized:
            self.showMaximized()
        DocumentWindow.show_(self)
        
    def initStyleName(self):
        if NodeBoxDocumentBaseClass.styleName is None:
            styleName = str(self.style().metaObject().className())[1:-5].lower()
            NodeBoxDocumentBaseClass.styleName = styleName

    def _initSizeAndPosition(self):
        if self.windowPosition is None:
            pos = QPoint(283, self.app.desktop().height() - self.height() - 378)
            NodeBoxDocumentBaseClass.windowPosition = self.settings.value("windowPosition", QVariant(pos)).toPoint()
        if self.windowSize is None:
            NodeBoxDocumentBaseClass.windowSize = self.settings.value("windowSize", QVariant(self.size())).toSize()
            frameYOffset = self.frameGeometry().height() - self.geometry().height()
            if frameYOffset > self.frameYOffset:
                NodeBoxDocumentBaseClass.frameYOffset = frameYOffset
        if self.windowMaximized is None:
            NodeBoxDocumentBaseClass.windowMaximized = self.settings.value("windowMaximized", QVariant(False)).toBool()

        self.resize(self.windowSize)
        NodeBoxDocumentBaseClass.windowPosition = self.initialPosition = self._windowPosition()
        self.move(self.initialPosition)

    def _windowPosition(self):
        controller = self.app.documentController
        if len(controller.documents) == 0:
            return self.windowPosition
        else:
            doc = controller.documents[-1]
            pos = doc.initialPosition
            xpos = pos.x() + self.frameXOffset
            ypos = pos.y() + self.frameYOffset
            gx, gy, gw, gh = self.app.desktop().availableGeometry().getRect()
            if xpos + self.width() >= gx + gw:
                xpos = gx + 10
            if ypos + self.frameGeometry().height() >= gy + gh:
                ypos = gy + 10
            return QPoint(xpos, ypos)

    def resizeEvent(self, event):
        controller = self.app.documentController
        if controller.documents and controller.documents[-1] == self:
            NodeBoxDocumentBaseClass.windowSize = self.size()

    def setupUi(self, document):
        super(NodeBoxDocumentBaseClass, self).setupUi(document)
        superView = self.graphicsSuperView
        self.BACKGROUND_COLOR = QColor(superView.palette().color(QPalette.Window).name())
        self.setupStyleSpecificUI()
        self.verticalSplitter.setStyleSheet(superView.styleSheet())
        self.graphicsView = graphicsView = self.app.newNodeBoxGraphicsView()
        superView._scene = scene = QGraphicsScene()
        scene.setItemIndexMethod(QGraphicsScene.NoIndex)
        superView.setScene(scene)
        scene.addItem(graphicsView)
        graphicsView._scene = scene
        graphicsView.superView = superView
        graphicsView._viewPort = superView.viewport()
        self._initSizeAndPosition()
        self.horizontalSplitter.setSizes(split(self.width(), 0.466))
        self.verticalSplitter.setSizes(split(self.height(), 0.767))
        self.outputView.setFont(self.textView.font())
        self.textView.setFocus()
        if self.styleName not in ("windowsxp", "windowsvista", "mac"):
            self.app.setPalette(self.app.style().standardPalette())
            self.setPalette(self.style().standardPalette())
        if sys.platform == "darwin":
            self.setWindowIcon(QIcon())

    def setupStyleSpecificUI(self):
        for widget in [self.graphicsSuperView, self.textView, self.outputView]:
            widget.setProperty("style", QVariant(self.styleName))
        self.graphicsSuperView.setStyleSheet(self.graphicsSuperView.styleSheet())
        if self.styleName in ["windows", "windowsvista", "cde", "motif", "gtk"]:
            asNeeded = Qt.ScrollBarAsNeeded
            for widget in [self.textView, self.outputView]:
                widget.setHorizontalScrollBarPolicy(asNeeded)
                widget.setVerticalScrollBarPolicy(asNeeded)
            if self.styleName == "gtk":
                if self.palette().color(QPalette.Window).valueF() < 0.5:
                    self.zoomPanel.setStyleSheet("#zoomInButton, #zoomOutButton { background-color: %s; }" % self.palette().color(QPalette.WindowText).name())
                else:
                    self.zoomPanel.setStyleSheet("")
            sz = {"windows": 27, "windowsvista": 27, "cde": 28, "motif": 29, "gtk": 28}
            self.zoomPanel.setFixedHeight(sz.get(self.styleName))
            self.zoomWidget.setFixedHeight(sz.get(self.styleName))
            self.zoomPanel.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Minimum)
            self.zoomWidget.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Minimum)
        elif self.styleName == "mac":
            self.zoomPanel.setStyleSheet("")
            self.zoomSlider.setStyleSheet("")
            self.zoomSlider.setAttribute(Qt.WA_MacMiniSize, True)

    def setupActions(self):
        prefix = 'act_'
        actions = [act for act in dir(self) if act.startswith(prefix)]
        signal = SIGNAL("triggered()")
        
        recentFileActions = self.app.documentController.recentFileActions
        self.menuOpen_Recent.insertActions(self.menuOpen_Recent.actions()[0], recentFileActions)
        self.menuOpen_Recent.setEnabled(self.app.documentController.recentFileActionsAvailable())

        def setupActions_(prefix, target):
            start = len(prefix)
            for action in [act for act in actions if act.startswith(prefix)]:
                try:
                    self.connect(getattr(self, action), signal, getattr(target, action[start:]))
                except:
                    print "Warning: No slot '%s' defined in %s." % (action[start:], target.objectName())

        def setupSpecificActions(type, fn):
            for action in [act for act in actions if act.startswith(prefix + type)]:
                self.connect(getattr(self, action), signal, fn)

        setupActions_(prefix + 'app_', self.app)
        setupActions_(prefix + 'doc_', self.app.documentController)
        setupActions_(prefix + 'main_', self)
        setupActions_(prefix + 'code_', self.textView)
        setupSpecificActions("zoom_", self.zoomToTag_)
        setupSpecificActions("find_", self.textView.performFindPanelAction)
        self.textView.addActions([getattr(self, "act_code_%s" % act) for act in "cut", "copy", "paste", "delete", "selectAll"])

        self.connect(self.textView.document(), SIGNAL("modificationChanged(bool)"), self.setDocumentModified)
        self.connect(self.textView, SIGNAL("fontChanged()"), self.fontChanged)
        self.connect(self.textView, SIGNAL("performFindPanelAction(int)"), self.app.performFindPanelAction)
        self.connect(self.app.documentController, SIGNAL("recentFileActionsAvailable(bool)"), self.menuOpen_Recent.setEnabled)
    
    def fontChanged(self):
        self.outputView.setFont(self.textView.font())
        
    def _enableWindowActions(self, enabled):
        for action in [self.act_main_showMinimized, self.act_main_toggleMaximized, self.act_main_bringAllToFront]:
            action.setEnabled(enabled)
        
    def changeEvent(self, event):
        if event.type() == QEvent.WindowStateChange:
            self._enableWindowActions(not self.isMinimized())
            controller = self.app.documentController
            if controller.documents and controller.documents[-1] == self:
                NodeBoxDocumentBaseClass.windowMaximized = self.isMaximized()
        QMainWindow.changeEvent(self, event)

    def toggleMaximized(self):
        if self.isMaximized() or self.isMinimized():
            self.showNormal()
        else:
            self.showMaximized()

    def bringAllToFront(self):
        for doc in self.app.documentController.documents:
            doc.show()
            doc.raise_()
        self.raise_()

    def closeEvent(self, event):
        self.raise_()
        self.activateWindow()
        if self.canCloseDocument():
            self.disconnect(self.app.documentController, SIGNAL("recentFileActionsAvailable(bool)"), self.menuOpen_Recent.setEnabled)
            self.closeDocument()
            event.accept()
        else:
            event.ignore()

    @classmethod
    def updateFont(cls):
        app = QApplication.instance()
        from nodebox.gui.qt.PyDETextView import PythonHighlighter, saveConfig
        PythonHighlighter.initializeFormats()
        for doc in app.documentController.documents:
            PythonHighlighter(doc.textView)
        saveConfig()

    @classmethod
    def readSettings(cls):
        pass

    @classmethod
    def writeSettings(cls):
        settings = QSettings()
        settings.setValue("windowPosition", QVariant(cls.windowPosition))
        settings.setValue("windowSize", QVariant(cls.windowSize))
        settings.setValue("windowMaximized", QVariant(cls.windowMaximized))
        
class ExportMoviePanel(QDialog, formClass("ExportMoviePanel")):
    
    MPEG4, XVID, X264, MPEG1, FLV, HUFFYUV = range(6)
    formats = { MPEG4: "MPEG4 (.avi, .mov, .mp4)",
                XVID: "XVID (.avi, .mov, .mp4)",
                X264: "X264 (.avi, .mov, mp4)",
                MPEG1: "MPEG1 (.mpg)",
                FLV: "Flash Video (.flv)",
                HUFFYUV: "Lossless Huffyuv (.avi)" }
                
    def __init__(self, *args):
        QDialog.__init__(self, *args)
        self.setupUi(self)
        self.fileNameEdit.addAction(self.actionCopy)
        self.connect(self.chooseFileButton, SIGNAL("clicked()"), self.chooseFile)
        self.allowedFileTypes = ""
        self._fileName = None

    def _get_filename(self):
        return self._fileName
    def _set_filename(self, fileName):
        self._fileName = unicode(fileName)
        self.fileNameEdit.setText(fileName)
    fileName = property(_get_filename, _set_filename)
    
    def _get_format(self):
        return self.exportMovieType.currentIndex()
    format = property(_get_format)
    
    def setupUi(self, dialog):
        super(ExportMoviePanel, self).setupUi(dialog)
        self.setWindowFlags(Qt.Sheet)
        self.setFixedSize(self.size())
        button = self.buttonBox.button(QDialogButtonBox.Save)
        button.setText("Export")
        if self.style().metaObject().className() != "QMacStyle":
            layout = self.buttonBox.parent().layout()
            layout.removeItem(layout.takeAt(1))
        formats = self.formats.items()
        formats.sort()
        self.exportMovieType.addItems([val for key, val in formats])
    
    def chooseFile(self):
        fileName = QFileDialog.getSaveFileName(self.parent(), self.tr("Export File"), self.fileName, self.tr(self.allowedFileTypes), None, QFileDialog.DontUseSheet)
        if not fileName.isEmpty():
            self.fileName = fileName

class ProgressBarControllerBaseClass(QDialog, formClass("ProgressBarController")):
    def __init__(self, *args):
        QDialog.__init__(self, *args)
        self.setupUi(self)
        self.setFixedSize(self.size())
        if sys.platform != 'darwin':
            self.helpField.setText("Press Ctrl-period (Ctrl-.) to stop.")

class NodeBoxPreferencesControllerBaseClass(QDialog, formClass("NodeBoxPreferences")):
    def __init__(self, *args):
        QDialog.__init__(self, *args)
        self.setWindowFlags(self.windowFlags() | Qt.CustomizeWindowHint)
        self.setupUi(self)
        self.setFixedSize(self.size())

class AskStringBaseClass(QDialog, formClass("AskString")):
    def __init__(self, *args):
        QDialog.__init__(self, *args)
        self.setWindowFlags(self.windowFlags() | Qt.CustomizeWindowHint)
        self.setupUi(self)
        if self.style().metaObject().className() != "QMacStyle":
            self.buttonBox.setStyleSheet("")
        self.setFixedSize(self.size())

class FindReplaceControllerBaseClass(QDialog, formClass("FindReplaceDialog")):
    def __init__(self, *args):
        QDialog.__init__(self, *args)
        self.setWindowFlags(self.windowFlags() | Qt.CustomizeWindowHint)
        self.setupUi(self)
        self.setFixedSize(self.size())
        styleName = str(self.style().metaObject().className())[1:-5].lower()
        if styleName not in ("windowsxp", "windowsvista", "mac"):
            self.setPalette(self.style().standardPalette())
        self.setWindowIcon(QIcon())

class NodeBoxApplicationBaseClass(QApplication):
    def __init__(self, *args):
        QApplication.__init__(self, *args)
        styleName = str(self.style().metaObject().className())[1:-5].lower()
        if styleName not in ("windowsxp", "windowsvista", "mac"):
            self.setPalette(self.style().standardPalette())
        self.setWindowIcon(QIcon(":/icon_app.svg"))
        self.connect(self, SIGNAL("lastWindowClosed()"), NodeBoxDocumentBaseClass.writeSettings)

    def terminate(self):
        if self.documentController.closeAllDocuments():
            NodeBoxDocumentBaseClass.writeSettings()
            self.quit()

