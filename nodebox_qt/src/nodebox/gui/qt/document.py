import os
import sys

from PyQt4.QtGui import QApplication, QWidget, QMainWindow, QFileDialog, QMessageBox, QAction, QIcon
from PyQt4.QtCore import QObject, QSettings, QVariant, SIGNAL, QFileInfo, QStringList, QTimer, QPoint

from nodebox.gui.qt.util import chooseAction

class DocumentWindow(QMainWindow):
    def __init__(self, *args):
        QMainWindow.__init__(self, *args)
        self.app = QApplication.instance()
        self.settings = QSettings()
        self._fileName = None
        self._documentTitle = "Untitled"
        self._documentModified = False

    def readFromFile(self, path):
        raise NotImplementedError, "readFromFile is not implemented on this Document class."

    def writeToFile(self, path):
        raise NotImplementedError, "writeToFile is not implemented on this Document class."

    def _get_fileName(self):
        return self._fileName
    def _set_fileName(self, fileName):
        self._fileName = unicode(fileName)
        self.documentTitle = unicode(QFileInfo(fileName).fileName())
        if sys.platform == "darwin":
            self.setWindowIcon(QIcon(":/icon_file.png"))
    fileName = property(_get_fileName, _set_fileName)

    def _get_documentTitle(self):
        return self._documentTitle
    def _set_documentTitle(self, title):
        self._documentTitle = title
        self.setWindowTitle("[*]" + self._documentTitle)
    documentTitle = property(_get_documentTitle, _set_documentTitle)

    def _is_documentModified(self):
        return self._documentModified
    def setDocumentModified(self, modified):
        self._documentModified = modified
        self.setWindowModified(modified)
    documentModified = property(_is_documentModified, setDocumentModified)

    def revertDocumentToSaved(self):
        if self.fileName is not None and self._documentModified:
            if chooseAction(self, "Do you want to revert to the most recently saved version of the document \"%s\"?" %  self.documentTitle,
        "Your current changes will be lost.", "Revert") == QMessageBox.Yes:
                if self.readFromFile(self.fileName):
                    self.documentModfied = False

    def saveDocument(self):
        if self.fileName is None:
            return self.saveDocumentAs()
        elif self.writeToFile(self.fileName):
            self.emit(SIGNAL("documentTitleChanged()"))
            self._documentModified = False
            self.app.documentController.updateRecentFileActions(self.fileName)
            return True
        return False

    def saveDocumentAs(self):
        documentController = self.app.documentController
        path = self.fileName
        saveDir = path is not None and os.path.dirname(path) or documentController.lastDir
        
        fileName = QFileDialog.getSaveFileName(self, self.tr("Save File"), saveDir, self.documentFilter)
        if not fileName.isEmpty():
            if self.writeToFile(fileName):
                self.fileName = unicode(fileName)
                self._documentModified = False
                documentController.lastDir = os.path.dirname(unicode(fileName))
                documentController.updateRecentFileActions(fileName)
                return True
        documentController.lastDir = saveDir
        return False

    def canCloseDocument(self):
        if not self.documentModified:
            return True

        action = chooseAction(self, "Do you want to save the changes you made in the document \"%s\"?" % self.documentTitle, 
                             "Your changes will be lost if you don't save them.", "Save%s" % (self.fileName is None and "..." or ""), "Don't Save")
        if action == QMessageBox.Yes:
            return self.saveDocument()
        else:
            return action == QMessageBox.Discard

    def closeDocument(self):
        self.app.documentController.remove(self)

    def hideAndShow(self):
        self.hide()
        QTimer.singleShot(100, self.show)

    def show_(self):
        self.show()
        self.raise_()
        self.activateWindow()


class DocumentController(QObject):
    MaxRecentFiles = 10
    recentFileActions = []
    documents = []

    def __init__(self, parent, klass, documentFilter):
        QObject.__init__(self, parent)
        self.app = QApplication.instance()
        self.settings = QSettings()
        home = os.getenv("USERPROFILE") or os.getenv("HOME")
        self._lastDir = self.settings.value("lastDir", QVariant(home)).toString()
        self.documentClass = klass
        self.currentDocument = None
        self.documentFilter = documentFilter
        self._loadRecentFileActions()
        self.connect(self.app, SIGNAL("focusChanged(QWidget *, QWidget *)"), self.focusChanged)

    def remove(self, document):
        try:
            self.documents.remove(document)
        except:
            pass
        if not self.documents:
            self.currentDocument = None

    def focusChanged(self, oldWidget, newWidget):
        try:
            if isinstance(newWidget, self.documentClass) and self.currentDocument != newWidget:
                self.currentDocument = newWidget
                self.emit(SIGNAL("currentDocumentChanged(QMainWindow *)"), self.currentDocument)
            else:
                widget = newWidget
                while True:
                    widget = widget.parent()
                    if widget.isWindow():
                        break
                if isinstance(widget, self.documentClass) and self.currentDocument != widget:
                    self.currentDocument = widget
                    self.emit(SIGNAL("currentDocumentChanged(QMainWindow *)"), self.currentDocument)
        except:
            pass
        
    def _loadRecentFileActions(self):
        for i in range(self.MaxRecentFiles):
            action = QAction(self)
            action.setVisible(False)
            self.connect(action, SIGNAL("triggered()"), self.openRecent)
            self.recentFileActions.append(action)
        self._updateRecentFileActions()

    def updateRecentFileActions(self, fileName):
        files = self.settings.value("recentFileList").toStringList()
        files.removeAll(fileName)
        files.prepend(fileName)

        while files.count() > self.MaxRecentFiles:
            files.removeAt(files.count() - 1)
        
        self.settings.setValue("recentFileList", QVariant(files))
        self._updateRecentFileActions()
        
    def _updateRecentFileActions(self):
        files = list(self.settings.value("recentFileList").toStringList())
        fileNames = {}
        numRecentFiles = min(len(files), self.MaxRecentFiles)

        for action in self.recentFileActions:
            action.setVisible(False)
            
        for i in range(numRecentFiles):
            fileName = QFileInfo(files[i]).fileName()
            if fileName in fileNames.keys():
                fileNames[fileName] += 1
            else:
                fileNames[fileName] = 1

        for i in range(numRecentFiles):
            fileName = QFileInfo(files[i]).fileName()
            filePath = QVariant(files[i])
            action = self.recentFileActions[i]
            action.setText(fileNames[fileName] == 1 and fileName or filePath.toString())
            action.setData(filePath)
            action.setVisible(True)
        self.emit(SIGNAL("recentFileActionsAvailable(bool)"), numRecentFiles > 0)

    def recentFileActionsAvailable(self):
        return any(action.isVisible() for action in self.recentFileActions)
        
    def _get_lastDir(self):
        return self._lastDir
    def _set_lastDir(self, lastDir):
        self._lastDir = lastDir
        self.settings.setValue("lastDir", QVariant(lastDir))
    lastDir = property(_get_lastDir, _set_lastDir)

    def _newUntitledDocumentTitle(self):
        t = "Untitled"
        li = []
        for title in [doc.documentTitle for doc in self.documents]:
            if title == t:
                li.append(1)
            elif title.startswith(t + " "):
                parts = title.split()
                if len(parts) == 2:
                    try:
                        num = int(parts[1])
                        li.append(num)
                    except ValueError:
                        pass
        if len(li) == 0:
            return t
        else:
            return "%s %i" % (t, (max(li) + 1))
        
    def newDocument(self):
        doc = self.createNewDocument()
        doc.documentTitle = self._newUntitledDocumentTitle()
        doc.documentFilter = self.documentFilter
        doc.show_()
        self.documents.append(doc)
        return doc

    def createNewDocument(self):
        klass = self.documentClass
        return klass()
        
    def openDocument(self, fileName=None):
        if fileName is None:
            path = self.sender().parent().fileName
            openDir = path is not None and os.path.dirname(path) or self.lastDir
            fileName = QFileDialog.getOpenFileName(None, self.tr("Open File"), openDir, self.documentFilter)
            openRecent = False
        else:
            openDir = os.path.dirname(unicode(fileName))
            openRecent = True

        openFiles = [doc.fileName for doc in self.documents]
        if fileName in openFiles:
            doc = self.documents[openFiles.index(fileName)]
            doc.show_()
            return doc
        
        if not fileName.isEmpty():
            doc = self.createNewDocument()
            if doc.readFromFile(fileName):
                doc.fileName = unicode(fileName)
                self.documents.append(doc)
                self.lastDir = os.path.dirname(unicode(fileName))
                self.updateRecentFileActions(fileName)
                doc.show_()
                if not openRecent and 'linux' in sys.platform:
                     QTimer.singleShot(100, doc.hideAndShow)
            return doc

        self.lastDir = openDir
        return None

    def openRecent(self):
        action = self.sender()
        if action:
            self.openDocument(action.data().toString())
    
    def clearRecentDocuments(self):
        self.settings.setValue("recentFileList", QVariant(QStringList()))
        self._updateRecentFileActions()
        
    def modifiedDocuments(self):
        return [doc for doc in self.documents if doc.documentModified]
    
    def hasModifiedDocuments(self):
        return len(self.modifiedDocuments()) > 0

    def closeAllDocuments(self):
        modifiedCount = len(self.modifiedDocuments())

        if modifiedCount == 0:
            return True

        docs = list(self.documents)
        docs.reverse()
        
        action = None
        if modifiedCount > 1:
            action = chooseAction(None, "You have %i %s documents with unsaved changes. Do you want to review these changes before quitting?" % (modifiedCount, self.app.applicationName()), 
                                  "If you don't review your documents, all your changes will be lost.", "Review Changes...", "Discard Changes")
            
        if modifiedCount == 1 or action == QMessageBox.Yes:            
            for doc in docs:
                if not doc.close():
                    return False
            return True
        else:
            return action == QMessageBox.Discard
