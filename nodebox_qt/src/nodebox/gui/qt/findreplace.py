from PyQt4.QtGui import QApplication, QTextEdit, QTextDocument, QTextCursor
from PyQt4.QtCore import Qt, SIGNAL, QSettings, QVariant, QRegExp
from nodebox.gui.qt.baseclasses import FindReplaceControllerBaseClass

class FindReplaceController(FindReplaceControllerBaseClass):
    
    SHOW_PANEL, FIND_NEXT, FIND_PREVIOUS, USE_SELECTION_FOR_FIND = range(4)
    CONTAINS, STARTS_WITH, FULL_WORD = range(3)
    
    def __init__(self, *args):
        FindReplaceControllerBaseClass.__init__(self, *args)
        self.textEdit = None
        self.readSettings()
        self.actionsDict = { self.FIND_NEXT: self.findNext,
                             self.FIND_PREVIOUS: self.findPrevious,
                             self.USE_SELECTION_FOR_FIND: self.selectionForFind }
        self.connect(self.nextButton, SIGNAL("clicked()"), self.findNext)
        self.connect(self.previousButton, SIGNAL("clicked()"), self.findPrevious)
        self.connect(self.replaceButton, SIGNAL("clicked()"), self.replace)
        self.connect(self.replaceFindButton, SIGNAL("clicked()"), self.replaceAndFind)
        self.connect(self.replaceAllButton, SIGNAL("clicked()"), self.replaceAll)

    def readSettings(self):
        settings = QSettings()
        self.findComboBox.setEditText(settings.value("findText", QVariant("")).toString())
        self.ignoreCaseCheckBox.setChecked(settings.value("ignoreCase", QVariant("True")).toBool())   
        self.wrapAroundCheckBox.setChecked(settings.value("wrapAround", QVariant("True")).toBool())   
        self.flagsComboBox.setCurrentIndex(settings.value("findFlags", QVariant(0)).toInt()[0])
            
    def writeSettings(self):
        settings = QSettings()
        settings.setValue("findText", QVariant(self.findComboBox.currentText()))
        settings.setValue("ignoreCase", QVariant(self.ignoreCaseCheckBox.isChecked()))
        settings.setValue("wrapAround", QVariant(self.wrapAroundCheckBox.isChecked()))
        settings.setValue("findFlags", QVariant(self.flagsComboBox.currentIndex()))
        
    def get_textEdit(self):
        return self._textEdit
    def set_textEdit(self, textEdit=None):
        self._textEdit = textEdit
        self.enableButtons(textEdit is not None)
    textEdit = property(get_textEdit, set_textEdit)
    
    def clearFeedback(self):
        self.feedbackLabel.clear()
    
    def enableButtons(self, enabled):
        for el in ["replaceAll", "replace", "replaceFind", "previous", "next"]:
            getattr(self, "%sButton" % el).setEnabled(enabled)
        
    def performAction(self, action, target):
        if not isinstance(target, QTextEdit):
            return
        
        cmd = self.actionsDict.get(action)
        if cmd is not None:
            cmd(target)
    
    def findNext(self, textEdit=None):
        self._find(textEdit)

    def findPrevious(self, textEdit=None):
        self._find(textEdit, backward=True)

    def _textAndFindFlags(self, backward=False):
        text = self.findComboBox.currentText()
        flags = QTextDocument.FindFlags()
        if backward:
            flags |= QTextDocument.FindBackward
        if not self.ignoreCaseCheckBox.isChecked():
            flags |= QTextDocument.FindCaseSensitively
        if self.flagsComboBox.currentIndex() == self.FULL_WORD:
            flags |= QTextDocument.FindWholeWords
        elif self.flagsComboBox.currentIndex() == self.STARTS_WITH:
            text = QRegExp("\\b%s" % text)
            caseSensitive = not self.ignoreCaseCheckBox.isChecked() and Qt.CaseSensitive or Qt.CaseInsensitive
            text.setCaseSensitivity(caseSensitive)
        return text, flags

    def _find(self, textEdit=None, backward=False):
        if textEdit is None:
            textEdit = self.textEdit

        found = False

        if textEdit is not None:
            cursor = textEdit.textCursor()
            text, flags = self._textAndFindFlags(backward=backward)
            position = cursor.position()
            cursor = textEdit.document().find(text, cursor, flags)

            if not cursor.isNull():
                textEdit.setTextCursor(cursor)
                found = True
            elif self.wrapAroundCheckBox.isChecked():
                cursor = QTextCursor(textEdit.textCursor())
                cursor.movePosition(backward and QTextCursor.End or QTextCursor.Start)
                cursor = textEdit.document().find(text, cursor, flags)
                if not cursor.isNull():
                    if position == cursor.position():
                        pass #todo
                    textEdit.setTextCursor(cursor)
                    found = True
            self.writeSettings()

        if not found:
            QApplication.beep()
            self.feedbackLabel.setText(self.tr("Not found"))
        else:
            self.clearFeedback()
    
    def replaceAll(self):
        textEdit = self.textEdit
        
        count = 0
        
        if textEdit is not None and len(self.findComboBox.currentText()) > 0:
            findText = self.findComboBox.currentText()
            replaceText = self.replaceComboBox.currentText()
            caseSensitive = not self.ignoreCaseCheckBox.isChecked() and Qt.CaseSensitive or Qt.CaseInsensitive
            flagsIndex = self.flagsComboBox.currentIndex()
            if flagsIndex == self.STARTS_WITH:
                pattern = "\\b%s"
            elif flagsIndex == self.FULL_WORD:
                pattern = "\\b%s\\b"
            else:
                pattern = "%s"
            rx = QRegExp(pattern % findText)
            rx.setCaseSensitivity(caseSensitive)

            current = textEdit.toPlainText()
            pos = 0
            while pos >= 0:
                pos = rx.indexIn(current, pos)
                if pos >= 0:
                    pos += 1
                    count += 1 
            replaced = current.replace(rx, replaceText)

            textEdit.selectAll()
            cursor = textEdit.textCursor()
            cursor.beginEditBlock()
            cursor.insertText(replaced)
            cursor.endEditBlock()

        if textEdit is not None:
            if not count:
                QApplication.beep()
                self.feedbackLabel.setText(self.tr("Not found"))
            else:
                self.feedbackLabel.setText(self.tr("%s replaced" % count))
                
            self.writeSettings()
                
    def replace(self):
        textEdit = self.textEdit

        if textEdit is not None:
            cursor = textEdit.textCursor()
            cursor.insertText(self.replaceComboBox.currentText())
            self.writeSettings()
        self.clearFeedback()

    def replaceAndFind(self):
        self.replace()
        self.findNext()

    def selectionForFind(self, textEdit=None):
        if textEdit is None:
            textEdit = self.textEdit
        if textEdit is not None:
            self.findComboBox.setEditText(textEdit.textCursor().selectedText())
            self.writeSettings()
            