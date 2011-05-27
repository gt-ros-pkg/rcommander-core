from PyQt4.QtCore import Qt

from nodebox.gui.qt.baseclasses import AskStringBaseClass

class AskStringWindowController(AskStringBaseClass):
    def __init__(self, question, resultCallback, default="", parentWindow=None):
        AskStringBaseClass.__init__(self, parentWindow)
        self.question = question
        self.resultCallback = resultCallback
        self.default = default
        self.parentWindow = parentWindow
        if self.parentWindow is not None:
            self.setWindowFlags(Qt.Sheet)
        self.questionLabel.setText(self.question)
        self.textField.setFocus()
        self.exec_()
    
    def accept(self):
        value = str(self.textField.text())
        AskStringBaseClass.accept(self)
        self.resultCallback(value)
    
    def reject(self):
        AskStringBaseClass.reject(self)
        self.resultCallback(None)

def AskString(question, resultCallback, default="", parentWindow=None):
    AskStringWindowController(question, resultCallback, default, parentWindow)
        
        

