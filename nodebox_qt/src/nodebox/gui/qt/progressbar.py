from PyQt4.QtGui import QApplication
from PyQt4.QtCore import Qt, SIGNAL

from nodebox.gui.qt.baseclasses import ProgressBarControllerBaseClass

class ProgressBarController(ProgressBarControllerBaseClass):
    def __init__(self, *args):
        ProgressBarControllerBaseClass.__init__(self, *args)
        self.setWindowFlags(Qt.Sheet)
    
    def keyPressEvent(self, event):
        if int(QApplication.keyboardModifiers()) & Qt.ControlModifier > 0:
            if event.key() == Qt.Key_Period:
                self.emit(SIGNAL("progressAborted()"))
        elif event.key() == Qt.Key_Escape:
            self.emit(SIGNAL("progressAborted()"))
        event.ignore()
    
    def begin(self, message, maxval):
        self.value = 0
        self.message = message
        self.maxval = maxval
        self.progressBar.setMaximum(self.maxval)
        self.messageField.setText(self.message)
        self.show()
    
    def inc(self):
        self.value += 1
        self.progressBar.setValue(self.value)
    
    def end(self):
        self.close()