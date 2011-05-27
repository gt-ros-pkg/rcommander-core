import os

from PyQt4.QtGui import QApplication, QMessageBox, QIcon
from PyQt4.QtCore import Qt

def centerOnDesktop(box):
    r = box.geometry()
    d = QApplication.desktop()
    box.setGeometry((d.width()-r.width())/2, (d.height()-r.height())/2, r.width(), r.height())

def errorAlert(msgText, infoText):
    icon = QIcon(":/icon_app_error.svg")
    if msgText is None:
        msgText = "An unexpected error has occurred"
    box=QMessageBox()
    box.setText(msgText)
    box.setInformativeText(infoText)
    box.setIconPixmap(icon.pixmap(60, 60))
    box.show()
    if not len(infoText) > 300:
        box.setFixedSize(420, box.height())
    centerOnDesktop(box)
    return box.exec_()
    
def chooseAction(parent, msgText, infoText, doAction, discardAction=None):
    icon = QIcon(":/icon_app.svg")
    box=QMessageBox(parent)
    if parent is not None:
        box.setWindowFlags(Qt.Sheet)
    box.setText(msgText)
    box.setInformativeText(infoText)
    buttons = QMessageBox.Cancel | QMessageBox.Yes
    if discardAction is not None:
        buttons |= QMessageBox.Discard
    box.setStandardButtons(buttons)
    button = box.button(QMessageBox.Yes)
    button.setText(doAction)
    if discardAction is not None:
        button = box.button(QMessageBox.Discard)
        button.setText(discardAction)
    box.setDefaultButton(QMessageBox.Yes)
    box.setIconPixmap(icon.pixmap(60, 60))
    box.show()
    centerOnDesktop(box)
    return box.exec_()
