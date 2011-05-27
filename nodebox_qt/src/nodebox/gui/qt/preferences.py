import sys

from PyQt4.QtGui import QColor, QIcon, QFont, QFontInfo, QFontDialog
from PyQt4.QtCore import SIGNAL

from nodebox.gui.qt.baseclasses import NodeBoxPreferencesControllerBaseClass
from nodebox.gui.qt.PyDETextView import Config
from nodebox.gui.qt.widgets.colorbutton import ColorButton

class NodeBoxPreferencesController(NodeBoxPreferencesControllerBaseClass):
    def __init__(self, *args):
        NodeBoxPreferencesControllerBaseClass.__init__(self, *args)
        self.setWindowIcon(QIcon())

        buttons = [getattr(self, el) for el in dir(self) if isinstance(getattr(self, el), ColorButton)]
        for button in buttons:
            tag = button.property("tag").toString()
            button.setColor(QColor(Config["%sfontcolor"  % tag]))
            self.connect(button, SIGNAL("colorChanged()"), self.colorChanged)
        
        font = QFont(Config["font"])
        self.setFont(font)
        self.connect(self.fontButton, SIGNAL("clicked()"), self.chooseFont)

    def setFont(self, font):
        if sys.platform == "darwin":
            if QFontInfo(font).fixedPitch() and font.pointSize() <= 10:
                font.setStyleStrategy(QFont.NoAntialias)
        bolditalic = ""
        bolditalic += font.bold() and "Bold " or ""
        bolditalic += font.italic() and "Italic " or ""
        fullName = "%s %s%s" % (font.family(), bolditalic, font.pointSize())
        self._font = font
        self.fontPreview.setFont(font)
        self.fontPreview.setText(fullName)
        
    def colorChanged(self):
        sender = self.sender()
        if sender is not None:
            Config["%sfontcolor" % sender.property("tag").toString()] = sender.color().name()
            self.emit(SIGNAL("textFontChanged()"))
    
    def chooseFont(self):
        font, ok = QFontDialog.getFont(self._font, self)
        if ok:
            self.setFont(font)
            Config["font"] = font
            self.emit(SIGNAL("textFontChanged()"))
