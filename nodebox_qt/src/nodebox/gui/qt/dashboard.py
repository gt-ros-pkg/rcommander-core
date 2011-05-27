from PyQt4.QtGui import QDialog, QGridLayout, QLayout, QFont, QLabel, QSlider, QLineEdit, QCheckBox, QPushButton
from PyQt4.QtCore import Qt, SIGNAL
from nodebox import graphics

class DashboardController(QDialog):
    def __init__(self, parent=None):
        QDialog.__init__(self, parent)
        self.setWindowFlags(self.windowFlags() | Qt.Tool | Qt.CustomizeWindowHint)
        self.document = parent
        self.documentWindow = parent.graphicsView
        self.layout = QGridLayout()
        self.layout.setSizeConstraint(QLayout.SetFixedSize)
        self.layout.setVerticalSpacing(8)
        self.setLayout(self.layout)
        self.styleName = str(self.style().metaObject().className())[1:-5].lower()
        self.qvars = []
    
    def clearInterface(self):
        while self.layout.count() > 0:
            widget = self.layout.itemAt(0).widget()
            self.layout.removeWidget(widget)
            widget.setParent(None)
            
    def numberChanged_(self, value):
        sender = self.sender()
        self.document.vars[sender.tag].value = sender.value() / 160.
        self.document._runScript(compile=False, newSeed=False)

    def textChanged_(self):
        sender = self.sender()
        self.document.vars[sender.tag].value = sender.text()
        self.document._runScript(compile=False, newSeed=False)

    def booleanChanged_(self):
        sender = self.sender()
        self.document.vars[sender.tag].value = not self.document.vars[sender.tag].value
        self.document._runScript(compile=False, newSeed=False)

    def buttonClicked_(self):
        sender = self.sender()
        var = self.document.vars[sender.tag]
        self.document.fastRun(self.document.namespace[var.name], newSeed=True)

    def buildInterface_(self, vars):
        self.vars = vars
        self.clearInterface()
        if len(self.vars) > 0:
            self.show()
        else:
            self.hide()
        self.setWindowTitle(self.document.windowTitle())
        cnt = 0
        for v in self.vars:
            if v.type == graphics.NUMBER:
                self._addLabel(v, cnt)
                self._addSlider(v, cnt)
            elif v.type == graphics.TEXT:
                self._addLabel(v, cnt)
                self._addTextField(v, cnt)
            elif v.type == graphics.BOOLEAN:
                self._addSwitch(v, cnt)
            elif v.type == graphics.BUTTON:
                self._addButton(v, cnt)
            cnt += 1
            
    def _addLabel(self, v, cnt):
        label = QLabel(v.name + ':')
        f = QFont(label.font().family())
        f.setPointSizeF(11)
        label.setFont(f)
        label.setMinimumWidth(80)
        label.setAlignment(Qt.AlignRight)
        self.layout.addWidget(label, cnt, 0)
    
    def _addSlider(self, v, cnt):
        control = QSlider(Qt.Horizontal)
        control.setMinimumWidth(172)
        control.setFocusPolicy(Qt.StrongFocus)
        control.setMaximum(v.max * 160)
        control.setMinimum(v.min * 160)
        control.setValue(v.value * 160)
        control.tag = cnt
        if self.styleName == "mac":
            control.setAttribute(Qt.WA_MacMiniSize, True)
        self.layout.addWidget(control, cnt, 1)
        self.connect(control, SIGNAL("valueChanged(int)"), self.numberChanged_)

    def _addTextField(self, v, cnt):
        control = QLineEdit()
        control.setMinimumWidth(172)
        control.setMaximumHeight(17)
        control.setFocusPolicy(Qt.StrongFocus)
        f = control.font()
        f.setPointSizeF(9)
        control.setFont(f)
        control.setText(v.value)
        control.tag = cnt
        self.layout.addWidget(control, cnt, 1)
        self.connect(control, SIGNAL("returnPressed()"), self.textChanged_)
        
    def _addSwitch(self, v, cnt):
        control = QCheckBox(v.name)
        control.setChecked(v.value)
        control.setFocusPolicy(Qt.StrongFocus)
        f = control.font()
        f.setPointSizeF(11)
        control.setFont(f)
        control.tag = cnt
        if self.styleName == "mac":
            control.setAttribute(Qt.WA_MacSmallSize, True)
        self.layout.addWidget(control, cnt, 1)
        self.connect(control, SIGNAL("stateChanged(int)"), self.booleanChanged_)

    def _addButton(self, v, cnt):
        control = QPushButton(v.name)
        control.setFocusPolicy(Qt.StrongFocus)
        f = control.font()
        f.setPointSizeF(11)
        if self.styleName == "mac":
            f.setPointSizeF(10)
        control.setFont(f)
        control.tag = cnt
        if self.styleName == "mac":
            control.setAttribute(Qt.WA_MacMiniSize, True)
        self.layout.addWidget(control, cnt, 1)
        self.connect(control, SIGNAL("clicked()"), self.buttonClicked_)
