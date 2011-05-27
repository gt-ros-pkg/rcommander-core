import roslib; roslib.load_manifest('rcommander')
import rospy
import sys

from PyQt4 import QtGui
from PyQt4.QtGui import QGraphicsScene
from rcommander_auto import Ui_RCommanderWindow
from nodebox.gui.qt import NodeBoxGraphicsView 


class RCommander(QtGui.QMainWindow):
    def __init__(self):
        QtGui.QMainWindow.__init__(self)
        self.ui = Ui_RCommanderWindow()
        self.ui.setupUi(self)

        superView = self.ui.graphicsView
        superView._scene = scene = QGraphicsScene()
        scene.setItemIndexMethod(QGraphicsScene.NoIndex)
        superView.setScene(scene)

        self.graphicsView = graphicsView = NodeBoxGraphicsView()
        scene.addItem(graphicsView)
        graphicsView._scene = scene
        graphicsView.superView = superView
        graphicsView._viewPort = superView.viewport()

app = QtGui.QApplication(sys.argv)
rc = RCommander()
rc.show()
sys.exit(app.exec_())

