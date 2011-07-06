import tool_utils as tu
from PyQt4.QtGui import *
from PyQt4.QtCore import *
import numpy as np

class Point3DTool(tu.ToolBase):

    def __init__(self, rcommander):
        tu.ToolBase.__init__(self, rcommander, 'point3d', 'Point 3D')
        self.default_frame = 'base_link'

    def fill_property_box(self, pbox):
        formlayout = pbox.layout()

        self.xline = QLineEdit(pbox)
        self.yline = QLineEdit(pbox)
        self.zline = QLineEdit(pbox)
        self.frameline = QLineEdit(pbox)
        self.pose_button = QPushButton(pbox)
        self.pose_button.setText('Get Pose')
        self.pose_button.setEnabled(False)

        formlayout.addRow("&x", self.xline)
        formlayout.addRow("&y", self.yline)
        formlayout.addRow("&z", self.zline)
        formlayout.addRow('&frame', self.frameline)
        formlayout.addRow(self.pose_button)
        self.reset()
        pbox.update()

    def _create_node(self, name=None):
        point = np.matrix([float(self.xline.text()), float(self.yline.text()), float(self.zline.text())]).T
        frame = str(self.frameline.text())
        if name == None:
            nname = self.name + str(self.counter)
        else:
            nname = name
        return Point3DState(nname, point, frame)
    
    def _node_selected(self, node):
        self.xline.setText(str(node.point[0,0]))
        self.yline.setText(str(node.point[1,0]))
        self.zline.setText(str(node.point[1,0]))
        self.frameline.setText(node.frame)

    def reset(self):
        self.xline.setText(str(0.))
        self.yline.setText(str(0.))
        self.zline.setText(str(0.))
        self.frameline.setText(self.default_frame)


class Point3DState(tu.InfoStateBase):
    
    def __init__(self, name, point, frame):
        tu.InfoStateBase.__init__(self, name)
        self.point = point
        self.frame = frame

    def set_info(self, info):
        self.point, self.frame = info

    def set_info(self):
        return [self.point, self.frame]

    def __getstate__(self):
        state = tu.InfoStateBase.__getstate__(self)
        my_state = [self.point, self.frame]
        return {'simple_state': state, 'self': my_state}


