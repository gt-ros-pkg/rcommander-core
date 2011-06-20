import tool_utils as tu
from PyQt4.QtGui import *
from PyQt4.QtCore import *
import time
from pr2_common_action_msgs.msg as ca 

class TuckTool(tu.ToolBase):

    def __init__(self, rcommander):
        tu.ToolBase.__init__(self, rcommander, 'tuck', 'Tuck')

    def fill_property_box(self, pbox):
        formlayout = pbox.layout()
        self.speedline = QLineEdit(pbox)

        self.tuck_left = QComboBox(pbox)
        self.tuck_left.addItem('True')
        self.tuck_left.addItem('False')

        self.tuck_right = QComboBox(pbox)
        self.tuck_right.addItem('True')
        self.tuck_right.addItem('False')

        formlayout.addRow("&Left Arm", self.tuck_left)
        formlayout.addRow("&Right Arm", self.tuck_right)
        pbox.update()


    def _create_node(self, name=None):
        left = ('True' == str(self.tuck_left.text()))
        right = ('True' == str(self.tuck_right.text()))
        if name == None:
            nname = self.name + str(self.counter)
        else:
            nname = name
        return TuckState(nname, left, right)
    
    def _node_selected(self, node):
        self.tuck_left.setCurrentIndex(self.tuck_left.findText(str(node.tuck_left)))
        self.tuck_right.setCurrentIndex(self.tuck_right.findText(str(node.tuck_right)))

    def reset(self):
        self.tuck_left.setCurrentIndex(self.tuck_left.findText('True'))
        self.tuck_right.setCurrentIndex(self.tuck_right.findText('True'))


class TuckState(tu.SimpleStateBase): # smach_ros.SimpleActionState):
    def __init__(self, name, tuck_left, tuck_right):
        tu.SimpleStateBase.__init__(self, name, \
                'tuck_arm', ca.TuckArmsAction, 
                goal_cb = self.ros_goal) 

        self.tuck_left = tuck_left
        self.tuck_right = tuck_right

    def ros_goal(self):
        goal = ca.TuckArmsGoal()
        goal.tuck_left = self.tuck_left
        goal.tuck_right = self.tuck_right

    def __getstate__(self):
        state = tu.SimpleStateBase.__getstate__(self)
        my_state = [self.tuck_left, self.tuck_right]
        return {'simple_state': state, 'self': my_state}

    def __setstate__(self, state):
        tu.SimpleStateBase.__setstate__(self, state['simple_state'])
        self.tuck_left, self.tuck_right = state['self']
