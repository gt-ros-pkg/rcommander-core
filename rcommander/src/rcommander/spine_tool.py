import roslib; roslib.load_manifest('rcommander')
import rospy
from PyQt4.QtGui import *
from PyQt4.QtCore import *
import tool_utils as tu
import smach
import pr2_controllers_msgs.msg as pm


class SpineTool(tu.ToolBase):

    def __init__(self, rcommander):
        tu.ToolBase.__init__(self, rcommander, 'move_spine', 'Spine')

    def fill_property_box(self, pbox):
        formlayout = pbox.layout()
        self.spine_box = tu.SliderBox(pbox, 15., 29.5, 1., .05, 'spine', units='cm')
        formlayout.addRow('&Height', self.spine_box.container)
        pbox.update()

    def _create_node(self, name=None):
        if name == None:
            nname = self.name + str(self.counter)
        else:
            nname = name
        return SpineState(nname, self.spine_box.value()/100.)

    def _node_selected(self, my_node):
        self.spine_box.set_value(my_node.position*100.)

    def reset(self):
        self.spine_box.set_value(15.)

class SpineState(tu.SimpleStateBase): 

    def __init__(self, name, position):
        tu.SimpleStateBase.__init__(self, name, \
                'torso_controller/position_joint_action', 
                pm.SingleJointPositionAction, 
                goal_cb_str = 'ros_goal')
        self.position = position

    def ros_goal(self, userdata, default_goal):
        return pm.SingleJointPositionGoal(position = self.position)

    def __getstate__(self):
        state = tu.SimpleStateBase.__getstate__(self)
        my_state = [self.position]
        return {'simple_state': state, 'self': my_state}

    def __setstate__(self, state):
        tu.SimpleStateBase.__setstate__(self, state['simple_state'])
        self.position = state['self'][0]


