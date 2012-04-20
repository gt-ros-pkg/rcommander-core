#import roslib; roslib.load_manifest('rcommander_core')
import rospy
from PyQt4.QtGui import *
from PyQt4.QtCore import *
import rcommander.tool_utils as tu
import smach

class MyInputTool(tu.ToolBase):

    def __init__(self, rcommander):
        tu.ToolBase.__init__(self, rcommander, 'node_with_input', 'My Input', InputState)

    def fill_property_box(self, pbox):
        formlayout = pbox.layout()
        self.source_box = QComboBox(pbox)
        self.source_box.addItem(' ')
        node_names = self.rcommander.outputs_of_type(float)
        for n in node_names:
            self.source_box.addItem(n)
        formlayout.addRow("&Sources", self.source_box)

    def new_node(self, name=None):
        if name == None:
            nname = self.name + str(self.counter)
        else:
            nname = name
        return InputState(nname, str(self.source_box.currentText()))

    def set_node_properties(self, my_node):
        self.source_box.setCurrentIndex(self.source_box.findText(my_node.remapping_for('myinput')))

    def reset(self):
        self.source_box.setCurrentIndex(self.source_box.findText(' '))

class InputState(tu.StateBase):

    def __init__(self, name, source_input):
        tu.StateBase.__init__(self, name)
        self.set_remapping_for('myinput', source_input) 

    def get_smach_state(self):
        return InputSmachState()

class InputSmachState(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['done'], input_keys=['myinput'], output_keys=[])

    def execute(self, userdata):
        rospy.loginfo('got ' + str(userdata.myinput))
        return 'done'


