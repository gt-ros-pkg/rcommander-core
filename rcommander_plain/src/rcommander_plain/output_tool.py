#import roslib; roslib.load_manifest('rcommander_core')
import rospy
from PyQt4.QtGui import *
from PyQt4.QtCore import *
import rcommander.tool_utils as tu
import smach

class MyOutputTool(tu.ToolBase):

    def __init__(self, rcommander):
        tu.ToolBase.__init__(self, rcommander, 'my_output', 'My Output', MyOutputState)

    def fill_property_box(self, pbox):
        formlayout = pbox.layout()
        self.output_number_box = QDoubleSpinBox(pbox)
        self.output_number_box.setMinimum(0)
        self.output_number_box.setMaximum(1000.)
        self.output_number_box.setSingleStep(.2)
        self.output_number_box.setValue(3.)
        formlayout.addRow("&Output", self.output_number_box)

    def new_node(self, name=None):
        if name == None:
            nname = self.name + str(self.counter)
        else:
            nname = name
        return MyOutputState(nname, self.output_number_box.value())

    def set_node_properties(self, my_node):
        self.output_number_box.setValue(my_node.output_number)

    def reset(self):
        self.output_number_box.setValue(3.)

class MyOutputState(tu.StateBase):

    def __init__(self, name, output_number):
        tu.StateBase.__init__(self, name, outputs={name: float})
        self.output_number = output_number

    def get_smach_state(self):
        return MyOutputSmachState(self.get_name(), self.output_number)

class MyOutputSmachState(smach.State):

    def __init__(self, output_variable_name, output_number):
        smach.State.__init__(self, outcomes=['done'], input_keys=[], output_keys=[output_variable_name])
        self.output_number = output_number
        self.output_variable_name = output_variable_name

    def execute(self, userdata):
        exec("userdata.%s = self.output_number" % self.output_variable_name)
        return 'done'


