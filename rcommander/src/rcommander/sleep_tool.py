import rospy
from PyQt4.QtGui import *
from PyQt4.QtCore import *
import tool_utils as tu
import smach

##
# Adds a sleep to the state machine, making it pause execution for a specified
# number of seconds.
class SleepTool(tu.ToolBase):

    ## Constructor
    def __init__(self, rcommander):
        tu.ToolBase.__init__(self, rcommander, 'sleep', 'Sleep', SleepState)

    ## Inherited
    def fill_property_box(self, pbox):
        formlayout = pbox.layout()
        self.time_box = QDoubleSpinBox(pbox)
        self.time_box.setMinimum(0)
        self.time_box.setMaximum(1000.)
        self.time_box.setSingleStep(.2)
        self.time_box.setValue(3.)
        formlayout.addRow("&Seconds", self.time_box)

    ## Inherited
    def new_node(self, name=None):
        if name == None:
            nname = self.name + str(self.counter)
        else:
            nname = name
        return SleepState(nname, self.time_box.value())

    ## Inherited
    def set_node_properties(self, my_node):
        self.time_box.setValue(my_node.sleep_time)

    ## Inherited
    def reset(self):
        self.time_box.setValue(3.)

##
# Adds a sleep to the state machine, making it pause execution for a specified
# number of seconds.
class SleepState(tu.StateBase):

    #Constructor
    def __init__(self, name, sleep_time):
        tu.StateBase.__init__(self, name)
        self.sleep_time = sleep_time

    ## Inherited
    def get_smach_state(self):
        return SleepSmachState(self.sleep_time)

##
# Adds a sleep to the state machine, making it pause execution for a specified
# number of seconds.
class SleepSmachState(smach.State):

    #Constructor
    def __init__(self, sleep_time):
        smach.State.__init__(self, outcomes=['preempted', 'done'], input_keys=[], output_keys=[])
        self.sleep_time = sleep_time

    ## Inherited
    def execute(self, userdata):
        r = rospy.Rate(30)
        start_time = rospy.get_time()
        while (rospy.get_time() - start_time) < self.sleep_time:
            r.sleep()
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
        return 'done'



