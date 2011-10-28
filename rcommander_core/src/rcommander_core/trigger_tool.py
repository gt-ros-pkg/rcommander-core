import tool_utils as tu
from PyQt4.QtGui import *
from PyQt4.QtCore import *
import smach
import rospy
from rcommander_core.msg import Trigger

TRIGGER_TOPIC = 'trigger'

class TriggerTool(tu.ToolBase):

    def __init__(self, rcommander):
        tu.ToolBase.__init__(self, rcommander, 'wait_trigger', 'Trigger', TriggerState)
        self.trigger_pub = rospy.Publisher(TRIGGER_TOPIC, Trigger)

    def fill_property_box(self, pbox):
        formlayout = pbox.layout()

        self.time_out_box = QDoubleSpinBox(pbox)
        self.time_out_box.setMinimum(1.)
        self.time_out_box.setMaximum(1000.)
        self.time_out_box.setSingleStep(1.)

        self.trigger_button = QPushButton(pbox)
        self.trigger_button.setText('Send Trigger')
        self.rcommander.connect(self.trigger_button, SIGNAL('clicked()'), self.send_trigger_cb)

        formlayout.addRow('&Time out', self.time_out_box)
        formlayout.addRow(self.trigger_button)

        self.reset()
        pbox.update()

    def send_trigger_cb(self):
        self.trigger_pub.publish(Trigger('a trigger!'))
        rospy.sleep(.5)
        self.trigger_pub.publish(Trigger('a trigger!'))

    def new_node(self, name=None):
        if name == None:
            nname = self.name + str(self.counter)
        else:
            nname = name

        return TriggerState(nname, self.time_out_box.value())

    def set_node_properties(self, node):
        self.time_out_box.setValue(node.time_out)

    def reset(self):
        self.time_out_box.setValue(10.)


class TriggerState(tu.StateBase):

    def __init__(self, name, time_out):
        tu.StateBase.__init__(self, name)
        self.time_out = time_out

    def get_smach_state(self):
        return TriggerStateSmach(time_out = self.time_out)

class TriggerStateSmach(smach.State):
    ##
    #@param message if self.message is None we will wait for a message, else use provided message
    #@param time_out if we have to wait for a message specify how long to wait before timing out
    def __init__(self, time_out):
        smach.State.__init__(self, 
                outcomes = ['succeeded', 'preempted', 'timed_out'], 
                input_keys = [], output_keys = [])
        self.time_out = time_out

    def execute(self, userdata):
        msg = None
        t_start = rospy.get_time()

        while msg == None:
            try:
                msg = rospy.wait_for_message(TRIGGER_TOPIC, Trigger, .1)
            except rospy.ROSException, e:
                pass

            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'

            if (self.time_out != None) and ((rospy.get_time() - t_start) > self.time_out):
                return 'timed_out'

        return 'succeeded'


