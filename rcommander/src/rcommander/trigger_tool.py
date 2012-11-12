import tool_utils as tu
import PyQt4.QtGui as qtg
import PyQt4.QtCore as qtc
from PyQt4.QtGui import *
from PyQt4.QtCore import *
import smach
import rospy
from msg import Trigger


## Topic to send trigger to
TRIGGER_TOPIC = 'trigger'

## Creates a state that sleeps but can be interrupted if it receives a message
# on the topic TRIGGER_TOPIC
class TriggerTool(tu.ToolBase):

    ## Constructor
    def __init__(self, rcommander):
        tu.ToolBase.__init__(self, rcommander, 'wait_trigger', 'Trigger', TriggerState)
        self.trigger_pub = rospy.Publisher(TRIGGER_TOPIC, Trigger)

    ## Inherited
    def fill_property_box(self, pbox):
        formlayout = pbox.layout()

        self.time_out_box = qtg.QDoubleSpinBox(pbox)
        self.time_out_box.setMinimum(1.)
        self.time_out_box.setMaximum(1000.)
        self.time_out_box.setSingleStep(1.)

        self.trigger_button = qtg.QPushButton(pbox)
        self.trigger_button.setText('Send Trigger')
        self.rcommander.connect(self.trigger_button, qtc.SIGNAL('clicked()'), self.send_trigger_cb)

        formlayout.addRow('&Time out', self.time_out_box)
        formlayout.addRow(self.trigger_button)

        self.reset()
        pbox.update()

    ## Callback to send a message on the trigger topic.
    def send_trigger_cb(self):
        self.trigger_pub.publish(Trigger('a trigger!'))
        rospy.sleep(.5)
        self.trigger_pub.publish(Trigger('a trigger!'))

    ## Inherited
    def new_node(self, name=None):
        if name == None:
            nname = self.name + str(self.counter)
        else:
            nname = name

        return TriggerState(nname, self.time_out_box.value())

    ## Inherited
    def set_node_properties(self, node):
        self.time_out_box.setValue(node.time_out)

    ## Inherited
    def reset(self):
        self.time_out_box.setValue(10.)


class TriggerState(tu.StateBase):

    ## Constructor
    def __init__(self, name, time_out):
        tu.StateBase.__init__(self, name)
        self.time_out = time_out

    ## Inherited
    def get_smach_state(self):
        return TriggerStateSmach(time_out = self.time_out)

class TriggerStateSmach(smach.State):

    ## Constructor
    #@param message if self.message is None we will wait for a message, else use provided message
    #@param time_out if we have to wait for a message specify how long to wait before timing out
    def __init__(self, time_out):
        smach.State.__init__(self, 
                outcomes = ['succeeded', 'preempted', 'timed_out'], 
                input_keys = [], output_keys = [])
        self.time_out = time_out

    ## Inherited
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


