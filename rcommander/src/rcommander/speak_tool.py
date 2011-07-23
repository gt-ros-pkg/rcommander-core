import roslib; roslib.load_manifest('rcommander')
import rospy
from PyQt4.QtGui import *
from PyQt4.QtCore import *
import tool_utils as tu
from sound_play.libsoundplay import SoundClient


class SpeakTool(tu.ToolBase):

    DEFAULT_TEXT = 'hello world'

    def __init__(self, rcommander):
        tu.ToolBase.__init__(self, rcommander, 'speak', 'Speak')

    def fill_property_box(self, pbox):
        formlayout = pbox.layout()
        self.text = QLineEdit(pbox)
        self.text.setText(SpeakTool.DEFAULT_TEXT)

    def _create_node(self, name=None):
        if name == None:
            nname = self.name + str(self.counter)
        else:
            nname = name
        return SpeakNode(nname, str(self.text.text()))

    def _node_selected(self, my_node):
        self.text.setText(my_node.text)

    def reset(self):
        self.text.setText(SpeakTool.DEFAULT_TEXT)


class SpeakNode(smach.State, tu.StateBase): 

    def __init__(self, name, text):
        self.name = name
        self.text = text
        self.__init_unpicklables__()

    def execute(self, userdata):
        self.sound.say(self.text)
        return 'done'

    def __init_unpicklables__(self):
        tu.StateBase.__init__(self, self.name)
        smach.State.__init__(self, outcomes = ['done'], input_keys = [], output_keys = [])
        self.sound = SoundClient()

    def __getstate__(self):
        state = tu.StateBase.__getstate__(self)
        my_state = [self.name, self.text]
        return {'state_base': state, 'self': my_state}

    def __setstate__(self, state):
        tu.StateBase.__setstate__(self, state['state_base'])
        self.name, self.text = state['self']
        self.__init_unpicklables__()



