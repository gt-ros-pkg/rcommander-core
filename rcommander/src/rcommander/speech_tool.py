import roslib; roslib.load_manifest('rcommander')
import rospy
from PyQt4.QtGui import *
from PyQt4.QtCore import *
import tool_utils as tu


class SpeechTool(tu.ToolBase):

    def __init__(self, rcommander):
        tu.ToolBase.__init__(self, rcommander, 'speech', 'Speak')
        pass

    def fill_property_box(self, pbox):
        pass

    def _create_node(self, name=None):
        if name == None:
            nname = self.name + str(self.counter)
        else:
            nname = name
        pass

    def _node_selected(self, my_node):
        pass

    def reset(self):
        pass


class MyState(smach.State, tu.StateBase): 

    def __init__(self, name, sleep_time):
        self.name = name
        self.__init_unpicklables__()
        pass

    def execute(self, userdata):
        pass

    def __init_unpicklables__(self):
        tu.StateBase.__init__(self, self.name)
        smach.State.__init__(self, outcomes = ['done'], input_keys = [], output_keys = [])

    def __getstate__(self):
        state = tu.StateBase.__getstate__(self)
        my_state = [self.name] #Change this
        return {'state_base': state, 'self': my_state}

    def __setstate__(self, state):
        tu.StateBase.__setstate__(self, state['state_base'])
        self.name = state['self'][0] #Change this
        self.__init_unpicklables__()



