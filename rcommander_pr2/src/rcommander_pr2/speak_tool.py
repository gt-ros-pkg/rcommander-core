import roslib; roslib.load_manifest('rcommander_pr2')
import rospy
from PyQt4.QtGui import *
from PyQt4.QtCore import *
import rcommander_core.tool_utils as tu

from tts_server.srv import *
import smach

class SpeakTool(tu.ToolBase):

    DEFAULT_TEXT = 'hello world'

    def __init__(self, rcommander):
        tu.ToolBase.__init__(self, rcommander, 'speak', 'Speak', SpeakNode)
        self.available_voices = rospy.ServiceProxy('available_voices', AvailableVoices)

    def fill_property_box(self, pbox):
        formlayout = pbox.layout()
        self.text = QPlainTextEdit(pbox)
        self.voices_box = QComboBox(pbox)
        ret = self.available_voices()
        for v in ret.voices:
            self.voices_box.addItem(v)

        self.reset()
        formlayout.addRow('&Voice', self.voices_box)
        formlayout.addRow('&Say', self.text)

    def new_node(self, name=None):
        if name == None:
            nname = self.name + str(self.counter)
        else:
            nname = name
        
        txt = str(self.text.document().toPlainText())
        voice = str(self.voices_box.currentText())
        if voice == ' ':
            return None
        return SpeakNode(nname, txt, voice)

    def set_node_properties(self, my_node):
        self.document.setPlainText(my_node.text)

    def reset(self):
        self.document = QTextDocument(self.DEFAULT_TEXT)
        self.layout = QPlainTextDocumentLayout(self.document)
        self.document.setDocumentLayout(self.layout)
        self.text.setDocument(self.document)
        self.voices_box.setCurrentIndex(0)


class SpeakNodeSmach(smach.State): 

    def __init__(self, text, voice):
        smach.State.__init__(self, outcomes = ['done'], input_keys = [], output_keys = [])
        self.text = text
        self.voice = voice

    def execute(self, userdata):
        self.say = rospy.ServiceProxy('say', Say)
        self.say(self.voice, self.text)
        return 'done'


class SpeakNode(tu.StateBase):

    def __init__(self, name, text, voice):
        tu.StateBase.__init__(self, name)
        self.text = text
        self.voice = voice

    def get_smach_state(self):
        return SpeakNodeSmach(self.text, self.voice)


#class SpeakNode(smach.State, tu.StateBase): 
#
#    def __init__(self, name, text, sound_client=None):
#        self.name = name
#        self.text = text
#        self.sound_client = sound_client
#        self.__init_unpicklables__()
#
#    def execute(self, userdata):
#        self.sound_client.say(self.text)#, 'voice_kal_diphone')
#        return 'done'
#
#    def __init_unpicklables__(self):
#        tu.StateBase.__init__(self, self.name)
#        smach.State.__init__(self, outcomes = ['done'], input_keys = [], output_keys = [])
#        if self.sound_client == None:
#            self.sound_client = SoundClient()
#
#    def __getstate__(self):
#        state = tu.StateBase.__getstate__(self)
#        my_state = [self.name, self.text]
#        return {'state_base': state, 'self': my_state}
#
#    def __setstate__(self, state):
#        tu.StateBase.__setstate__(self, state['state_base'])
#        self.name, self.text = state['self']
#        self.__init_unpicklables__()



