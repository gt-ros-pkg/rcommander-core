import tool_utils as tu
from PyQt4.QtGui import *
from PyQt4.QtCore import *

TOOL_NAME = 'outcome'

class EmptyState(tu.StateBase):

    def __init__(self, name, temporary, tool_name=TOOL_NAME):
        tu.StateBase.__init__(self, name)
        self.set_runnable(False)
        self.temporary = temporary
        self.tool_name = tool_name

    def __getstate__(self):
        state = tu.StateBase.__getstate__(self)
        my_state = [self.temporary]
        return {'parent_state': state, 'self': my_state}

    def __setstate__(self, state):
        tu.StateBase.__setstate__(self, state['parent_state'])
        self.temporary = state['self'][0]

class OutcomeTool(tu.ToolBase):

    def __init__(self, button, rcommander):
        tu.ToolBase.__init__(self, rcommander, TOOL_NAME, 'Add Outcome')
        self.button = button
        self.rcommander.connect(self.button, SIGNAL('clicked()'), self.activate_cb)

    def activate_cb(self, loaded_node_name=None):
        tu.ToolBase.activate_cb(self, loaded_node_name)
        self.outcome_mode()

    def outcome_mode(self):
        cidx = self.rcommander.ui.node_settings_tabs.indexOf(self.rcommander.ui.connections_tab)
        self.rcommander.ui.node_settings_tabs.setCurrentIndex(cidx)
        self.rcommander.ui.run_button.setDisabled(True)
        self.rcommander.ui.reset_button.setDisabled(True)

    def _create_node(self, name=None):
        if name == None:
            nname = self.name + str(self.counter)
        else:
            nname = name
        state = EmptyState(nname, False) 
        return state

    def _node_selected(self, node):
        self.rcommander.disable_buttons()
        self.outcome_mode()

    #Don't have any properties
    def fill_property_box(self, pbox):
        return

    #Can't reset
    def reset(self):
        return

    #Can't save
    def save(self):
        return

#NavigateState setting state {'self': ['navigate1', [0.0, 0.0], 0.0, 'map'], 
#                             'simple_state': ['navigate1', 'navigate', 
#                                 {'preempted': PyQt4.QtCore.QString(u'preempted'), 'aborted': PyQt4.QtCore.QString(u'aborted'), 'succeeded': PyQt4.QtCore.QString(u'succeeded')}]}
#    
