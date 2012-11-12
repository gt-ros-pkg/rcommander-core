import tool_utils as tu
from PyQt4.QtGui import *
from PyQt4.QtCore import *

## Tool class to represent outcomes of other nodes in SMACH
class OutcomeTool(tu.ToolBase):

    ## Constructor
    #
    # @param button QT button that's on the tool bar.
    # @param rcommander RCommander main class
    def __init__(self, button, rcommander):
        tu.ToolBase.__init__(self, rcommander, tu.EmptyState.TOOL_NAME, 'Add Outcome', tu.EmptyState)
        self.button = button
        self.rcommander.connect(self.button, SIGNAL('clicked()'), self.activate_cb)

    ## Override ToolBase's activate_cb
    # since this tool is special and builtin.
    def activate_cb(self, loaded_node_name=None):
        tu.ToolBase.activate_cb(self, loaded_node_name)
        self.outcome_mode()
        self.rcommander.ui.add_button.setDisabled(False)

    ## Start Outcome mode, which disables the run, and reset buttons.
    def outcome_mode(self):
        cidx = self.rcommander.ui.node_settings_tabs.indexOf(self.rcommander.ui.connections_tab)
        self.rcommander.ui.node_settings_tabs.setCurrentIndex(cidx)
        self.rcommander.ui.run_button.setDisabled(True)
        self.rcommander.ui.reset_button.setDisabled(True)

    ## Inherited.
    def new_node(self, name=None):
        if name == None:
            nname = self.name + str(self.counter)
        else:
            nname = name
        state = tu.EmptyState(nname, False) 
        return state

    ## Inherited
    def set_node_properties(self, node):
        self.rcommander.disable_buttons()
        self.outcome_mode()

    ## Override ToolBase's fill_property_box
    # Don't have any properties so don't fill out anything here
    def fill_property_box(self, pbox):
        return

    ## Inherited
    # Outcome nodes don't reset
    def reset(self):
        return

    ## Inherited
    # Outcome nodes don't save
    def save(self):
        return

