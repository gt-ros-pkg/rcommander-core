from PyQt4 import QtGui
from PyQt4.QtGui import *
from PyQt4.QtCore import *
import smach_ros
import functools as ft

def create_tool_button(name, container):
    button = QtGui.QToolButton(container)
    sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Fixed)
    sizePolicy.setHorizontalStretch(0)
    sizePolicy.setVerticalStretch(0)
    sizePolicy.setHeightForWidth(button.sizePolicy().hasHeightForWidth())
    button.setSizePolicy(sizePolicy)
    button.setAutoRaise(True)
    button.setObjectName("button")
    button.setCheckable(True)
    container.layout().addWidget(button)
    button.setText(QtGui.QApplication.translate("RCommanderWindow", name, None, QtGui.QApplication.UnicodeUTF8))
    return button


class ToolBase:

    def __init__(self, rcommander, name, button_name):
        self.rcommander = rcommander
        self.properties_box = self.rcommander.ui.properties_tab
        self.connections_box = self.rcommander.ui.connections_tab

        self.name = name
        self.name_input = None
        self.loaded_node_name = None

        self.button = None
        self.button_name = button_name

        self.outcome_inputs = {}
        #self.combo_box_cbs = {}
        self.counter = 0


    def get_name(self):
        return self.name

    def create_button(self, container):
        self.button = create_tool_button(self.button_name, container)
        self.rcommander.connect(self.button, SIGNAL('clicked()'), self.activate_cb)
        return self.button

    def activate_cb(self, loaded_node_name=None):
        #print 'clicked on', self.name
        self.rcommander.enable_buttons()
        #self.loaded_node_name = None
        self.set_loaded_node_name(loaded_node_name)
        self.rcommander.add_mode()
        self.rcommander.empty_properties_box()
        if self.button.isChecked():
            self.fill_property_box(self.properties_box)
            self.fill_connections_box(self.connections_box)
            self.rcommander.set_selected_tool(self.get_name())
        else:
            self.rcommander.set_selected_tool(None)

    def get_outcomes(self):
        return self._create_node().get_registered_outcomes()

    def _get_outcome_choices(self):
        outcome_choices = {}
        for outcome_name in self.outcome_inputs.keys():
            outcome_choices[outcome_name] = str(self.outcome_inputs[outcome_name].currentText())
        return outcome_choices

    def refresh_connections_box(self):
        if not self.button.isChecked():
            return

        #record what is selected
        outcome_choices = self._get_outcome_choices()
        node_name = str(self.name_input.text())

        #empty box
        self.rcommander.empty_container(self.connections_box)

        #fill it back up
        self.fill_connections_box(self.connections_box)

        #replace defaults with saved values
        self.name_input.setText(node_name)
        for k in outcome_choices:
            selected = outcome_choices[k]
            widget = self.outcome_inputs[k]
            widget.setCurrentIndex(widget.findText(selected))

    def fill_connections_box(self, pbox):
        formlayout = pbox.layout()

        self.name_input = QLineEdit()
        self.name_input.setText(self._create_node().get_name())
        formlayout.addRow('Name', self.name_input)

        for outcome in self.get_outcomes():
            #Make a new combobox and add available choices to it
            input_box = QComboBox(pbox)
            nodes = self.rcommander.connectable_nodes(self.get_current_node_name(), outcome)
            nodes.sort()
            for n in nodes:
                input_box.addItem(n)

            #add to view 
            formlayout.addRow(outcome, input_box)
            #set outcome as default
            input_box.setCurrentIndex(input_box.findText(outcome))
            #store object
            self.outcome_inputs[outcome] = input_box

            #make callback
            def cb(outcome, new_index):
                new_outcome = str(self.outcome_inputs[outcome].currentText())
                self.rcommander.connection_changed(self.get_current_node_name(), outcome, new_outcome)
            outcome_cb = ft.partial(cb, outcome)
            self.rcommander.connect(input_box, SIGNAL('currentIndexChanged(int)'), outcome_cb)
            #self.combo_box_cbs[outcome] = outcome_cb

    def set_loaded_node_name(self, name):
        self.loaded_node_name = name

    def get_current_node_name(self):
        return self.loaded_node_name 
        #return str(self.name_input.text())

    def create_node(self, unique=True):
        if unique:
            print 'COUNTER INCREMENTED!'
            self.counter = self.counter + 1
        n = self._create_node(str(self.name_input.text()))
        n.tool_name = self.get_name()
        n.outcome_choices = self._get_outcome_choices()
        return n

    def node_selected(self, node):
        #Set default choices to new node info
        for outcome_name in node.outcome_choices:
            selected = node.outcome_choices[outcome_name]
            widget = self.outcome_inputs[outcome_name]
            widget.setCurrentIndex(widget.findText(selected))
        self.name_input.setText(node.get_name())
        self.set_loaded_node_name(node.get_name())
        #self.loaded_node_name = node.get_name()
        self._node_selected(node)

    ##
    # @param pbox a QT widget using a FormLayout that can be filled with
    #             appropriate controls for this tool
    def fill_property_box(self, pbox):
        pass

    ##
    # Called when user clicks Add
    #
    # @return a valid smach state derived from StateBase
    def _create_node(self, name=None):
        pass

    ##
    # Called by parent when user selects a node created by this tool
    #
    # @param node a State object created by this tool
    def _node_selected(self, node):
        pass

    ##
    # Resets the state of this GUI tool, sets all options to sensible defaults
    def reset(self):
        pass


class StateBase:

    def __init__(self, name):
        self.name = name
        self.tool_name = None
        self.outcome_choices = []

    def set_name(self, name):
        self.name = name

    def get_name(self):
        return self.name

    def get_registered_outcomes(self):
        return []

    def __getstate__(self):
        r = [self.name, self.tool_name, self.outcome_choices]
        return r

    def __setstate__(self, state):
        #print 'state base', state
        name, tool, choices = state
        self.name = name
        self.tool_name = tool
        self.outcome_choices = choices


class SimpleStateBase(smach_ros.SimpleActionState, StateBase):
    def __init__(self, name, action_name, action_type, goal_cb):
        smach_ros.SimpleActionState.__init__(self, action_name, action_type, goal_cb = goal_cb)
        StateBase.__init__(self, name)


