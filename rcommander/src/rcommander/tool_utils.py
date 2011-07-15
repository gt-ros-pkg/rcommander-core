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

def make_radio_box(parent, options, name_preffix):
    container_name = name_preffix + '_radio_box'

    container = QWidget(parent)
    container.setObjectName(container_name)
    hlayout = QHBoxLayout(container)
    radio_buttons = []

    for option in options:
        r = QRadioButton(container)
        r.setObjectName(name_preffix + '_' + option)
        r.setText(option)
        hlayout.addWidget(r)
        radio_buttons.append(r)
    radio_buttons[0].setChecked(True)

    return container, radio_buttons


class SliderBox:

    def __init__(self, parent, initial_value, max_value, min_value, ticks, name_preffix, units=''):
        self.min_value = min_value
        self.max_value = max_value

        container_name = name_preffix + '_box'
        disp_name = name_preffix + '_disp'
        slider_name = name_preffix + '_slider'

        nticks = float(max_value - min_value) / float(ticks)
        tick_size = int(round(100. / nticks))
        tick_pos = self._units_to_slider(initial_value)

        container = QWidget(parent)
        container.setObjectName(container_name)
        
        disp = QLabel(container)
        disp.setObjectName(disp_name)
        disp.setText('%.2f %s' % (initial_value, units))

        slider = QSlider(container)
        slider.setSingleStep(1)
        slider.setOrientation(Qt.Horizontal)
        slider.setTickPosition(QSlider.TicksBelow)
        slider.setTickInterval(tick_size)
        slider.setObjectName(slider_name)
        slider.setSliderPosition(tick_size)
        slider.setValue(tick_pos)

        hlayout = QHBoxLayout(container)
        hlayout.addWidget(disp)
        hlayout.addWidget(slider)

        def slider_moved_cb(disp, value):
            cv = self._slider_to_units(value)
            disp.setText('%3.2f %s' % (cv, units))
        parent.connect(slider, SIGNAL('sliderMoved(int)'), ft.partial(slider_moved_cb, disp))

        self.container = container
        self.slider = slider

    def _slider_to_units(self, value):
        return ((value / 100.) * (self.max_value - self.min_value)) + self.min_value
        
    def _units_to_slider(self, value):
        return int(round(((value - self.min_value) / (self.max_value - self.min_value)) * 100.0))

    def value(self):
        return self._slider_to_units(self.slider.value())

    def set_value(self, value):
        self.slider.setValue(self._units_to_slider(value))


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
        current_node = self._create_node()
        current_node_name = current_node.get_name()
        self.name_input = QLineEdit()
        self.name_input.setText(current_node_name)
        formlayout.addRow('Name', self.name_input)

        if not issubclass(current_node.__class__, InfoStateBase):
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
                #TODO abstract this line out
                #outcome_name = self.get_current_node_name() + '_' + outcome
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

    def create_node(self, unique=True):
        if unique:
            self.counter = self.counter + 1
        n = self._create_node(str(self.name_input.text()))
        n.tool_name = self.get_name()
        #n.outcome_choices = self._get_outcome_choices()
        return n

    def node_selected(self, node):
        #print 'node name', node.get_name()
        outcome_list = self.rcommander.current_children_of(node.get_name())
        #print 'outcome_list', outcome_list
        for outcome_name, connected_node in outcome_list:
            if not self.outcome_inputs.has_key(outcome_name):
                continue
            widget = self.outcome_inputs[outcome_name]
            #print 'connected_node', connected_node
            #print 'widget returned', widget.findText(connected_node)
            widget.setCurrentIndex(widget.findText(connected_node))
        #print 'returned'

        #Set default choices to new node info
        #for outcome_name in node.outcome_choices:
        #    selected = node.outcome_choices[outcome_name]
        #    widget = self.outcome_inputs[outcome_name]
        #    widget.setCurrentIndex(widget.findText(selected))

        self.name_input.setText(node.get_name())
        self.set_loaded_node_name(node.get_name())
        self.loaded_node_name = node.get_name()
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
        #self.outcome_choices = []
        self.remapping = {}
        self.runnable = True

    def is_runnable(self):
        return self.runnable

    def set_runnable(self, v):
        self.runnable = v

    def set_name(self, name):
        self.name = name

    def get_name(self):
        return self.name

    def get_registered_outcomes(self):
        return []

    def __getstate__(self):
        #r = [self.name, self.tool_name, self.outcome_choices, self.remapping]
        r = [self.name, self.tool_name, self.remapping]
        return r

    def __setstate__(self, state):
        #print 'state base', state
        name, tool, choices, remapping = state
        self.name = name
        self.tool_name = tool
        #self.outcome_choices = choices
        self.remapping = remapping

    def source_for(self, var_name):
        return self.remapping[var_name]

    def set_source_for(self, var, source):
        self.remapping[var] = source


class InfoStateBase(StateBase):

    GLOBAL_NAME = 'Global'

    def __init__(self, name):
        StateBase.__init__(self, name)
        self.set_runnable(False)

    def set_info(self, info):
        raise RuntimeError('Unimplemented method. set_info')

    def get_info(self):
        raise RuntimeError('Unimplemented method. get_info')

    def get_registered_outcomes(self):
        return [InfoStateBase.GLOBAL_NAME]


class SimpleStateCB:

    def __init__(self, cb_func, input_keys, output_keys):
        self.cb_func = cb_func
        self.input_keys = input_keys
        self.output_keys = output_keys

    def __call__(self, userdata, default_goal): 
        return self.cb_func(userdata, default_goal)

    def get_registered_input_keys(self):
        return self.input_keys

    def get_registered_output_keys(self):
        return self.output_keys

    def get_registered_outcomes(self):
        return []


class SimpleStateBase(smach_ros.SimpleActionState, StateBase):
    def __init__(self, name, action_name, action_type, goal_cb_str, input_keys=[]):
        #func = eval('self.%s' % goal_cb_str)
        #print dir(func)
        #func.get_registered_input_keys  = self.get_registered_input_keys
        #func.get_registered_output_keys = self.get_registered_output_keys
        #func.get_registered_outcomes    = self.get_registered_outcomes
        smach_ros.SimpleActionState.__init__(self, action_name, action_type, goal_cb = SimpleStateCB(eval('self.%s' % goal_cb_str), input_keys, []))

        StateBase.__init__(self, name)

        self.action_name = action_name
        self.action_type = action_type
        self.goal_cb_str = goal_cb_str
        self.input_keys = input_keys

    def __call__(self, userdata, default_goal): 
        f = eval('self.%s' % self.goal_cb_str)
        return f(userdata, default_goal)

    def __getstate__(self):
        sb_state = StateBase.__getstate__(self)
        return {'sb_state': sb_state, 'action_name': self.action_name, 'action_type': self.action_type, 'goal_cb_str': self.goal_cb_str, 'input_keys': self.input_keys}

    def __setstate__(self, state):
        StateBase.__setstate__(self, state['sb_state'])
        self.action_name = state['action_name']
        self.action_type = state['action_type']
        self.goal_cb_str = state['goal_cb_str']
        self.input_keys  = state['input_keys']
        smach_ros.SimpleActionState.__init__(self, self.action_name, self.action_type, goal_cb = SimpleStateCB(eval('self.%s' % self.goal_cb_str), self.input_keys, []))
               





