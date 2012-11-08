##
# Classes that can be inherited from to create ROS Commander plugins         ## 
# and convenient utility functions.

import PyQt4.QtGui as qtg
import PyQt4.QtCore as qtc
import smach_ros
import functools as ft
import actionlib_msgs.msg as am
import os.path as pt
from tf_broadcast_server.srv import GetTransforms
import rospy
import actionlib
import smach
import numpy as np

## Take all widgets out of a container
# @param pbox A QT container.
def empty_container(pbox): 
    layout = pbox.layout()

    for i in range(layout.count()):
        item = layout.itemAt(0)
        layout.removeItem(item)
    children = pbox.children()

    for c in children[1:]:
        try:
            layout.removeWidget(c)
            c.setParent(None)
        except TypeError, e:
            pass

    layout.invalidate()
    pbox.update()

## Mappings from goal statuses to strings.
status_dict = {am.GoalStatus.PENDING   : 'PENDING',
               am.GoalStatus.ACTIVE    : 'ACTIVE',   
               am.GoalStatus.PREEMPTED : 'PREEMPTED',
               am.GoalStatus.SUCCEEDED : 'SUCCEEDED',
               am.GoalStatus.ABORTED   : 'ABORTED',  
               am.GoalStatus.REJECTED  : 'REJECTED', 
               am.GoalStatus.PREEMPTING: 'PREEMPTING',
               am.GoalStatus.RECALLING : 'RECALLING',
               am.GoalStatus.RECALLED  : 'RECALLED', 
               am.GoalStatus.LOST      : 'LOST'}

## Converts goal statuses to strings
# @param status status code from status_dict above.
def goal_status_to_string(status):
    return status_dict[status]

## ROS Commander exception thrown in cases where executing SMACH nodes
# run into any type of error with TaskFrames.
class TaskFrameError(Exception):
    def __init__(self, node_name, destination):
        self.node_name = node_name

    def __str__(self):
        return repr('Error: Node %s needs a task frame to relate to frame %s' 
                % (self.node_name, self.destination))

## ROS Commander exception thrown in clases where executing SMACH noes
# run into any kind of tf error.
class FrameError(Exception):
    def __init__(self, node_name, source, destination):
        self.node_name = node_name
        self.source = source
        self.destination = destination

## Creates and manage a QT combo box.
class ComboBox:

    def __init__(self):
        pass

    ## Creates the combo box
    def create_box(self, pbox):
        self.combobox = qtg.QComboBox(pbox)

    ## Sets selected item
    #
    # @param item item to set (string)
    # @param create whether to create the item in this ComboBox if it isn't already there
    def set_text(self, item, create=True):
        idx = combobox_idx(self.combobox, item, create=create)
        if idx != -1:
            self.combobox.setCurrentIndex(idx)

    ## Gets currently selected text.
    def text(self):
        return str(self.combobox.currentText())

## Creates and manage a ComboBox that stores TF frame names.
class FrameBox(ComboBox):

    ## Constructor
    # @param frames_service A rospy.ServiceProxy object to GetTransforms.
    def __init__(self, frames_service=None):
        ComboBox.__init__(self)
        if frames_service == None:
            frames_service = rospy.ServiceProxy('get_transforms', GetTransforms, persistent=True)
        self.frames_service = frames_service

    ## Creates the ComboBox
    def create_box(self, pbox):
        ComboBox.create_box(self, pbox)
        for f in self.frames_service().frames:
            self.combobox.addItem(f)
        self.setEnabled = self.combobox.setEnabled
        return self.combobox

## Creates a button in ROS Commander for the given tool (called by ToolBase)
def create_tool_button(name, container):
    button = qtg.QToolButton(container)
    sizePolicy = qtg.QSizePolicy(qtg.QSizePolicy.Expanding, qtg.QSizePolicy.Fixed)
    sizePolicy.setHorizontalStretch(0)
    sizePolicy.setVerticalStretch(0)
    sizePolicy.setHeightForWidth(button.sizePolicy().hasHeightForWidth())
    button.setSizePolicy(sizePolicy)
    button.setAutoRaise(True)
    button.setObjectName("button")
    button.setCheckable(True)
    container.layout().addWidget(button)
    button.setText(qtg.QApplication.translate("RCommanderWindow", name, None, qtg.QApplication.UnicodeUTF8))
    return button

## Makes a QT radio box.
# @param parent Parent QT object.
# @param options Options to display in radio box (list of strings).
# @param name_preffix Prefix for names of containers created (string).
def make_radio_box(parent, options, name_preffix):
    container_name = name_preffix + '_radio_box'

    container = qtg.QWidget(parent)
    container.setObjectName(container_name)
    hlayout = qtg.QHBoxLayout(container)
    radio_buttons = []

    for option in options:
        r = qtg.QRadioButton(container)
        r.setObjectName(name_preffix + '_' + option)
        r.setText(option)
        hlayout.addWidget(r)
        radio_buttons.append(r)
    radio_buttons[0].setChecked(True)

    return container, radio_buttons

## Finds index of item in QComboBox
#
# @param combobox QComboBox object
# @param name name of selection
# @param create whether to create the item if it's not in the list
def combobox_idx(combobox, name, create=True):
    if name == None:
        name = ' '
    idx = combobox.findText(name)
    if idx == -1:
        if create == True:
            combobox.addItem(name)
            idx = combobox.findText(name)
            return idx
        else:
            return -1
    return idx

## Creates a QT double spin box
# @param minv Mininum value allowed in spin box (double).
# @param maxv Maximum value allowed in spin box (double).
# @param step Amount to increment at each click (double).
def double_spin_box(parent, minv, maxv, step):
    box = qtg.QDoubleSpinBox(parent)
    box.setMinimum(minv)
    box.setMaximum(maxv)
    box.setSingleStep(step)
    return box

## Gets the value of the currently selected radio button.
def selected_radio_button(buttons_list):
    selected = None
    for r in buttons_list:
        if r.isChecked():
            selected = str(r.text())
    return selected

## Creates a QT Slider
class SliderBox:

    ## Constructor
    #
    # @param parent Parent widget (QtWidget)
    # @param initial_value Initial value for slider (double).
    # @param max_value Maximum value for slider (double).
    # @param min_value Minimum value for slider (double).
    # @param ticks Size of each tick (double).
    # @param name_preffix Prefix for name of objects created.
    # @param unit Unit of slider.
    def __init__(self, parent, initial_value, max_value, min_value, ticks, 
            name_preffix, unit=''):
        self.min_value = min_value
        self.max_value = max_value

        container_name = name_preffix + '_box'
        disp_name = name_preffix + '_disp'
        slider_name = name_preffix + '_slider'

        nticks = float(max_value - min_value) / float(ticks)
        tick_size = int(round(100. / nticks))
        tick_pos = self._units_to_slider(initial_value)

        container = qtg.QWidget(parent)
        container.setObjectName(container_name)
        
        disp = qtg.QLabel(container)
        disp.setObjectName(disp_name)
        disp.setText('%.2f %s' % (initial_value, unit))

        slider = qtg.QSlider(container)
        slider.setSingleStep(1)
        slider.setOrientation(qtc.Qt.Horizontal)
        slider.setTickPosition(qtg.QSlider.TicksBelow)
        slider.setTickInterval(tick_size)
        slider.setObjectName(slider_name)
        slider.setSliderPosition(tick_size)
        slider.setValue(tick_pos)

        hlayout = qtg.QVBoxLayout(container)
        hlayout.addWidget(slider)
        hlayout.addWidget(disp)

        def slider_moved_cb(disp, value):
            cv = self._slider_to_units(value)
            disp.setText('%3.2f %s' % (cv, unit))
        parent.connect(slider, qtc.SIGNAL('sliderMoved(int)'), ft.partial(slider_moved_cb, disp))

        self.container = container
        self.slider = slider
        self.disp = disp
        self.unit = unit

    ## Converts from slider coordinates to values in units given.
    def _slider_to_units(self, value):
        return ((value / 100.) * (self.max_value - self.min_value)) + self.min_value
       
    ## Converts from units given to slider position.
    def _units_to_slider(self, value):
        return int(round(((value - self.min_value) / (self.max_value - self.min_value)) * 100.0))

    ## Gets the current value of this slider.
    def value(self):
        return self._slider_to_units(self.slider.value())

    ## Sets the value of this slider.
    def set_value(self, value):
        self.slider.setValue(self._units_to_slider(value))
        self.disp.setText('%3.2f %s' % (value, self.unit))

## Base class for tools (plugins in ROS Commander that displays as button in
# tabs at the top)
class ToolBase:

    ## Constructor
    # This method needs to be called by inherited tool
    def __init__(self, rcommander, name, button_name, smach_class):
        self.rcommander = rcommander

        self.name = name
        self.name_input = None
        self.loaded_node_name = None

        self.button = None
        self.button_name = button_name

        self.smach_class = smach_class
        self.outcome_inputs = {}
        self.counter = 0

        self.node_exists = False
        self.saved_state = None

    ## Creates the tool's Qt button (called by RCommander class)
    def create_button(self, container):
        self.button = create_tool_button(self.button_name, container)
        self.rcommander.connect(self.button, qtc.SIGNAL('clicked()'), 
                self.activate_cb)
        return self.button

    ## Called when the tool's button has been clicked.
    def activate_cb(self, loaded_node_name=None):
        self.rcommander.notify_activated()

        self.node_exists = False
        self.rcommander.enable_buttons()
        self.set_loaded_node_name(loaded_node_name)
        self.rcommander.add_mode()
        self.rcommander.empty_properties_box()

        if self.button.isChecked():
            #send fresh boxes out
            self.rcommander.set_selected_tool(self.get_smach_class())
            #This needs to appear *before* the various fills as they can fail
            self.fill_property_box(self.rcommander.ui.properties_tab)
            self.fill_connections_box(self.rcommander.ui.connections_tab)
            self.rcommander.ui.node_settings_tabs.setCurrentIndex(0)
        else:
            self.rcommander.set_selected_tool(None)

        if self.saved_state != None and loaded_node_name == None:
            self.set_node_properties(self.saved_state)

    ## Clear the state currently saved in this tool (state are saved in the
    # tool when they have not been added to the graph).
    def clear_saved_state(self):
        self.saved_state = None

    #Called by RCommander when user deselects this tool
    def deselect_tool(self):
        # Is the node we're displaying saved? has it been added??
        if self.node_exists:
            return
        else:
            try:
                self.saved_state = self.new_node()
            except Exception, e:
                rospy.loginfo(str(e))

    def _get_outcome_choices(self):
        outcome_choices = {}
        for outcome_name in self.outcome_inputs.keys():
            outcome_choices[outcome_name] = str(self.outcome_inputs[outcome_name].currentText())
        return outcome_choices

    ## Repopulates the connections tab with information from the node being
    # edited.
    def refresh_connections_box(self):
        if not self.button.isChecked():
            return

        #record what is selected
        outcome_choices = self._get_outcome_choices()
        node_name = str(self.name_input.text())

        #empty box
        empty_container(self.rcommander.ui.connections_tab)

        #fill it back up
        self.fill_connections_box(self.rcommander.ui.connections_tab)

        #replace defaults with saved values
        self.name_input.setText(node_name)
        for k in outcome_choices:
            selected = outcome_choices[k]
            widget = self.outcome_inputs[k]
            widget.setCurrentIndex(widget.findText(selected))

    ## Fills the connections tab with information from the node being edited.
    def fill_connections_box(self, pbox):
        formlayout = pbox.layout()

        if hasattr(self, 'set_child_node'):
            if self.rcommander.selected_node == None:
                return
            else:
                state = self.rcommander.graph_model.get_state(self.rcommander.selected_node)
                self.set_child_node(state)

        self.name_input = qtg.QLineEdit() #Needs to occur before new_node as it can fail
        formlayout.addRow('Name', self.name_input)

        if self.get_current_node_name() == None:
            current_node = self.new_node()
        else:
            current_node = self.rcommander.graph_model.get_state(self.get_current_node_name())

        current_node_smach = current_node.get_smach_state()
        self.rcommander.connect_node(current_node_smach)

        current_node_name = current_node.get_name()
        self.name_input.setText(current_node_name)

        if issubclass(current_node.__class__, EmptyState):
            return 

        registered_outcomes = list(current_node_smach.get_registered_outcomes())
        registered_outcomes.sort()
        for outcome in registered_outcomes:
            #Make a new combobox and add available choices to it
            input_box = qtg.QComboBox(pbox)
            nodes = self.rcommander.connectable_nodes(self.get_current_node_name(), outcome)

            nodes.sort()
            for n in nodes:
                input_box.addItem(n)

            #add to view 
            formlayout.addRow(outcome, input_box)
            self.outcome_inputs[outcome] = input_box

            #make callback
            def cb(outcome, new_index):
                new_outcome = str(self.outcome_inputs[outcome].currentText())
                self.rcommander.connection_changed(self.get_current_node_name(), outcome, new_outcome)

            outcome_cb = ft.partial(cb, outcome)
            self.rcommander.connect(input_box, qtc.SIGNAL('currentIndexChanged(int)'), outcome_cb)
            if len(nodes) == 1:
                self.rcommander.connection_changed(self.get_current_node_name(), outcome, nodes[0])

    ## Sets the name of the node displayed by this tool.
    # @param name string
    def set_loaded_node_name(self, name):
        self.loaded_node_name = name

    ## Gets name of node currently displayed by this tool.
    def get_current_node_name(self):
        return self.loaded_node_name 

    ## Called by ROS Commander to create a new node.
    def create_node(self, unique=True):
        if unique:
            n = self.new_node(str(self.name_input.text()))
            if n == None:
                return None
            has_node_name = self.rcommander.has_node_name(n.get_name())
            while has_node_name:
                self.counter = self.counter + 1
                n = self.new_node()
                has_node_name = self.rcommander.has_node_name(n.get_name())
        else:
            n = self.new_node(str(self.name_input.text()))

        if unique:
            self.node_exists = True
            self.saved_state = None

        return n

    ## Called by ROS Commander when a node is selected in the GUI.
    def node_selected(self, node):
        self.node_exists = True
        outcome_list = self.rcommander.current_children_of(node.get_name())
        for outcome_name, connect_node in outcome_list:
            if not self.outcome_inputs.has_key(outcome_name):
                continue
            widget = self.outcome_inputs[outcome_name]
            widget.setCurrentIndex(widget.findText(connect_node))

        self.name_input.setText(node.get_name())
        self.set_loaded_node_name(node.get_name())
        self.loaded_node_name = node.get_name()
        self.set_node_properties(node)

    ## Returns the class of SMACH states created by this tool's nodes.
    def get_smach_class(self):
        return self.smach_class

    ##
    # Responsible for setting up graphical widgets allowing users to edit the
    # parameters of an action. Called when users click on a node of the class
    # smach_class (given in constructor) and whenever this tool is selected via
    # its menu bar button.
    #
    # @param pbox a QT widget with a FormLayout that can be filled with
    #             appropriate controls for this tool
    #
    def fill_property_box(self, pbox):
        pass

    ##
    # When called must return an instance of smach_class (an object which
    # inherits from tool_utils.StateBase given in constructor). 
    #
    # Called with a name parameter when users click the big Add button, called
    # with a name=None by the connections tab to find out how many outgoing
    # connections this node has.
    #
    # @return a valid smach state derived from StateBase
    def new_node(self, name=None):
        pass

    ##
    # Called by parent when user selects a node created by this tool (of class
    # smach_class).
    #
    # @param node a State object created by this tool of class smach_class
    def set_node_properties(self, node):
        pass

    ##
    # Resets the state of this GUI tool, sets all options to sensible defaults.
    # Called when users click the big Reset button.
    #
    def reset(self):
        pass


## Base class for a ROS Commander state.
class StateBase:

    ##
    # Base construction for a state.
    #
    # @param name name of this state
    # @param outputs a dictionary mapping names (strings) to Python classes.
    def __init__(self, name, outputs={}):
        self.name = name
        self.remapping = {}
        self.runnable = True
        self.outputs = outputs

    ##
    # Returns a list of names of all declared outputs variables (same as the
    # output variables in SMACH).
    #
    def output_names(self):
        return self.outputs.keys()

    ##
    # Gives the type (class) of output with the given name.  Used by RCommander
    # to match node's input with other node's outputs.
    #
    # @param name string, name of output whose class we are interested in.
    def output_type(self, name):
        return self.outputs[name]
    
    ##
    # Sets the name of this action, used when user sets name in the GUI's Connections
    # tab.
    #
    # @param name string
    def set_name(self, name):
        self.name = name

    ##
    # Gets the name of this object.
    #
    def get_name(self):
        return self.name

    ##
    # Gets the name of an output variable of another node in the state machine
    # that maps to the input var_name in this node.
    #
    # @param var_name an input variable for this state
    def remapping_for(self, var_name):
        return self.remapping[var_name]

    ##
    # Sets the name of an output variable of another node in the state machine
    # that maps to the input var_name in this node.
    #
    # @param var input variable for this node.
    # @param remapped output variable of another node.
    def set_remapping_for(self, var, remapped):
        self.remapping[var] = remapped

    ##
    # Checks to see whether this node is executable.
    #
    def is_runnable(self):
        return self.runnable

    ##
    # Identifies whether this node is executable or not (almost always this
    # should be True.
    #
    # @param v boolean value
    def set_runnable(self, v):
        self.runnable = v

    ##
    # Returns an object that implements smach.State, needed by RCommander to
    # create the final state machine.
    #
    def get_smach_state(self):
        raise RuntimeError('Unimplemented method: get_smach_state')


## EmptyStates are used to represent outcomes.
class EmptyState(StateBase):

    TOOL_NAME = 'outcome'

    ## Constructor
    # @param name Name of this outcome.
    # @param temporary
    # @param tool_name Name of tool in this outcome.
    def __init__(self, name, temporary, tool_name=TOOL_NAME):
        StateBase.__init__(self, name)
        self.set_runnable(False)
        self.temporary = temporary
        self.tool_name = tool_name

    def get_smach_state(self):
        return self

## Inherit from this base class to create states that can have other states
# embedded in it.
class EmbeddableState(StateBase):

    ## Constructor
    # @param name Name of this state (string)
    # @param child_gm Child state machine (a GraphModel object).
    # @param outputs Possible outputs of this state.
    def __init__(self, name, child_gm, outputs={}):
        StateBase.__init__(self, name, outputs)
        self.child_gm = child_gm
        self.child_document = None 

        #Look inside state machine and look for things with remaps
        self._init_child(child_gm)

    ## Gets the document object in the child state machine.
    def get_child_document(self):
        return self.child_document

    ## Takes the child state machine out of this state.
    def abort_child(self):
        fetus = self.child_gm 
        self.child_gm = None
        return fetus

    ## Getter for child state machine (but this state still has a pointer).
    def get_child(self):
        return self.child_gm

    ## Setter for child state machine.
    def set_child(self, child):
        self.child_gm = child

    ## Look for remappings in child state machine.
    #
    # @param child_gm GraphModel object
    def _init_child(self, child_gm):
        if child_gm != None:
            self.child_document = child_gm.get_document()
            #child_gm.set_document(None)
            for node_name in child_gm.states_dict.keys():
                mapping = child_gm.get_state(node_name).remapping
                for input_key in mapping.keys():
                    source = mapping[input_key]
                    self.set_remapping_for(source, source)

    ##
    # Saves child state machine to path given by base_path
    #
    def save_child(self, base_path=""):
        child_gm = self.get_child()
        fname = pt.join(base_path, child_gm.get_document().get_name())
        if not pt.exists(fname):
            child_gm.save(fname)
        else:
            raise RuntimeError('FILE EXISTS. CAN\'T OVERWRITE')

        self.child_document = child_gm.document

    ##
    # Loads the child state machine and calls recreate on it.
    #
    # @param base_path Base path of parent state machine (children state
    #   machines are stored as subfolders).
    def load_and_recreate(self, base_path):
        import graph_model as gm

        fname = pt.join(base_path, pt.split(self.child_document.get_filename())[1])
        child_gm = gm.GraphModel.load(fname)
        return self.recreate(child_gm)

    ## Returns the name of the child state machine.
    def get_child_name(self):
        return self.child_gm.get_start_state()

    ## After being loaded from disk, recreates/clones the object to 
    # reestablish things that can't be stored on disk (like network
    # connections, file handlers, etc).
    #
    # @param graph_model A GraphModel object.
    def recreate(self, graph_model):
        raise RuntimeError('Unimplemented!!')


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

##
# Inherit from this base state class if all that's needed is a call to 
# an actionlib action.
#
class SimpleStateBase(StateBase):

    ##
    #
    # @param name Name of state
    # @param action_name Actionlib action name to connect to.
    # @param action_type Type of actionlib action to connect to.
    # @param goal_cb_str String with name of goal callback function in
    #                       inherited class.
    # @param input_keys Input keys of this state (in SMACH sense, a list of
    #               strings).
    # @param output_keys Ouput keys of this state (in SMACH sense, a list of
    #               strings).
    def __init__(self, name, action_name, action_type, goal_cb_str, input_keys=[], output_keys=[]):
        StateBase.__init__(self, name)
        self.action_name = action_name
        self.action_type = action_type
        self.goal_cb_str = goal_cb_str
        self.input_keys = input_keys
        self.output_keys = output_keys

    def get_smach_state(self):
        return SimpleStateBaseSmach(self.action_name, self.action_type, self, self.goal_cb_str,
                self.input_keys, self.output_keys)

## SMACH state to go with classes that inherits from SimpleStateBase.
class SimpleStateBaseSmach(smach_ros.SimpleActionState):

    ## Constructor
    # Params used here are the same as the ones used by SimpleState
    def __init__(self, action_name, action_type, goal_obj, goal_cb_str, input_keys, output_keys):
        smach_ros.SimpleActionState.__init__(self, action_name, action_type, 
                goal_cb = SimpleStateCB(eval('goal_obj.%s' % goal_cb_str), input_keys, output_keys))
        self.goal_obj = goal_obj

    ## Executes this state
    def __call__(self, userdata, default_goal): 
        f = eval('self.goal_obj.%s' % self.goal_cb_str)
        return f(userdata, default_goal)


## Smach state for executing actions that needs to TIME OUT (like navigation
# with nav stack).
class ActionWithTimeoutSmach(smach.State):

    ## Constructor
    #
    # @param time_out How long before we should time out
    # @param action_name Name of action.
    # @param action_class Class of action.
    def __init__(self, time_out, action_name, action_class):
        smach.State.__init__(self, 
                outcomes = ['succeeded', 'preempted', 'timed_out'], 
                input_keys = [], output_keys = [])
        self.action_client = actionlib.SimpleActionClient(action_name, action_class)
        self.action_name = action_name
        self.time_out = time_out

    ##
    # Returns the goal to execute by the action client.
    def get_goal(self):
        pass

    ## Called back to process the action's results.
    def process_result(self):
        pass 

    ## Execute the node (SMACH function)
    def execute(self, userdata):
        self.action_client.send_goal(self.get_goal())
        result = monitor_goals(self, [self.action_client], self.action_name, self.time_out)
        if result == 'succeeded':
            self.process_result(self.action_client.get_result())
        return result

## Monitors an executing goal on given client, cancels the goal after
# timeout period. Returns one of failed, preempted, timed_out, or succeeded.
def monitor_goals(self, clients, name, timeout):
    r = rospy.Rate(30)
    status = 'failed'

    states = [c.get_state() for c in clients]
    start_time = rospy.get_time()
    while not rospy.is_shutdown():
        #we have been preempted
        if self.preempt_requested():
            rospy.loginfo(name + ': preempt requested')
            [c.cancel_goal() for c in clients]
            self.service_preempt()
            status = 'preempted'
            break
    
        if (rospy.get_time() - start_time) > timeout:
            [c.cancel_goal() for c in clients]
            rospy.loginfo(name + ': timed out!')
            status = 'timed_out'
            break
    
        #print tu.goal_status_to_string(state)
        stopped_states = [s not in [am.GoalStatus.ACTIVE, am.GoalStatus.PENDING] for s in states]
        if np.all(stopped_states):
            #succeed if all states succeeded
            if np.all([s == am.GoalStatus.SUCCEEDED for s in states]):
                rospy.loginfo(name + ': Succeeded!')
                status = 'succeeded'

            #failed if any state failed
            break
    
        states = [c.get_state() for c in clients]
        r.sleep()

    return status

    







