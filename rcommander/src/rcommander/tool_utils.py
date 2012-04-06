import PyQt4.QtGui as qtg
import PyQt4.QtCore as qtc
#from PyQt4.QtGui import *
#from PyQt4.QtCore import *
import smach_ros
import functools as ft
import actionlib_msgs.msg as am
import os.path as pt
from tf_broadcast_server.srv import GetTransforms
import rospy
import actionlib
import smach


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

class ComboBox:

    def __init__(self):
        pass

    def create_box(self, pbox):
        self.combobox = qtg.QComboBox(pbox)

    ##
    # Set selected item
    #
    # @param item item to set (string)
    # @param create whether to create the item in this ComboBox if it isn't already there
    def set_text(self, item, create=True):
        idx = combobox_idx(self.combobox, item, create=create)
        if idx != -1:
            self.combobox.setCurrentIndex(idx)

    def text(self):
        return str(self.combobox.currentText())

class FrameBox(ComboBox):

    def __init__(self, frames_service=None):
        ComboBox.__init__(self)
        if frames_service == None:
            frames_service = rospy.ServiceProxy('get_transforms', GetTransforms)
        self.frames_service = frames_service

    def create_box(self, pbox):
        ComboBox.create_box(self, pbox)
        for f in self.frames_service().frames:
            self.combobox.addItem(f)
        self.setEnabled = self.combobox.setEnabled
        return self.combobox

def goal_status_to_string(status):
    return status_dict[status]

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

##
# Finds index of item in QComboBox
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


def double_spin_box(parent, minv, maxv, step):
    box = qtg.QDoubleSpinBox(parent)
    box.setMinimum(minv)
    box.setMaximum(maxv)
    box.setSingleStep(step)
    return box

def selected_radio_button(buttons_list):
    selected = None
    for r in buttons_list:
        if r.isChecked():
            selected = str(r.text())
    return selected

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

        container = qtg.QWidget(parent)
        container.setObjectName(container_name)
        
        disp = qtg.QLabel(container)
        disp.setObjectName(disp_name)
        disp.setText('%.2f %s' % (initial_value, units))

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
            disp.setText('%3.2f %s' % (cv, units))
        parent.connect(slider, qtc.SIGNAL('sliderMoved(int)'), ft.partial(slider_moved_cb, disp))

        self.container = container
        self.slider = slider
        self.disp = disp
        self.units = units

    def _slider_to_units(self, value):
        return ((value / 100.) * (self.max_value - self.min_value)) + self.min_value
        
    def _units_to_slider(self, value):
        return int(round(((value - self.min_value) / (self.max_value - self.min_value)) * 100.0))

    def value(self):
        return self._slider_to_units(self.slider.value())

    def set_value(self, value):
        self.slider.setValue(self._units_to_slider(value))
        self.disp.setText('%3.2f %s' % (value, self.units))


class ToolBase:

    def __init__(self, rcommander, name, button_name, smach_class):
        self.rcommander = rcommander
        #self.properties_box = self.rcommander.ui.properties_tab
        #self.connections_box = self.rcommander.ui.connections_tab

        self.name = name
        self.name_input = None
        self.loaded_node_name = None

        self.button = None
        self.button_name = button_name

        self.smach_class = smach_class
        self.outcome_inputs = {}
        #self.combo_box_cbs = {}
        self.counter = 0

        self.node_exists = False
        self.saved_state = None

    #def get_name(self):
    #    return self.name

    def create_button(self, container):
        self.button = create_tool_button(self.button_name, container)
        self.rcommander.connect(self.button, qtc.SIGNAL('clicked()'), self.activate_cb)
        return self.button

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

    def clear_saved_state(self):
        self.saved_state = None

    #Called by RCommander when user deselects this tool
    def deselect_tool(self):
        # Is the node we're displaying saved? has it been added??
        if self.node_exists:
            return
        else:
            #self.saved_state = self.create_node(unique=False)
            try:
                self.saved_state = self.new_node()
            except Exception, e:
                rospy.loginfo(str(e))

        #create_node called when rcommander runs nodes
        #                   when - adds nodes

        #node_cb is called when add node is called
        #save_cb

    #def get_outcomes(self):
    #    return self.new_node().get_registered_outcomes()

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
        self.rcommander.empty_container(self.rcommander.ui.connections_tab)

        #fill it back up
        self.fill_connections_box(self.rcommander.ui.connections_tab)

        #replace defaults with saved values
        self.name_input.setText(node_name)
        for k in outcome_choices:
            selected = outcome_choices[k]
            widget = self.outcome_inputs[k]
            widget.setCurrentIndex(widget.findText(selected))

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

        #if not issubclass(current_node.__class__, InfoStateBase):

        #for outcome in self.get_outcomes():
        if issubclass(current_node.__class__, EmptyState):
            return 

        registered_outcomes = list(current_node_smach.get_registered_outcomes())
        registered_outcomes.sort()
        for outcome in registered_outcomes: #self.get_outcomes():
            #Make a new combobox and add available choices to it
            input_box = qtg.QComboBox(pbox)
            nodes = self.rcommander.connectable_nodes(self.get_current_node_name(), outcome)

            nodes.sort()
            for n in nodes:
                input_box.addItem(n)

            #add to view 
            formlayout.addRow(outcome, input_box)
            #set outcome as default
            #TODO abstract this line out
            input_box.setCurrentIndex(input_box.findText(outcome))
            #store object
            self.outcome_inputs[outcome] = input_box

            #make callback
            def cb(outcome, new_index):
                new_outcome = str(self.outcome_inputs[outcome].currentText())
                self.rcommander.connection_changed(self.get_current_node_name(), outcome, new_outcome)

            outcome_cb = ft.partial(cb, outcome)
            self.rcommander.connect(input_box, qtc.SIGNAL('currentIndexChanged(int)'), outcome_cb)


    def set_loaded_node_name(self, name):
        self.loaded_node_name = name

    def get_current_node_name(self):
        return self.loaded_node_name 

    def create_node(self, unique=True):
        if unique:
            self.counter = self.counter + 1
        n = self.new_node(str(self.name_input.text()))
        return n

    def node_selected(self, node):
        self.node_exists = True
        #print 'node name', node.get_name()
        outcome_list = self.rcommander.current_children_of(node.get_name())
        #print 'outcome_list', outcome_list
        for outcome_name, connect_node in outcome_list:
            if not self.outcome_inputs.has_key(outcome_name):
                continue
            widget = self.outcome_inputs[outcome_name]
            widget.setCurrentIndex(widget.findText(connect_node))

        self.name_input.setText(node.get_name())
        self.set_loaded_node_name(node.get_name())
        self.loaded_node_name = node.get_name()
        self.set_node_properties(node)

    def get_smach_class(self):
        return self.smach_class

    ##
    # @param pbox a QT widget using a FormLayout that can be filled with
    #             appropriate controls for this tool
    def fill_property_box(self, pbox):
        pass

    ##
    # Called when user clicks Add
    #
    # @return a valid smach state derived from StateBase
    def new_node(self, name=None):
        pass

    ##
    # Called by parent when user selects a node created by this tool
    #
    # @param node a State object created by this tool
    def set_node_properties(self, node):
        pass

    ##
    # Resets the state of this GUI tool, sets all options to sensible defaults
    def reset(self):
        pass


#class StateBase:
#
#    def __init__(self, name):
#        self.name = name
#        #self.tool_name = None
#        #self.outcome_choices = []
#        self.remapping = {}
#        self.runnable = True
#
#    def is_runnable(self):
#        return self.runnable
#
#    def set_runnable(self, v):
#        self.runnable = v
#
#    def set_name(self, name):
#        self.name = name
#
#    def get_name(self):
#        return self.name
#
#    def get_registered_outcomes(self):
#        return []
#
#    def __getstate__(self):
#        #r = [self.name, self.tool_name, self.outcome_choices, self.remapping]
#        r = [self.name, self.remapping, self.runnable]
#        return r
#
#    def __setstate__(self, state):
#        #print 'state base', state
#        #print state
#        #name, tool, remapping, runnable = state
#        name, remapping, runnable = state
#        self.name = name
#        #self.tool_name = tool
#        #self.outcome_choices = choices
#        self.remapping = remapping
#        self.runnable = runnable
#        #print '>>>>', name, 'toolname set to', self.tool_name
#
#    def source_for(self, var_name):
#        return self.remapping[var_name]
#
#    def set_source_for(self, var, source):
#        self.remapping[var] = source


class StateBase:

    def __init__(self, name, outputs={}):
        self.name = name
        self.remapping = {}
        self.runnable = True
        self.outputs = outputs

    def output_names(self):
        return self.outputs.keys()

    def output_type(self, name):
        return self.outputs[name]
        
    def set_name(self, name):
        self.name = name

    def get_name(self):
        return self.name

    def remapping_for(self, var_name):
        return self.remapping[var_name]

    def set_remapping_for(self, var, remapped):
        self.remapping[var] = remapped

    def is_runnable(self):
        return self.runnable

    def set_runnable(self, v):
        self.runnable = v

    def get_smach_state(self):
        raise RuntimeError('Unimplemented method: get_smach_state')


class EmptyState(StateBase):

    TOOL_NAME = 'outcome'

    def __init__(self, name, temporary, tool_name=TOOL_NAME):
        StateBase.__init__(self, name)
        self.set_runnable(False)
        self.temporary = temporary
        self.tool_name = tool_name

    def get_smach_state(self):
        return self

#class InfoStateBase(StateBase):
#
#    GLOBAL_NAME = 'Global'
#
#    def __init__(self, name):
#        StateBase.__init__(self, name)
#        self.set_runnable(False)
#
#    def set_info(self, info):
#        raise RuntimeError('Unimplemented method. set_info')
#
#    def get_info(self):
#        raise RuntimeError('Unimplemented method. get_info')
#
#    def get_registered_outcomes(self):
#        return [InfoStateBase.GLOBAL_NAME]

class EmbeddableState(StateBase):

    def __init__(self, name, child_gm):
        StateBase.__init__(self, name)
        self.child_gm = child_gm
        self.child_document = None 

        #Look inside state machine and look for things with remaps
        self._init_child(child_gm)

    def get_child_document(self):
        return self.child_document

    def abort_child(self):
        fetus = self.child_gm 
        self.child_gm = None
        return fetus

    def get_child(self):
        return self.child_gm

    def set_child(self, child):
        self.child_gm = child

    ##
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

    def save_child(self, base_path=""):
        child_gm = self.get_child()
        if child_gm.document.has_real_filename():
            #print 'saving child to', child_gm.get_document().get_filename() 
            child_gm.save(child_gm.get_document().get_filename())
        else:
            fname = pt.join(base_path, child_gm.get_document().get_name())
            #If child does not exists
            if not pt.exists(fname):
                child_gm.save(fname)
                #print 'saving child to', fname
            else:
                raise RuntimeError('FILE EXISTS. CAN\'T OVERWRITE')

        self.child_document = child_gm.document

    def load_and_recreate(self, base_path):
        import graph_model as gm

        fname = pt.join(base_path, pt.split(self.child_document.get_filename())[1])
        #print 'load_and_recreate:', fname
        child_gm = gm.GraphModel.load(fname)
        return self.recreate(child_gm)

    def get_child_name(self):
        return self.child_gm.get_start_state()

    def recreate(self, graph_model):
        raise RuntimeError('Unimplemented!!')

#    def __getstate__(self):
#        state = StateBase.__getstate__(self)
#        my_state = [self.document]
#        return {'state_base': state, 'self': my_state}
#
#    def __setstate__(self, state):
#        StateBase.__setstate__(self, state['state_base'])
#        self.document = state['self'][0]
#        self.child_gm = None


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


class SimpleStateBase(StateBase):

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


class SimpleStateBaseSmach(smach_ros.SimpleActionState):

    def __init__(self, action_name, action_type, goal_obj, goal_cb_str, input_keys, output_keys):
        smach_ros.SimpleActionState.__init__(self, action_name, action_type, 
                goal_cb = SimpleStateCB(eval('goal_obj.%s' % goal_cb_str), input_keys, output_keys))
        self.goal_obj = goal_obj

    def __call__(self, userdata, default_goal): 
        f = eval('self.goal_obj.%s' % self.goal_cb_str)
        return f(userdata, default_goal)


class ActionWithTimeoutSmach(smach.State):

    def __init__(self, time_out, action_name, action_class):
        smach.State.__init__(self, 
                outcomes = ['succeeded', 'preempted', 'timed_out'], 
                input_keys = [], output_keys = [])
        self.action_client = actionlib.SimpleActionClient(action_name, action_class)
        self.action_name = action_name
        self.time_out = time_out

    def get_goal(self):
        pass

    def process_result(self):
        pass

    def execute(self, userdata):
        self.action_client.send_goal(self.get_goal())
        state = self.action_client.get_state()

        succeeded = False
        preempted = False
        r = rospy.Rate(30)
        start_time = rospy.get_time()
        while True:
            #we have been preempted
            if self.preempt_requested():
                rospy.loginfo('%s: preempt requested' % self.action_name)
                self.action_client.cancel_goal()
                self.service_preempt()
                preempted = True
                break

            if (rospy.get_time() - start_time) > self.time_out:
                self.action_client.cancel_goal()
                rospy.loginfo('%s: timed out!' % self.action_name)
                succeeded = False
                break

            #print tu.goal_status_to_string(state)
            if (state not in [am.GoalStatus.ACTIVE, am.GoalStatus.PENDING]):
                if state == am.GoalStatus.SUCCEEDED:
                    rospy.loginfo('%s: Succeeded!' % self.action_name)
                    succeeded = True
                break

            state = self.action_client.get_state()
            r.sleep()

        if preempted:
            return 'preempted'

        if succeeded:
            self.process_result(self.action_client.get_result())
            return 'succeeded'

        return 'timed_out'



