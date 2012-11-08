import roslib; roslib.load_manifest('rcommander')

import PyQt4.QtGui as qtg
import PyQt4.QtCore as qtc
from rcommander_auto import Ui_RCommanderWindow

import rospy

import sy
import os.path as pt
import time
import signal
import smach

#NodeBox 
import graph

import graph_view as gv
import nodebox_gui as nbg
import graph_model as gm
import outcome_tool as ot
import library_tool as lb
import trigger_tool as tt
import tool_utils as tu

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

## Given a number x and percentage y, 
# returns a list of two numbers: [x*y, x-x*y]
def split(num, factor):
    num1 = int(round(num * factor))
    num2 = num - num1
    return [num1, num2]

## Represents element of a finite state machine stack.
class FSMStackElement:a

    def __init__(self, model, view, node):
        self.model = model
        self.view = view
        self.graph_node = None
        self.node = node

## ROS Commander main window, inherits from NodeBoxGUI code
class RCommander(qtg.QMainWindow, nbg.NodeBoxGUI):

    ## Constructor
    # @param app QApplication object.
    # @param width Width of window.
    # @param height Height of window.
    def __init__(self, app, width, height): #, robot, tf_listener=None):
        qtg.QMainWindow.__init__(self)
        self.size = [width, height]
        self.app = app
        self.ui = Ui_RCommanderWindow()
        self.ui.setupUi(self)
        self.resize(width, height)
        nbg.NodeBoxGUI.__init__(self, self.ui.graphicsSuperView)

        self.connect(self.ui.run_button,         
                qtc.SIGNAL('clicked()'), self.try_cb)
        self.connect(self.ui.add_button,         
                qtc.SIGNAL('clicked()'), self.add_cb)
        self.connect(self.ui.reset_button,       
                qtc.SIGNAL('clicked()'), self.reset_cb)
        self.connect(self.ui.save_button,        
                qtc.SIGNAL('clicked()'), self.save_cb)
        self.connect(self.ui.start_state_button, 
                qtc.SIGNAL('clicked()'), self.start_state_cb)
        self.connect(self.ui.add_to_library_button, 
                qtc.SIGNAL('clicked()'), self.add_to_library_cb)

        self.connect(self.ui.delete_button, 
                qtc.SIGNAL('clicked()'), self.delete_cb)
        self.connect(self.ui.action_Run, 
                qtc.SIGNAL('triggered(bool)'), self.run_sm_cb)
        self.connect(self.ui.action_stop, 
                qtc.SIGNAL('triggered(bool)'), self.stop_sm_cb)
        self.connect(self.ui.actionNew, 
                qtc.SIGNAL('triggered(bool)'), self.new_sm_cb)
        self.connect(self.ui.action_save, 
                qtc.SIGNAL('triggered(bool)'), self.save_sm_cb)
        self.connect(self.ui.action_save_as, 
                qtc.SIGNAL('triggered(bool)'), self.save_as_sm_cb)
        self.connect(self.ui.action_open, 
                qtc.SIGNAL('triggered(bool)'), self.open_sm_cb)
        self.connect(self.ui.action_quit, 
                qtc.SIGNAL('triggered(bool)'), self.quit_cb)
        self.ui.splitter.setSizes(split(self.width(), .83))

        empty_container(self.ui.properties_tab)
        empty_container(self.ui.connections_tab)
        self.add_mode()
        self.disable_buttons()

        # Tabs at the top.
        self.tabs = {}

        # Maps from class type to plugin instance.
        self.tool_dict = {}

        # Status vars for selection
        self.selected_tool = None
        self.selected_graph_tool = None 
        self.selected_node = None
        self.selected_edge = None
        self.fsm_stack = []

        # Status bar at the bottom of window.
        self.status_bar_msg = ''
        self.status_bar_exception = None

    ## Set the robot object that the tools in this ROS Commander instance
    # operates.
    def set_robot(self, robot, tf_listener):
        self.robot = robot
        self.tf_listener = tf_listener

    ##########################################################################
    # Methods for GUI logic
    ##########################################################################

    ## Create tab at top of window
    # @param tab_name A string with the tab's name.
    def _create_tab(self, tab_name):
        ntab = qtg.QWidget()
        ntab.setObjectName(tab_name)
        qtg.QHBoxLayout(ntab)
        self.ui.tools_box.addTab(ntab, tab_name)
        self.ui.tools_box.setTabText(self.ui.tools_box.indexOf(ntab), tab_name)
        self.tabs[tab_name] = ntab

    ## Add tools to tabs at top of window
    # Should only be called once during initialization
    # @param list of [tab-name, tool-object] pairs
    def add_tools(self, tools_list):
        #Add tools to the right tab, creating tabs if needed
        self.button_group_tab = qtg.QButtonGroup()

        for tab_name, tool in tools_list:
            if not self.tabs.has_key(tab_name):
                self._create_tab(tab_name)
            tab_widget = self.tabs[tab_name]
            self.button_group_tab.addButton(tool.create_button(tab_widget))
            self.tool_dict[tool.get_smach_class()] = {'tool_obj': tool}

        for tname in self.tabs.keys():
            self.tabs[tname].update()

        #Outcome tool is a specialized built in tool, add it here.
        self.button_group_tab.addButton(self.ui.add_outcome_button)
        outcome_tool = ot.OutcomeTool(self.ui.add_outcome_button, self)
        self.tool_dict[outcome_tool.get_smach_class()] = 
                    {'tool_obj': outcome_tool}

        self.button_group_tab.addButton(self.ui.library_button)
        library_tool = lb.LibraryTool(self.ui.library_button, self)
        self.tool_dict[library_tool.get_smach_class()] = 
                    {'tool_obj': library_tool}


    def set_selected_tool(self, tool_class):
        self.selected_tool = tool_class

    def get_selected_tool(self):
        return self.selected_tool

    ## Run a SMACH state machine.
    # @param sm SMACH state machine to run.
    # @param graph_model A GraphModel instance to manage run, 
    #       used for visualization purposes.
    def run_state_machine(self, sm, graph_model):
        if self.graph_model.is_running():
            raise RuntimeError('Only state machine execution thread '+
                    'maybe be active at a time.')

        graph_model.register_status_cb(self._state_machine_status_cb)
        cur_sm_thread = graph_model.run(graph_model.document.get_name(),
                state_machine=sm)

    ## Updates the status bar in response to events in state machine executions
    # @param message A message from the execution thread.
    # @param exception Exceptions thrown while executing.
    def _state_machine_status_cb(self, message, exception=None):
        self.status_bar_msg = message
        self.status_bar_exception = exception

    ## Checks whether the current state machine needs to be saved. Pops
    # up dialog boxes if so.
    def check_current_document(self):
        if self.graph_model.document.modified:
            msg_box = qtg.QMessageBox()
            msg_box.setText('Current state machine has not been saved.')
            msg_box.setInformativeText('Do you want to save it first?')
            msg_box.setStandardButtons( qtg.QMessageBox.Yes |\
                    qtg.QMessageBox.No | qtg.QMessageBox.Cancel)
            msg_box.setDefaultButton(qtg.QMessageBox.Cancel)
            ret = msg_box.exec_()

            if ret == qtg.QMessageBox.Cancel:
                return False

            elif ret == qtg.QMessageBox.Yes:
                return self.save_sm_cb()

        return True

    ## Disables buttons in properties box.
    def disable_buttons(self):
        self.ui.run_button.setDisabled(True)
        self.ui.reset_button.setDisabled(True)
        self.ui.add_button.setDisabled(True)
        self.ui.save_button.setDisabled(True)

    ## Enables buttons in properties box.
    def enable_buttons(self):
        self.ui.run_button.setDisabled(False)
        self.ui.reset_button.setDisabled(False)
        self.ui.add_button.setDisabled(False)
        self.ui.save_button.setDisabled(False)

    ## Tell the currently selected button (through a callback)
    # that it has been deselected.
    def notify_deselected_button(self):
        selected_tool_class = self.get_selected_tool()
        if selected_tool_class != None:
            tool = self.tool_dict[selected_tool_class]['tool_obj']
            tool.deselect_tool()

    ## Deselects current selected buttons in tabbed area at the top of window.
    def deselect_tool_buttons(self):
        self.notify_deselected_button()
        self.button_group_tab.setExclusive(False)
        button = self.button_group_tab.checkedButton()
        #print button
        if button != None:
            button.setDown(False)
            button.setChecked(False)
        self.button_group_tab.setExclusive(True)

    ## Set properties box to edit mode.
    def edit_mode(self):
        self.ui.add_button.hide()
        self.ui.save_button.show()

    ## Set properties box to add mode.
    def add_mode(self):
        self.ui.add_button.show()
        self.ui.save_button.hide()

    ## Remove everything from properties box.  
    # Used when transitioning between tools.
    def empty_properties_box(self):
        empty_container(self.ui.properties_tab)
        empty_container(self.ui.connections_tab)
        empty_container(self.ui.properties_container)

        # Restore property tab's vbox
        container = self.ui.properties_container
        # Remove current layout and items
        cl = container.layout()
        cl.deleteLater()
        qtc.QCoreApplication.sendPostedEvents(cl, qtc.QEvent.DeferredDelete)
        # Add in a new layout
        clayout = qtg.QVBoxLayout(container)
        clayout.setMargin(0)
        # Add in a container widget
        self.ui.properties_tab = qtg.QWidget(container)
        pbox_layout = qtg.QFormLayout(self.ui.properties_tab)
        clayout.addWidget(self.ui.properties_tab)
        spacer = qtg.QSpacerItem(20, 40, qtg.QSizePolicy.Minimum, 
                qtg.QSizePolicy.Expanding)
        clayout.addItem(spacer)


    ###########################################################################
    # Graph tools
    ###########################################################################

    ## Gives SMACH node that has a "set_robot" pointer to robot object.
    # @param node SMACH node to connect.
    def connect_node(self, node):
        if hasattr(node, 'set_robot'): 
            node.set_robot(self.robot)

    ## Called by tool object when connection of a node changes.
    # @param node_name The node name (a string).
    # @param outcome_name The outcome's name (a string).
    # @param new_outcome The new node to connect outcome to (a string).
    def connection_changed(self, node_name, outcome_name, new_outcome):
        self.graph_model.connection_changed(node_name, outcome_name, 
                new_outcome)
        self.graph_model.document.modified = True

    ## Called by tool object to check the current children of a node.
    # @param node_name A string.
    def current_children_of(self, node_name):
        return self.graph_model.current_children_of(node_name)

    ## Called by tool object to check nodes that can connect to a given node.
    # @param node_name Name of node to check (string).
    # @param outcome outcome Outcome type of that node (string).
    def connectable_nodes(self, node_name, outcome):
        return self.graph_model.connectable_nodes(node_name, outcome)

    ## Searches for outputs (SMACH 'outputs') of a given class type.
    # @param class_filter A Python class object.
    def outputs_of_type(self, class_filter):
        return self.graph_model.outputs_of_type(class_filter)

    ## Set node as selected in interface (colors it blue).
    # @param name Node's name (string).
    def set_selected_node(self, name):
        self.selected_node = name

    ## Set an edge as being selected. Currently unused.
    # @param n1 Node name (string).
    # @param n2 Node name (string).
    # @param label Label on edge / outcome name (string).
    def set_selected_edge(self, n1, n2, label):
        if n1 == None:
            self.selected_edge = None
        else:
            self.selected_edge = self.graph_model.edge(n1, n2, label=label)

    ## Check to see if graph has node of a given name
    # @param node_name A string.
    def has_node_name(self, node_name):
        return self.graph_model.has_node_name(node_name)

    ###########################################################################
    # Callbacks
    ###########################################################################

    ## Tools (from tool_utils.ToolBase) notifies ROS Commander that a new 
    #button has been activated
    def notify_activated(self):
        self.save_cb()
        self.notify_deselected_button()

    ## Callback from the big Try button in properties box.
    # Creates a state machine with a single state and runs it.
    def try_cb(self):
        if self.selected_tool == None:
            return
        try:
            tool_instance = self.tool_dict[self.selected_tool]['tool_obj']
            node = tool_instance.create_node(unique=False)
            if node == None:
                qtg.QMessageBox.information(self, str("Run Error"), 
                        'Unable to create node to run.')
                return
            singleton_sm, graph_model = \
            self.graph_model.create_singleton_statemachine(node, self.robot)
            self.run_state_machine(singleton_sm, graph_model)
        except RuntimeError, e:
            qtg.QMessageBox.information(self, str(self.objectName()),\
                    'RuntimeError: ' + e.message)

    ## Callback for the add button.
    # Adds state machine being edited to GUI.
    def add_cb(self):
        if self.selected_tool == None:
            return

        selected_tool = self.selected_tool

        tool_instance = self.tool_dict[selected_tool]['tool_obj']
        if hasattr(tool_instance, 'set_child_node'):
            if self.selected_node == None:
                qtg.QMessageBox.information(self, str(self.objectName()), 
                        'Need to have another node selected to create an'+\
                                ' instance of this node.')
                return
            else:
                state = self.graph_model.get_state(self.selected_node)
                tool_instance.set_child_node(state)

        try:
            node = tool_instance.create_node()
            if node == None:
                rospy.loginfo('For some reason node wasn\'t created')
                return
            self.graph_model.add_node(node)
        except RuntimeError, e:
            qtg.QMessageBox.information(self, str(self.objectName()), 
                    'RuntimeError: ' + e.message)
            return 

        if self.selected_node == None:
            self.node_cb(self.graph_model.node(node.name))
        else:
            snode = self.graph_model.node(self.selected_node)
            if snode != None:
                self.node_cb(snode)
            else:
                self.nothing_cb(None)

        self.tool_dict[selected_tool]['tool_obj'].refresh_connections_box()
        self.graph_view.refresh()
        self.graph_model.document.modified = True
        tool_instance.clear_saved_state()

    ## Callback for reset button in properties box.
    def reset_cb(self):
        if self.selected_tool == None:
            return
        tool_instance = self.tool_dict[self.selected_tool]['tool_obj']
        tool_instance.reset()
        
    ## Callback for the save button in properties box.
    def save_cb(self):
        if self.selected_tool == None:
            rospy.logdebug('No selected tool. Not saving')
            return

        tool_instance = self.tool_dict[self.selected_tool]['tool_obj']
        old_node_name = tool_instance.get_current_node_name()

        # create a node with new settings
        try:
            if not self.graph_model.has_node_name(old_node_name):
                rospy.logdebug('Does not have node named %s in the graph already. not saving.' % old_node_name)
                return

            if self.graph_model.get_state(old_node_name).__class__ == \
                    tu.EmptyState:
                rospy.logdebug('Not saving outcome state.')
                return

            node = tool_instance.create_node(unique=False)
            if node == None:
                qtg.QMessageBox.information(self, str("Save Error"), 
                        'Unable to create state for saving. Do you have enough keyframes?')
                return
            self.graph_model.replace_node(node, old_node_name)
            self.graph_model.document.modified = True

        except IOError, e:
            return

        except RuntimeError, e:
            qtg.QMessageBox.information(self, str(self.objectName()), 
                    'RuntimeError: ' + e.message)
            return 

    ## Callback for start state button, sets currently selected node as the
    # start state.
    def start_state_cb(self):
        if self.selected_node != None:
            try:
                self.graph_model.set_start_state(self.selected_node)
            except RuntimeError, e:
                qtg.QMessageBox.information(self, str(self.objectName()), 
                        'RuntimeError: ' + e.message)

    ## Callback for add node to library button.
    def add_to_library_cb(self):
        if self.selected_node != None:
            self.tool_dict['library']['tool_obj'].add_to_library(\
                    self.graph_model.get_state(self.selected_node))

    ## Callback for delete button. Deletes a state.
    def delete_cb(self):
        if self.selected_node != None:
            if self.selected_node != 'start':
                self.graph_model.delete_node(self.selected_node)
                self.set_selected_node(None)
                self.graph_view.refresh()

        self.graph_model.document.modified = True
        self.nothing_cb(None)

    ## Callback for running the entire state machine.
    def run_sm_cb(self, checked):
        if self.graph_model.get_start_state() == None:
            qtg.QMessageBox.information(self, str(self.objectName()), \
		        'No start state set.  Select a state and click on' +\
                ' \'Start State\' to set a new start state.')
        else:
            try:
                self.run_state_machine(
                        self.graph_model.create_state_machine(self.robot), 
                        self.graph_model)
            except RuntimeError, e:
                qtg.QMessageBox.information(self, str(self.objectName()), 
                        'RuntimeError: ' + e.message)

    ## Callback for stopping the currently running state machine.
    def stop_sm_cb(self):
        self.graph_model.preempt()

    ## Callback for create a new state machine menu item.
    def new_sm_cb(self):
        #Prompt user to save if document has been modifid
        if not self.check_current_document():
            return

        self._set_model(gm.GraphModel())
        self.nothing_cb(None)

    ## Callback for saving the current state machine.
    def save_sm_cb(self):
        self.save_cb()
        if self.graph_model.document.has_real_filename():
            self.graph_model.save(self.graph_model.document.get_filename())
            return True
        else:
            return self.save_as_sm_cb()

    ## Callback for save as menu item.  Saves current state machine.
    def save_as_sm_cb(self):
        self.save_cb()
        self.stop_drawing()
        filename = str(qtg.QFileDialog.getSaveFileName(self, 'Save As', 
            self.graph_model.document.get_filename()))
        self.start_drawing()

        #user canceled
        if len(filename) == 0:
            return False

        if pt.exists(filename):
            #Ask if want to over write
            msg_box = qtg.QMessageBox()
            msg_box.setText('There is already a file with this name.')
            msg_box.setInformativeText('Do you want to overwrite it?')
            msg_box.setStandardButtons(qtg.QMessageBox.Yes |\
                    qtg.QMessageBox.No | qtg.QMessageBox.Cancel)
            msg_box.setDefaultButton(qtg.QMessageBox.Cancel)
            ret = msg_box.exec_()
            if ret == qtg.QMessageBox.No or ret == qtg.QMessageBox.Cancel:
                return False

        self.graph_model.save(filename)
        self.graph_model.document.set_filename(filename)
        self.graph_model.document.real_filename = True
        self.graph_model.document.modified = False
        return True

    ## Callback for opening a state machine on disk menu item.
    def open_sm_cb(self):
        #prompt user if current document has not been saved
        if not self.check_current_document():
            return

        dialog = qtg.QFileDialog(self, 'Open State Machine', '~')
        dialog.setFileMode(qtg.QFileDialog.Directory)
        dialog.setViewMode(qtg.QFileDialog.List)

        self.stop_drawing()
        if dialog.exec_():
            filenames = dialog.selectedFiles()
            filename = str(filenames[0])

            #Stop things that are currently running
            self.stop_sm_cb()

            #Set this a the new model
            self._set_model(gm.GraphModel.load(filename))

            #TODO check that the top level state machine has been saved
            self.fsm_stack = []

            #Reset state of GUI
            self.nothing_cb(None)
            self._state_machine_status_cb(' ')
        self.start_drawing()


    ###########################################################################
    # Graph callbacks:
    #   called back from Nodebox's graph drawing library, and GraphModel.
    ###########################################################################

    ## Callback for when a click happens but it occurs in an area without any
    # selectable objects.
    # @param pt A 2D point.
    def nothing_cb(self, pt):
        self.save_cb()
        self.deselect_tool_buttons()
        self.set_selected_tool(None)
        self.set_selected_node(None)
        self.set_selected_edge(None, None, None)
        self.empty_properties_box()
        self.add_mode()
        self.disable_buttons()

    ## Callback for when a node has been selected.
    # @param node A nodebox graph node.
    def node_cb(self, node):
        self.save_cb()
        self.deselect_tool_buttons()

        self.set_selected_node(node.id)
        self.set_selected_edge(None, None, None)
        state = self.graph_model.get_state(node.id)

        tool = self.tool_dict[state.__class__]['tool_obj']
        tool.button.setChecked(True)
        tool.activate_cb(state.get_name())
        self.set_selected_tool(state.__class__)

        self.edit_mode()
        self.enable_buttons()
        tool.node_selected(state)

        if state.is_runnable():
            self.ui.run_button.setDisabled(False)
        else:
            self.ui.run_button.setDisabled(True)

    ## Callback for when an edge has been clicked on.
    # @param edge A nodebox graph edge.
    def edge_cb(self, edge):
        self.set_selected_edge(edge.node1.id, edge.node2.id, edge.label)
        self.set_selected_node(None)
        self.disable_buttons()

    ## Handler for double clicking on a node, descending a level.
    # @param node A nodebox node that has been double clicked.
    def dclick_cb(self, node):
        snode = self.graph_model.get_state(node.id)
        if gm.is_container(snode):
            self.fsm_stack.append(FSMStackElement(self.graph_model, 
                self.graph_view, snode))
            self._set_model(snode.get_child())
            self.nothing_cb(None)

    ##Handler for double clicking on circle, ascending a level.
    # @param fsm_stack_element A FSMStackElement object double clicked on.
    def dclick_container_cb(self, fsm_stack_element):

        #Store current model
        last_fsm_el = self.fsm_stack[-1]

        # Recreate replaces the old subtree with the current one being edited.
        new_node = last_fsm_el.node.recreate(self.graph_model)
        
        #replace old node in the graph, reserving links which exist
        last_fsm_el.model.replace_node(new_node, last_fsm_el.node.get_name())

        #Shorten the stack to the element selected
        self.fsm_stack = self.fsm_stack[:self.fsm_stack.index(fsm_stack_element)]

        #Load the element we're given
        self._set_model(fsm_stack_element.model, view=fsm_stack_element.view)
        self.nothing_cb(None)

    ## Sets the GraphModel being edited.
    # @param model GraphModel being edited.
    # @param view Optional GraphView object.
    def _set_model(self, model, view=None):
        self.graph_model = model
        if view == None:
            self.graph_view = gv.GraphView(self.context, self.graph_model)
            self.graph_view.setup()
        else:
            self.graph_view = view

        self.graph_model.gve.events.click = self.node_cb
        self.graph_model.gve.events.click_edge = self.edge_cb
        self.graph_model.gve.events.click_nothing = self.nothing_cb
        self.graph_model.gve.events.dclick = self.dclick_cb
        self.graph_view.fsm_dclick_cb = self.dclick_container_cb

    ###########################################################################
    # Links to drawing code
    ###########################################################################

    ## Setups nodebox drawing environment.
    def setup(self):
        graph._ctx = self.context
        self.context.speed(30.)
        self.context.size(self.size[0], self.size[1])
        self._set_model(gm.GraphModel())

    ## Called every cycle to redraw view.
    # @param properties_dictionary of properties for the GraphView object to view. 
    # Its model is RCommander's state variables and the current GraphModel.
    def draw(self, properties_dict):
        properties_dict['selected_edge'] = self.selected_edge
        properties_dict['selected_node'] = self.selected_node
        properties_dict['width'] = self.ui.graphicsSuperView.viewport().width()
        properties_dict['height'] = self.ui.graphicsSuperView.viewport().height()
        properties_dict['name'] = self.graph_model.document.get_name()
        properties_dict['fsm_stack'] = self.fsm_stack
        self.graph_view.draw(properties_dict)

        if str(self.statusBar().currentMessage()) != self.status_bar_msg:
            self.statusBar().showMessage(self.status_bar_msg)

        if self.status_bar_exception != None:
            e = self.status_bar_exception
            if e.__class__ == smach.InvalidStateError or\
                e.__class__ == smach.InvalidTransitionError or\
                e.__class__ == smach.InvalidUserCodeError:

                if str(e.message).find('TaskFrameError') != -1:
                    qtg.QMessageBox.information(self, str(self.objectName()), 
                            'Unable to transform using task frame.'+
                            ' Is your intended task frame unselected? Make sure the frame is selected (colored red) by clicking on it.')
                elif str(e.message).find('FrameError') != -1:
                    qtg.QMessageBox.information(self, str(self.objectName()), 
                            'Frame Error: unable to relate frames.')
                else:
                    qtg.QMessageBox.information(self, str(self.objectName()), 
                            '%s: %s' % (str(e.__class__), str(e.message)))
            else:
                qtg.QMessageBox.information(self, str(self.objectName()), 
                        '%s: %s' % (str(e.__class__), str(e)))

            self.status_bar_exception = None

    def quit_cb(self):
        self.stop_drawing()
        rospy.signal_shutdown('User closed window.')
        self.app.quit()

## Instantiates ROS Commander and runs it.
# @param plugin_namespace List of plugin categories to use (list of strings).
# @param robot Robot object needed by plugins loaded.
# @param tf_listener TF listener object.
# @param width Width of default window to spawn.
# @param height Height of default window to spawn.
def run_rcommander(plugin_namespace, robot=None, tf_listener=None, 
        width=600, height=600):
    import plugins 
    import state_machine_tool as smt
    import sleep_tool as st
    import pointcloud_click_tool as ptl
    import freeze_frame_tool as frz

    app = qtg.QApplication(sys.argv)
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    rc = RCommander(app, width, height)
    app.connect(app, qtc.SIGNAL('lastWindowClosed()'), app.quit)
    app.connect(rc.ui.action_quit, qtc.SIGNAL('clicked()'), app.quit)
    rc.set_robot(robot, tf_listener)

    default_tools = [['Origins', ptl.PointCloudClickTool(rc), 'default_frame'], 
                     ['Origins', frz.FreezeFrameTool(rc),     'default_frame'],
                     ['Misc',    smt.StateMachineTool(rc),    'default'], 
                     ['Misc',    st.SleepTool(rc),            'default'],
                     ['Misc',    tt.TriggerTool(rc),          'default']]
    tools_list = []
    for n,t,ns in default_tools:
        if ns in plugin_namespace:
            tools_list.append([n,t])

    plugin_clses = plugins.load_plugins(plugin_namespace)
    for tab_name, pcls in plugin_clses:
        tools_list.append([tab_name, pcls(rc)])
    rc.add_tools(tools_list)

    rc.show()
    sys.exit(app.#exec_())
