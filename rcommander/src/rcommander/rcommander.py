import roslib; roslib.load_manifest('rcommander')

import PyQt4.QtGui as qtg
import PyQt4.QtCore as qtc
from rcommander_auto import Ui_RCommanderWindow

import rospy
#import tf

import sys
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
#import pr2_utils as pu

def split(num, factor):
    num1 = int(round(num * factor))
    num2 = num - num1
    return [num1, num2]

class FSMStackElement:

    def __init__(self, model, view, node):
        self.model = model
        self.view = view
        self.graph_node = None
        self.node = node

class RCommander(qtg.QMainWindow, nbg.NodeBoxGUI):

    def __init__(self, app): #, robot, tf_listener=None):
        qtg.QMainWindow.__init__(self)
        self.app = app
        self.ui = Ui_RCommanderWindow()
        self.ui.setupUi(self)
        nbg.NodeBoxGUI.__init__(self, self.ui.graphicsSuperView)

        self.connect(self.ui.run_button,         qtc.SIGNAL('clicked()'), self.run_cb)
        self.connect(self.ui.add_button,         qtc.SIGNAL('clicked()'), self.add_cb)
        self.connect(self.ui.reset_button,       qtc.SIGNAL('clicked()'), self.reset_cb)
        self.connect(self.ui.save_button,        qtc.SIGNAL('clicked()'), self.save_cb)
        self.connect(self.ui.start_state_button, qtc.SIGNAL('clicked()'), self.start_state_cb)
        self.connect(self.ui.add_to_library_button, qtc.SIGNAL('clicked()'), self.add_to_library_cb)

        self.connect(self.ui.delete_button, qtc.SIGNAL('clicked()'), self.delete_cb)
        self.connect(self.ui.action_Run, qtc.SIGNAL('triggered(bool)'), self.run_sm_cb)
        self.connect(self.ui.action_stop, qtc.SIGNAL('triggered(bool)'), self.stop_sm_cb)
        self.connect(self.ui.actionNew, qtc.SIGNAL('triggered(bool)'), self.new_sm_cb)
        self.connect(self.ui.action_save, qtc.SIGNAL('triggered(bool)'), self.save_sm_cb)
        self.connect(self.ui.action_save_as, qtc.SIGNAL('triggered(bool)'), self.save_as_sm_cb)
        self.connect(self.ui.action_open, qtc.SIGNAL('triggered(bool)'), self.open_sm_cb)
        self.connect(self.ui.action_quit, qtc.SIGNAL('triggered(bool)'), self.quit_cb)
        self.ui.splitter.setSizes(split(self.width(), .83))

        self.empty_container(self.ui.properties_tab)
        self.empty_container(self.ui.connections_tab)
        self.add_mode()
        self.disable_buttons()

        #create instance variables
        self.tabs = {}
        self.tool_dict = {}

        #name of currently selected tool that operates on graph
        #self.graph_model = None
        self.selected_tool = None
        self.selected_graph_tool = None 
        self.selected_node = None
        self.selected_edge = None
        self.fsm_stack = []

        #Setup animation timer
        #self.status_bar_timer = QTimer()
        #self.connect(self.status_bar_timer, SIGNAL('timeout()'), self.status_bar_check)
        #self.status_bar_timer.start(100)
        self.status_bar_msg = ''
        self.status_bar_exception = None
        
        #Connect to ROS & PR2 
        #if tf_listener == None:
        #    tf_listener = tf.TransformListener()
        #self.tf_listener = tf_listener
        #self.robot = robot
        #self.pr2 = pu.PR2(self.tf_listener)

    def set_robot(self, robot, tf_listener):
        self.robot = robot
        self.tf_listener = tf_listener

    def status_bar_check(self):
        if self.graph_model.sm_thread.has_key('run_sm'):
            sm_thread = self.graph_model.sm_thread['run_sm']

            if sm_thread.exception != None:
                m = sm_thread.exception.message
                self.statusBar().showMessage('%s: %s' % (sm_thread.exception.__class__, m), 15000)
                self.graph_model.sm_thread.pop('run_sm')
                self.graph_model.sm_thread.pop('preempted')
                return

            if sm_thread.outcome != None:
                self.statusBar().showMessage('Finished with outcome: %s' % sm_thread.outcome, 15000)
                self.graph_model.sm_thread.pop('run_sm')
                self.graph_model.sm_thread.pop('preempted')
                return

            if not sm_thread.isAlive():
                self.statusBar().showMessage('Error: SM thread unexpectedly died.', 15000)
                self.graph_model.sm_thread.pop('run_sm')
                self.graph_model.sm_thread.pop('preempted')
                return

            if self.graph_model.sm_thread['preempted'] != None and (time.time() - self.graph_model.sm_thread['preempted'] > 5.):
                rospy.loginfo('Thread took too long to terminate.  Escallating and using exception exit.')
                self.graph_model.sm_thread['run_sm'].except_preempt()
                rospy.loginfo('Thread terminated.')
                self.graph_model.sm_thread.pop('run_sm')
                self.graph_model.sm_thread.pop('preempted')

            rstring = 'Running...'
            if str(self.statusBar().currentMessage()) != rstring:
                self.statusBar().showMessage(rstring, 1000)

    ####################################################################################################################
    # GUI logic
    ####################################################################################################################
    def _create_tab(self, tab_name):
        ntab = qtg.QWidget()
        ntab.setObjectName(tab_name)
        qtg.QHBoxLayout(ntab)
        self.ui.tools_box.addTab(ntab, tab_name)
        self.ui.tools_box.setTabText(self.ui.tools_box.indexOf(ntab), tab_name)
        self.tabs[tab_name] = ntab

    ##
    # Should only be called once during initialization
    #
    # @param list of [tab-name, tool-object] pairs
    def add_tools(self, tools_list):
        #add tools to the right tab, creating tabs if needed
        self.button_group_tab = qtg.QButtonGroup()
        #self.connect(self.button_group_tab, qtc.SIGNAL('clicked(int)'), self.button_group_clicked_cb)

        for tab_name, tool in tools_list:
            if not self.tabs.has_key(tab_name):
                self._create_tab(tab_name)
            tab_widget = self.tabs[tab_name]
            self.button_group_tab.addButton(tool.create_button(tab_widget))
            #self.tool_dict[tool.get_name()] = {'tool_obj': tool}
            self.tool_dict[tool.get_smach_class()] = {'tool_obj': tool}

        for tname in self.tabs.keys():
            self.tabs[tname].update()

        #self.tool_dict[outcome_tool.get_name()] = {'tool_obj': outcome_tool}
        #Outcome tool is a specialized built in tool
        self.button_group_tab.addButton(self.ui.add_outcome_button)
        outcome_tool = ot.OutcomeTool(self.ui.add_outcome_button, self)
        self.tool_dict[outcome_tool.get_smach_class()] = {'tool_obj': outcome_tool}

        self.button_group_tab.addButton(self.ui.library_button)
        library_tool = lb.LibraryTool(self.ui.library_button, self)
        self.tool_dict[library_tool.get_smach_class()] = {'tool_obj': library_tool}

    def empty_container(self, pbox): 
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

    def set_selected_tool(self, tool_name):
        self.selected_tool = tool_name

    def get_selected_tool(self):
        return self.selected_tool

    def run_state_machine(self, sm, graph_model):
        if self.graph_model.is_running():
            raise RuntimeError('Only state machine execution thread maybe be active at a time.')

        graph_model.register_status_cb(self._state_machine_status_cb)
        cur_sm_thread = graph_model.run(graph_model.document.get_name(), state_machine=sm)
        #self.statusBar().showMessage('Running state machine %s.' % graph_model.document.get_name())

    def _state_machine_status_cb(self, message, exception=None):
        self.status_bar_msg = message
        self.status_bar_exception = exception

    def check_current_document(self):
        if self.graph_model.document.modified:
            msg_box = qtg.QMessageBox()
            msg_box.setText('Current state machine has not been saved.')
            msg_box.setInformativeText('Do you want to save it first?')
            msg_box.setStandardButtons(qtg.QMessageBox.Yes | qtg.QMessageBox.No | qtg.QMessageBox.Cancel)
            msg_box.setDefaultButton(qtg.QMessageBox.Cancel)
            ret = msg_box.exec_()

            if ret == qtg.QMessageBox.Cancel:
                return False

            elif ret == qtg.QMessageBox.Yes:
                return self.save_sm_cb()

        return True

    def disable_buttons(self):
        self.ui.run_button.setDisabled(True)
        self.ui.reset_button.setDisabled(True)
        self.ui.add_button.setDisabled(True)
        self.ui.save_button.setDisabled(True)

    def enable_buttons(self):
        self.ui.run_button.setDisabled(False)
        self.ui.reset_button.setDisabled(False)
        self.ui.add_button.setDisabled(False)
        self.ui.save_button.setDisabled(False)


    def notify_deselected_button(self):
        selected_tool_class = self.get_selected_tool()
        if selected_tool_class != None:
            tool = self.tool_dict[selected_tool_class]['tool_obj']
            tool.deselect_tool()

    def deselect_tool_buttons(self):
        self.notify_deselected_button()
        self.button_group_tab.setExclusive(False)
        button = self.button_group_tab.checkedButton()
        #print button
        if button != None:
            button.setDown(False)
            button.setChecked(False)
        self.button_group_tab.setExclusive(True)

    def edit_mode(self):
        self.ui.add_button.hide()
        self.ui.save_button.show()

    def add_mode(self):
        self.ui.add_button.show()
        self.ui.save_button.hide()

    def empty_properties_box(self):
        self.empty_container(self.ui.properties_tab)
        self.empty_container(self.ui.connections_tab)
        self.empty_container(self.ui.properties_container)

        ##
        # Restore property tab's vbox
        ##
        container = self.ui.properties_container
        #Remove current layout and items
        cl = container.layout()
        cl.deleteLater()
        qtc.QCoreApplication.sendPostedEvents(cl, qtc.QEvent.DeferredDelete)
        #add in a new layout
        clayout = qtg.QVBoxLayout(container)
        clayout.setMargin(0)
        #add in a container widget
        self.ui.properties_tab = qtg.QWidget(container)
        pbox_layout = qtg.QFormLayout(self.ui.properties_tab)
        clayout.addWidget(self.ui.properties_tab)
        spacer = qtg.QSpacerItem(20, 40, qtg.QSizePolicy.Minimum, qtg.QSizePolicy.Expanding)
        clayout.addItem(spacer)


    ####################################################################################################################
    # Graph tools
    ####################################################################################################################
    #def _reconnect_states(self):
    #    for k in self.graph_model.states_dict:
    #        if hasattr(self.graph_model.get_state(k), 'set_robot'):
    #            self.graph_model.get_state(k).set_robot(self.robot)

    def connect_node(self, node):
        if hasattr(node, 'set_robot'): 
            node.set_robot(self.robot)

    def connection_changed(self, node_name, outcome_name, new_outcome):
        self.graph_model.connection_changed(node_name, outcome_name, new_outcome)
        #print 'connection_changed: State machine modified!'
        self.graph_model.document.modified = True

    def current_children_of(self, node_name):
        return self.graph_model.current_children_of(node_name)

    def connectable_nodes(self, node_name, outcome):
        return self.graph_model.connectable_nodes(node_name, outcome)

    def outputs_of_type(self, class_filter):
        return self.graph_model.outputs_of_type(class_filter)

    def set_selected_node(self, name):
        self.selected_node = name

    def set_selected_edge(self, n1, n2, label):
        if n1 == None:
            self.selected_edge = None
        else:
            self.selected_edge = self.graph_model.edge(n1, n2, label=label)

    ####################################################################################################################
    # All callbacks
    ####################################################################################################################
    def notify_activated(self):
        self.notify_deselected_button()

    def run_cb(self):
        if self.selected_tool == None:
            return
        try:
            tool_instance = self.tool_dict[self.selected_tool]['tool_obj']
            node = tool_instance.create_node(unique=False)
            singleton_sm, graph_model = self.graph_model.create_singleton_statemachine(node, self.robot)
            self.run_state_machine(singleton_sm, graph_model)
        except RuntimeError, e:
            qtg.QMessageBox.information(self, str(self.objectName()), 'RuntimeError: ' + e.message)
    
    def add_cb(self):
        if self.selected_tool == None:
            return

        #print 'selected tool ', self.selected_tool
        selected_tool = self.selected_tool

        tool_instance = self.tool_dict[selected_tool]['tool_obj']
        if hasattr(tool_instance, 'set_child_node'):
            if self.selected_node == None:
                qtg.QMessageBox.information(self, str(self.objectName()), 
                        'Need to have another node selected to create an instance of this node.')
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
                #self.selected_node = None

        self.tool_dict[selected_tool]['tool_obj'].refresh_connections_box()
        self.graph_view.refresh()
        self.graph_model.document.modified = True
        tool_instance.clear_saved_state()

    def reset_cb(self):
        if self.selected_tool == None:
            return
        tool_instance = self.tool_dict[self.selected_tool]['tool_obj']
        tool_instance.reset()
        
    def save_cb(self):
        tool_instance = self.tool_dict[self.selected_tool]['tool_obj']
        #old_smach_node = self.graph_model.get_smach_state()
        #print 'save cb called!!!'
        old_node_name = tool_instance.get_current_node_name()
        # create a node with new settings
        try:
            node = tool_instance.create_node(unique=False)
            self.graph_model.replace_node(node, old_node_name)
        except RuntimeError, e:
            qtg.QMessageBox.information(self, str(self.objectName()), 
                    'RuntimeError: ' + e.message)
            return 
        # 'delete' old smach node
        #print 'TRANS!', smach_node.vels
        #self.graph_model.set_smach_state(old_smach_node.get_name(), smach_node)

        # connection changes are made instantly (so don't worry about them)
        # only saving of internal node parameters must be implemented by client tools
        #print 'save_cb: State machine modified!'
        self.graph_model.document.modified = True

    def start_state_cb(self):
        if self.selected_node != None:
            try:
                self.graph_model.set_start_state(self.selected_node)
            except RuntimeError, e:
                qtg.QMessageBox.information(self, str(self.objectName()), 'RuntimeError: ' + e.message)

    def add_to_library_cb(self):
        if self.selected_node != None:
            self.tool_dict['library']['tool_obj'].add_to_library(self.graph_model.get_state(self.selected_node))

    def delete_cb(self):
        if self.selected_node != None:
            if self.selected_node != 'start':
                self.graph_model.delete_node(self.selected_node)
                self.set_selected_node(None)
                self.graph_view.refresh()
            else:
                print 'Can\'t delete start node!'

        #TODO rethink deleting of edges
        #if self.selected_edge != None:
        #    se = self.selected_edge
        #    self.set_selected_edge(None, None)
        #    self.graph_model.delete_edge(se)
        #    self.graph_view.refresh()
        print 'delete_cb: State machine modified!'
        self.graph_model.document.modified = True
        self.nothing_cb(None)

    def run_sm_cb(self, checked):
        #TODO Disable all buttons.
        #TODO Reflect state of running graph.
        if self.graph_model.get_start_state() == None:
            qtg.QMessageBox.information(self, str(self.objectName()), \
		'No start state set.  Select a state and click on \'Start State\' to set a new start state.')
        else:
            try:
                self.run_state_machine(self.graph_model.create_state_machine(self.robot), self.graph_model)
            except RuntimeError, e:
                qtg.QMessageBox.information(self, str(self.objectName()), 'RuntimeError: ' + e.message)

    def stop_sm_cb(self):
        self.graph_model.preempt()
        #if self.graph_model.is_running():
        #    self.graph_model.preempt()
        ##if self.graph_model.sm_thread.has_key('run_sm'):
        #    self.graph_model.sm_thread['run_sm'].preempt()
        #    self.graph_model.sm_thread['preempted'] = time.time()

    def new_sm_cb(self):
        #prompt user to save if document has been modifid
        if not self.check_current_document():
            return

        self._set_model(gm.GraphModel())
        self.nothing_cb(None)
        #self.document = FSMDocument.new_document()

    def save_sm_cb(self):
        #print 'has real filename?', self.document.has_real_filename()
        if self.graph_model.document.has_real_filename():
            self.graph_model.save(self.graph_model.document.get_filename())
            return True
        else:
            return self.save_as_sm_cb()

    def save_as_sm_cb(self):
        #popup file dialog
        #print 'save_as_sm_cb:before', rospy.is_shutdown()
        filename = str(qtg.QFileDialog.getSaveFileName(self, 'Save As', self.graph_model.document.get_filename()))
        #print 'save_as_sm_cb: after', rospy.is_shutdown()
        #self._fix_shutdown_flag()

        #user canceled
        if len(filename) == 0:
            return False

        if pt.exists(filename):
            #Ask if want to over write
            msg_box = qtg.QMessageBox()
            msg_box.setText('There is already a file with this name.')
            msg_box.setInformativeText('Do you want to overwrite it?')
            msg_box.setStandardButtons(qtg.QMessageBox.Yes | qtg.QMessageBox.No | qtg.QMessageBox.Cancel)
            msg_box.setDefaultButton(qtg.QMessageBox.Cancel)
            ret = msg_box.exec_()
            if ret == qtg.QMessageBox.No or ret == qtg.QMessageBox.Cancel:
                return False

        self.graph_model.save(filename)
        self.graph_model.document.set_filename(filename)
        self.graph_model.document.real_filename = True
        self.graph_model.document.modified = False
        return True

    #def _fix_shutdown_flag(self):
    #    pass
    #    #TODO Figure out what this crap was about.
    #    #Some messed up bug with QFileDialog!!!
    #    #import rospy.core as rpc
    #    #rpc._shutdown_flag = False

    def open_sm_cb(self):
        #prompt user if current document has not been saved
        if not self.check_current_document():
            return

        #print 'open IS SHUTDOWN before', rospy.is_shutdown()
        dialog = qtg.QFileDialog(self, 'Open State Machine', '~')
        dialog.setFileMode(qtg.QFileDialog.Directory)
        dialog.setViewMode(qtg.QFileDialog.List)

        #self._fix_shutdown_flag()
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

            #self.document = FSMDocument(filename, modified=False, real_filename=True)

        #print 'open IS SHUTDOWN after', rospy.is_shutdown()

    ####################################################################################################################
    # Graph Callbacks (called back from GraphModel)
    ####################################################################################################################

    def nothing_cb(self, pt):
        self.deselect_tool_buttons()
        self.set_selected_tool(None)
        self.set_selected_node(None)
        self.set_selected_edge(None, None, None)
        self.empty_properties_box()
        self.add_mode()
        self.disable_buttons()

    def node_cb(self, node):
        # When called back here, let the currently selected tool know that
        # it'll be *unselected*.

        # That tool would then save its state as a node (?) then
        # reload it when selected again.
        #   Would this interfere with node creation?
        #       only save nodes that have not been added to graph
        #       new names only if no saved nodes exist
        #       resetting *must* not create new names.
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

    def edge_cb(self, edge):
        self.set_selected_edge(edge.node1.id, edge.node2.id, edge.label)
        self.set_selected_node(None)
        self.disable_buttons()

    #Handler for double clicking on a node, descending a level
    def dclick_cb(self, node):
        snode = self.graph_model.get_state(node.id)
        if gm.is_container(snode):
            self.fsm_stack.append(FSMStackElement(self.graph_model, self.graph_view, snode))
            self._set_model(snode.get_child())
            #self._reconnect_states()
            self.nothing_cb(None)

    #Handler for double clicking on circle, ascending a level
    def dclick_container_cb(self, fsm_stack_element):

        #Store current model
        last_fsm_el = self.fsm_stack[-1]
        #last_fsm_el.node.set_child(self.graph_model)

        ######
        #recreate the old node with this new model as a child
        #each node need a function that lets you recreate it
        # what to call this? recreate? update?
        #       input: old node
        #       output: new node
        # Recreate replaces the old subtree with the current one being edited.
        new_node = last_fsm_el.node.recreate(self.graph_model)
        
        #replace old node in the graph, reserving links which exist
        # replace_node (fix it so that it works with new nodes of the same name)
        # restore_consistency
        #print 'new_smach_node', new_smach_node.get_name(), last_fsm_el.node.get_name()
        last_fsm_el.model.replace_node(new_node, last_fsm_el.node.get_name())

        #Shorten the stack to the element selected
        self.fsm_stack = self.fsm_stack[:self.fsm_stack.index(fsm_stack_element)]

        #Load the element we're given
        self._set_model(fsm_stack_element.model, view=fsm_stack_element.view)
        #self._reconnect_states()
        self.nothing_cb(None)

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

    ####################################################################################################################
    # Drawing code
    ####################################################################################################################
    def setup(self):
        graph._ctx = self.context
        self.context.speed(30.)
        self.context.size(700, 700)
        self._set_model(gm.GraphModel())

    def draw(self, properties_dict):
        properties_dict['selected_edge'] = self.selected_edge
        properties_dict['selected_node'] = self.selected_node
        properties_dict['width'        ] = self.ui.graphicsSuperView.viewport().width()
        properties_dict['height'       ] = self.ui.graphicsSuperView.viewport().height()
        properties_dict['name'         ] = self.graph_model.document.get_name()
        properties_dict['fsm_stack'    ] = self.fsm_stack
        self.graph_view.draw(properties_dict)

        if str(self.statusBar().currentMessage()) != self.status_bar_msg:
            self.statusBar().showMessage(self.status_bar_msg)

        if self.status_bar_exception != None:
            e = self.status_bar_exception
            if e.__class__ == smach.InvalidStateError or e.__class__ == smach.InvalidTransitionError:
                qtg.QMessageBox.information(self, str(self.objectName()), '%s: %s' % (str(e.__class__), str(e.message)))
            else:
                qtg.QMessageBox.information(self, str(self.objectName()), '%s: %s' % (str(e.__class__), str(e)))
            self.status_bar_exception = None

    def quit_cb(self):
        self.stop_drawing()
        rospy.signal_shutdown('User closed window.')
        self.app.quit()

def run_rcommander(plugin_namespace, robot=None, tf_listener=None):
    import plugins 
    import state_machine_tool as smt
    import sleep_tool as st
    import pointcloud_click_tool as ptl
    import freeze_frame_tool as frz

    app = qtg.QApplication(sys.argv)
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    rc = RCommander(app)
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
    sys.exit(app.exec_())
