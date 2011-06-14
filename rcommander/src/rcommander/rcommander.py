import roslib; roslib.load_manifest('rcommander')
import rospy
import sys
import time

from PyQt4 import QtGui
from PyQt4.QtGui import *
from PyQt4.QtCore import *
from nodebox.gui.qt import NodeBoxGraphicsView 
from nodebox import graphics
from nodebox.graphics.qt import *
import graph
import graph.style as gs
import math
import pdb
import tf
import smach
import smach_ros
import threading
from rcommander_auto import Ui_RCommanderWindow

#Import tools
import navigate_tool as nt
import tuck_tool as tt

class RNodeBoxBaseClass(QtGui.QMainWindow):
    def __init__(self):
        QtGui.QMainWindow.__init__(self)
        self.ui = Ui_RCommanderWindow()
        self.ui.setupUi(self)

        #Setup QGraphicsView
        #From NodeBoxDocumentBaseClass
        superView = self.ui.graphicsSuperView
        superView._scene = scene = QGraphicsScene()
        scene.setItemIndexMethod(QGraphicsScene.NoIndex)
        superView.setScene(scene)

        self.graphicsView = graphicsView = NodeBoxGraphicsView()
        scene.addItem(graphicsView)
        graphicsView._scene = scene
        graphicsView.superView = superView
        graphicsView._viewPort = superView.viewport()
        self.graphicsView.document = self
        self.currentView = self.graphicsView

        #Setup NB classes
        #from NodeBoxDocument
        self.namespace = {}
        textScaleFactor = QPixmap(1, 1).logicalDpiX() / 72.0
        self.canvas = graphics.Canvas()
        self.canvas._setTextScaleFactor(textScaleFactor)
        self.context = graphics.Context(self.canvas, self.namespace)

        #from NodeBoxDocument
        # _initNamespace
        self._pageNumber = 1
        self.__doc__ = {}
        self._frame = 150
        self._seed = time.time()
        self.animationTimer = None
        self.speed = 30.

        self.namespace["_ctx"] = self.context
        for attrName in dir(self.context):
            self.namespace[attrName] = getattr(self.context, attrName)
        self.namespace["__doc__"] = self.__doc__
        self.namespace["PAGENUM"] = self._pageNumber
        self.namespace["FRAME"] = self._frame

        #Setup the scene
        self._setup_draw(self.setup)

        #Start animation loop
        self.speed = self.canvas.speed
        self.animationTimer = QTimer(self)
        self.connect(self.animationTimer, SIGNAL("timeout()"), self.animation_cb)
        self.animationTimer.start(1000.0 / self.speed)


    def _setup_draw(self, fn):
        #from fastRun
        self.canvas.clear()
        pos = self.currentView.mousePosition
        mx, my = pos.x(), pos.y()
        self.namespace["MOUSEX"], self.namespace["MOUSEY"] = mx, my
        self.namespace["mousedown"] = self.currentView.mousedown
        self.namespace["keydown"] = self.currentView.keydown
        self.namespace["key"] = self.currentView.key
        self.namespace["keycode"] = self.currentView.keycode
        self.namespace["scrollwheel"] = self.currentView.scrollwheel
        self.namespace["wheeldelta"] = self.currentView.wheeldelta
        self.namespace['PAGENUM'] = self._pageNumber
        self.namespace['FRAME'] = self._frame
        for k in self.namespace.keys():
            exec "global %s\n" % (k)
            exec "%s = self.namespace['%s']" % (k, k)
        fn()
        self.currentView.canvas = self.canvas

    def animation_cb(self):
        self._setup_draw(self.draw)
        
    def stop(self):
        if self.animationTimer is not None:
            self.animationTimer.stop()
            self.animationTimer = None
        QApplication.restoreOverrideCursor()


def copy_style(astyle, bstyle):
    bstyle.background  = astyle.background  
    bstyle.fill        = astyle.fill       
    bstyle.stroke      = astyle.stroke     
    bstyle.strokewidth = astyle.strokewidth
    bstyle.text        = astyle.text       
    bstyle.font        = astyle.font       
    bstyle.fontsize    = astyle.fontsize   
    bstyle.textwidth   = astyle.textwidth  
    bstyle.align       = astyle.align      
    bstyle.depth       = astyle.depth      


class ThreadRunSM(threading.Thread):

    def __init__(self, sm_name, sm):
        threading.Thread.__init__(self)    
        self.sm = sm
        self.sm_name = sm_name
        self.outcome = None
        self.intro_server = None
        self.exception = None

    def run(self):
        print 'ThreadRunSM started with %s' % self.sm_name
        try:
            self.intro_server = smach_ros.IntrospectionServer(self.sm_name, self.sm, '/' + self.sm_name)
            self.intro_server.start()
            self.outcome = self.sm.execute()
        except smach.InvalidTransitionError, e:
            self.exception = e
        print 'ThreadRunSM finished'


##
# Checks for errors and redraw status bar if needed
##
class ThreadRunSMMonitor(threading.Thread):

    def __init__(self, sm_thread, parent_window):
        threading.Thread.__init__(self)    
        self.sm_thread = sm_thread
        self.parent_window = parent_window

    def run(self):
        r = rospy.Rate(100)
        while not rospy.is_shutdown():
            r.sleep()


##
# keeps track of smach state machine, makes sure it is consistent with (G,V) representation
# model at this level has (V,E) reprentation
class RCommanderWindow(RNodeBoxBaseClass):

    def __init__(self):
        RNodeBoxBaseClass.__init__(self)
        self.connect(self.ui.run_button,   SIGNAL('clicked()'), self.run_cb)
        self.connect(self.ui.add_button,   SIGNAL('clicked()'), self.add_cb)
        self.connect(self.ui.reset_button, SIGNAL('clicked()'), self.reset_cb)
        self.connect(self.ui.save_button, SIGNAL('clicked()'), self.save_cb)
        self.add_mode()

        self.connect(self.ui.delete_button, SIGNAL('clicked()'), self.delete_cb)
        self.connect(self.ui.action_Run, SIGNAL('triggered(bool)'), self.run_sm_cb)
        #self.connect(self.ui.add_edge_button, SIGNAL('clicked()'), self.add_edge_cb)
        self.empty_container(self.ui.properties_tab)
        self.empty_container(self.ui.connections_tab)

        #create instance variables
        self.tool_dict = {}
        self.selected_tool = None
        #Name of currently selected tool that operates on graph
        self.selected_graph_tool = None 
        self.selected_node = None
        self.selected_edge = None
        self.current_graph_name = 'untitled'
        #self.set_selected_node('start')
        self.tool_dict['add_edge'] = {}
        #self.node_cb(self.graph_model.node('start'))
        
        #ros things
        rospy.init_node('rcommander', anonymous=True)
        self.tf_listener = tf.TransformListener()

    def add_tools(self, tools_list):
        # In the future add tools according to which tab they want to be in
        self.button_group_tab = QButtonGroup()
        for tool in tools_list:
            self.button_group_tab.addButton(tool.create_button(self.ui.tab))
            self.tool_dict[tool.get_name()] = {'tool_obj': tool}
        self.ui.tab.update()

    def connectable_nodes(self, node_name, outcome):
        return self.graph_model.connectable_nodes(node_name, outcome)

    #def set_selected_node(self, name):
    #    if self.selected_node != None:
    #        self.set_node_style(self.selected_node, 'normal')

    #    self.selected_node = name
    #    if name != None:
    #        self.set_node_style(self.selected_node, 'selected')

    #    self.nb_graph.layout.refresh()

    def set_selected_node(self, name):
        self.selected_node = name

        #Unselect any tool from nongraph toolbar

        ##Load the state in
        #if self.graph_view.is_modifiable(name):
        #    smach_state = self.graph_model.get_smach_state(name)
        #    tool_obj = self.tool_dict[smach_state.tool_name()]['tool_obj']
        #    tool_obj.load_state(smach_state)

        #    #change add button to save

    def set_selected_edge(self, n1, n2):
        if n1 == None:
            self.selected_edge = None
        else:
            self.selected_edge = self.graph_model.edge(n1, n2)

    def empty_container(self, pbox): 
        #pbox = self.ui.properties_tab
        formlayout = pbox.layout()
        for i in range(formlayout.count()):
            item = formlayout.itemAt(0)
            formlayout.removeItem(item)
        children = pbox.children()
        for c in children[1:]:
            formlayout.removeWidget(c)
            c.setParent(None)
        formlayout.invalidate()
        pbox.update()

    def set_selected_tool(self, tool):
        self.selected_tool = tool
        #TODO: disable buttons

    #####################################################################
    # Callbacks
    #####################################################################
    def save_sm_cb(self):
        #write the line that would instantiate state
        pass

    def run_sm_cb(self, checked):
        #TODO Disable all buttons

        #Create and run state machine
        sm = self.graph_model.create_state_machine()
        rthread = ThreadRunSM(self.current_graph_name, sm)
        rthread.start()

        #Create monitoring thread


    ##################
    # Behavior tools
    ##################
    def run_cb(self):
        if self.selected_tool == None:
            return
        tool_instance = self.tool_dict[self.selected_tool]['tool_obj']
        tool_instance.run()

    def add_cb(self):
        if self.selected_tool == None:
            return
        tool_instance = self.tool_dict[self.selected_tool]['tool_obj']
        smach_node = tool_instance.create_node()
        self.graph_model.add_node(smach_node, connect_to=self.selected_node)
        if self.selected_node != None:
            #self.set_selected_node(None)
            self.set_selected_node(smach_node.name)

        self.tool_dict[self.selected_tool]['tool_obj'].refresh_connections_box()
        self.graph_view.refresh()

    def reset_cb(self):
        if self.selected_tool == None:
            return
        tool_instance = self.tool_dict[self.selected_tool]['tool_obj']
        tool_instance.reset()

    def save_cb(self):
        tool_instance = self.tool_dict[self.selected_tool]['tool_obj']
        #old_smach_node = self.graph_model.get_smach_state()
        old_node_name = tool_instance.get_current_node_name()
        # create a node with new settings
        smach_node = tool_instance.create_node()
        # 'delete' old smach node
        self.graph_model.replace_node(smach_node, old_node_name)
        #self.graph_model.set_smach_state(old_smach_node.get_name(), smach_node)

        # connection changes are made instantly (so don't worry about them)
        # only saving of internal node parameters must be implemented by client tools

    def connection_changed(self, node_name, outcome_name, new_outcome):
        self.graph_model.connection_changed(node_name, outcome_name, new_outcome)

    ##################
    # Graph tools
    ##################
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

    def deselect_tool_buttons(self):
        self.button_group_tab.setExclusive(False)
        button = self.button_group_tab.checkedButton()
        print button
        if button != None:
            print 'checked', button.isChecked(), button.isDown(), button.isCheckable(), button.text()
            button.setDown(False)
            button.setChecked(False)
            print 'checked2', button.isChecked(), button.isDown()
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

    def delete_cb(self):
        if self.selected_node != None:
            if self.selected_node != 'start':
                self.set_selected_node(self.graph_model.delete_node(self.selected_node))
                self.graph_view.refresh()
            else:
                print 'Can\'t delete start node!'

        if self.selected_edge != None:
            se = self.selected_edge
            self.set_selected_edge(None, None)
            self.graph_model.delete_edge(se)
            self.graph_view.refresh()

    def nothing_cb(self, pt):
        self.set_selected_node(None)
        self.set_selected_edge(None, None)
        self.empty_properties_box()
        self.add_mode()
        self.disable_buttons()
        self.deselect_tool_buttons()

    def node_cb(self, node):
        self.set_selected_node(node.id)
        self.set_selected_edge(None, None)

        if self.graph_model.get_smach_state(node.id).__class__ != EmptyState:
            smach_state = self.graph_model.get_smach_state(node.id)
            tool = self.tool_dict[smach_state.tool_name]['tool_obj']
            tool.button.setChecked(True)
            tool.activate_cb(smach_state.get_name())
            tool.node_selected(smach_state)

            self.edit_mode()
            self.set_selected_tool(smach_state.tool_name)
            self.enable_buttons()
        else:
            self.empty_properties_box()
            self.disable_buttons()

    def edge_cb(self, edge):
        self.set_selected_edge(edge.node1.id, edge.node2.id)
        self.set_selected_node(None)
        self.disable_buttons()

    #def add_edge_cb(self):
    #    print 'IGNORED. remove/rethink this tool.'
    #    #if self.ui.add_edge_button.isChecked():
    #    #    self.selected_graph_tool = 'add_edge'
    #    #    self.set_selected_node(None)
    #    #else:
    #    #    self.selected_graph_tool = None

    #    #print 'Tool selected:', self.selected_graph_tool

    #####################################################################
    # Drawing
    #####################################################################
    def setup(self):
        graph._ctx = self.context
        self.graph_model = GraphModel()
        self.graph_view = GraphView(self.context, self.graph_model)
        self.graph_view.setup()

        self.graph_model.gve.events.click = self.node_cb
        self.graph_model.gve.events.click_edge = self.edge_cb
        self.graph_model.gve.events.click_nothing = self.nothing_cb


    def draw(self):
        properties_dict = {'selected_edge': self.selected_edge,
                           'selected_node': self.selected_node}
        self.graph_view.draw(properties_dict)


class GraphView:
    def __init__(self, context, graph_model):
        self.graph_model = graph_model
        g = self.graph_model.gve
        self.context = context 
        self.gve = g
        self.refresh = self.gve.layout.refresh

        selected_style = g.styles.create('selected')
        normal_style   = g.styles.create('normal')
        normal_edge_style   = g.styles.create('normal_edge')
        selected_edge_style = g.styles.create('selected_edge')

        copy_style(g.styles.important, selected_style)
        copy_style(g.styles.default, normal_style)
        copy_style(g.styles.default, normal_edge_style)
        copy_style(g.styles.default, selected_edge_style)
        selected_edge_style.stroke = self.context.color(0.80, 0.00, 0.00, 0.75)
        selected_edge_style.strokewidth = 1.0

        #g.node('start').style = 'marked'

    def set_node_style(self, node_name, style):
        self.gve.node(node_name).style = style
        self.gve.layout.refresh()

    def setup(self):
        self.context.speed(30.)
        self.context.size(700, 700)

    def draw(self, properties_dict):
        cx = self.context
        g  = self.gve

        for n in g.nodes:
            if properties_dict['selected_node'] == n.id:
                self.set_node_style(n.id, 'selected')
            else:
                self.set_node_style(n.id, 'normal')

        draw_func = None
        if properties_dict['selected_edge'] != None:
            def draw_selected():
                cx = self.context
                g  = self.gve
                #edge = self.selected_edge 
                edge = properties_dict['selected_edge']
                x0, y0 = edge.node1.x, edge.node1.y
                x1, y1 = edge.node2.x, edge.node2.y
                coordinates = lambda x, y, d, a: (x+math.cos(math.radians(a))*d, y+math.sin(math.radians(a))*d)

                # Find the edge's angle based on node1 and node2 position.
                a = math.degrees(math.atan2(y1-y0, x1-x0))
                # draw line from node's edge instead of it's center.
                r = edge.node2.r
                d = math.sqrt(pow(x1-x0, 2) + pow(y1-y0, 2))
                x00, y00 = coordinates(x0, y0, r+1, a)
                x01, y01 = coordinates(x0, y0, d-r-1, a)

                # draw
                p1 = [x00, y00]
                p2 = [x01, y01]
                cx.fill()
                cx.strokewidth(1.0)
                cx.stroke(1., 153./255., 0, .75)
                cx.beginpath(p1[0], p1[1])
                cx.lineto(p2[0], p2[1])
                path = cx.endpath(False)
                gs.edge_arrow(g.styles[edge.node1.style], path, edge, radius=10)
                cx.drawpath(path)
            draw_func = draw_selected
        g.draw(directed=True, traffic=False, user_draw=draw_func)


class EmptyState:

    def __init__(self, name, temporary):
        self.name = name
        self.temporary = temporary


class GraphModel:

    def __init__(self):
        self.gve = graph.create(depth=True)
        self.smach_states = {}
        #self.add_outcome('start')

        self.node = self.gve.node
        self.edge = self.gve.edge

    def create_state_machine(self):
        sm = smach.StateMachine(outcomes=self.outcomes())
        with sm:
            for node_name in self.nonoutcomes():
                node = self.smach_states[node_name]
                transitions = {}
                for e in self.gve.node(node_name).edges:
                    transitions[e.outcome] = e.node2.id
                print node_name, transitions
                smach.StateMachine.add(node_name, node, transitions=transitions)
        return sm

    def nonoutcomes(self):
        noc = []
        for node_name in self.smach_states.keys():
            if self.smach_states[node_name].__class__ != EmptyState:
                noc.append(node_name)
        return noc

    def outcomes(self):
        #all empty states are outcomes
        oc = []
        for node_name in self.smach_states.keys():
            if self.smach_states[node_name].__class__ == EmptyState:
                oc.append(node_name)
        print 'outcomes', oc
        return oc



    def pop_smach_state(self, node_name):
        return self.smach_states.pop(node_name)

    def get_smach_state(self, node_name):
        return self.smach_states[node_name]

    def set_smach_state(self, node_name, state):
        self.smach_states[node_name] = state

    def replace_node(self, new_node, old_node_name):
        self.smach_states.pop(old_node_name)
        self.smach_states[new_node.get_name()] = new_node
        new_node_name = new_node.get_name()

        if new_node_name != old_node_name:
            self.gve.add_node(new_node_name)
            for e in self.gve.node(old_node_name).edges:
                self.gve.remove_edge(e.node1.id, e.node2.id)
                if e.node1.id == old_node_name:
                    self.gve.add_edge(new_node_name, e.node2.id)
                    #edges.append([new_node_name, e.node2.id])
                else:
                    self.gve.add_edge(e.node1.id, new_node_name)
                    #edges.append([e.node1.id, new_node_name])
            self.gve.remove_node(old_node_name)

    def connectable_nodes(self, node_name, outcome):
        #can't connect to
        #  temporary nodes already connected whose name is not current outcome
        allowed_nodes = []

        for k in self.smach_states.keys():
            if not self.is_modifiable(k) and k != outcome:
                continue
            if node_name == k:
                continue
            allowed_nodes.append(k)

        if node_name == None:
            allowed_nodes.append(outcome)
            allowed_nodes = list(set(allowed_nodes))

        return allowed_nodes

    def add_node(self, smach_node, connect_to=None):
        if self.smach_states.has_key(smach_node.name):
            raise RuntimeException('Already has node of the same name.  This case should not happen.')

        #Link this node to all its outcomes
        self.gve.add_node(smach_node.name)
        self.smach_states[smach_node.name] = smach_node
        for outcome in smach_node.get_registered_outcomes():
            if not self.smach_states.has_key(outcome):
                self.smach_states[outcome] = EmptyState(outcome, temporary=True)
                self.gve.add_node(outcome)
            #self.gve.add_edge(smach_node.name, outcome)
            self._add_edge(smach_node.name, outcome, outcome)

        #TODO figure out sensible way
        #Connect node to its parent
        #if connect_to != None:
        #    self.gve.add_edge(connect_to, smach_node.name)
        #    return True
        #else:
        #    return False

    def add_outcome(self, outcome_name):
        self.gve.add_node(outcome_name)
        self.smach_states[outcome_name] = EmptyState(outcome_name, False)

    def delete_node(self, node_name):
        #temporary nodes are only removable when the state transitions are linked to something else
        if not self.is_modifiable(node_name):
            return 

        #Find parents and children
        node_obj = self.gve.node(node_name)
        children_edges = []
        parent_edges = []
        for cn in node_obj.links:
            edge = self.gve.edge(node_name, cn.id)
            if (edge.node1.id == node_name) and (edge.node2.id == node_name):
                raise Exception('Self link detected on node %s! This isn\'t supposed to happen.' % node_name)
            if edge.node1.id == node_name:
                children_edges.append(edge)
            elif edge.node2.id == node_name:
                parent_edges.append(edge)

        #Remove added orphan children edges
        filtered_children_edges = []
        for e in children_edges:
            if not self.is_modifiable(e.node2.id) and len(e.node2.edges) <= 1:
                self.gve.remove_edge(node_name, e.node2.id)
                self.gve.remove_node(e.node2.id)
                self.smach_states.pop(e.node2.id)
            else:
                filtered_children_edges.append(e)

        new_selected_node = None
        #If we have one or more than one parent
        if len(parent_edges) >= 1:
            #Point edges on children to first parent
            parent_node_id = parent_edges[0].node1.id
            for e in filtered_children_edges:
                self.gve.remove_edge(node_name, e.node2.id)
                self.gve.add_edge(parent_node_id, e.node2.id)
            new_selected_node = parent_node_id

        #If no parents
        elif len(parent_edges) == 0:
            #just remove children edges
            for e in filtered_children_edges:
                self.gve.remove_edge(node_name, e.node2.id)

            if len(filtered_children_edges) > 1:
                new_selected_node = filtered_children_edges[0].node2.id
            else:
                if len(self.gve.nodes) > 0:
                    new_selected_node = self.gve.nodes[0].id
                else:
                    new_selected_node = 'start'

        self.gve.remove_node(node_name)
        self.smach_states.pop(node_name)
        return new_selected_node

    def is_modifiable(self, node_name):
        if (self.smach_states[node_name].__class__ == EmptyState) and self.smach_states[node_name].temporary:
            return False
        else:
            return True

    def _add_edge(self, n1, n2, n1_outcome):
        if not self.smach_states.has_key(n1) or not self.smach_states.has_key(n2):
            raise RuntimeException('One of the modes does not exist.  Can\'t add edge.')
        if self.gve.edge(n1, n2) != None:
            rospy.loginfo("Edge between %s and %s exists, ignoring connnection request" % (n1, n2))
            return False

        #Don't add edges to "temporary" nodes
        if n1_outcome == None and self.is_modifiable(n2):
            raise RuntimeException('Must specify outcome as goal node is not a temporary node.')

        self.gve.add_edge(n1, n2)
        self.gve.edge(n1, n2).outcome = n1_outcome
        return True

    def add_edge(self, n1, n2, n1_outcome):
        if not self.is_modifiable(n1) or not self.is_modifiable(n2):
            return False
        else:
            return self._add_edge(n1, n2, n1_outcome)

    def delete_edge(self, edge):
        if not self.is_modifiable(edge.node1.id) or not self.is_modifiable(edge.node2.id):
            return False
        else:
            self.gve.remove_edge(edge.node1.id, edge.node2.id)
            return True

    def connection_changed(self, node_name, outcome_name, new_outcome):
        #node is not valid or hasn't been created yet
        print 'connection_changed', node_name, outcome_name, new_outcome
        if node_name == None:
            return
        if not self.smach_states.has_key(new_outcome):
            raise RuntimeException('Doesn\'t have state: %s' % new_outcome)
        self.get_smach_state(node_name).outcome_choices[outcome_name] = new_outcome

        #find the old edge
        old_edge = None
        for edge in self.gve.node(node_name).edges:
            if edge.outcome == outcome_name:
                if old_edge != None:
                    raise RuntimeException('Two edges detected for one outcome named %s' % outcome_name)
                old_edge = edge

        #remove the old connection
        self.gve.remove_edge(node_name, old_edge.node2.id)
        #remove the old node if it's temporary 
        if not self.is_modifiable(old_edge.node2.id) and old_edge.node2.id != 'start':
            #and not connected
            if len(self.gve.node(old_edge.node2.id).edges) <= 0:
                self.gve.remove_node(old_edge.node2.id)

        #add new connection
        if self.gve.node(new_outcome) == None:
            self.smach_states[new_outcome] = EmptyState(new_outcome, temporary=True)
            self.gve.add_node(new_outcome)
        self._add_edge(node_name, new_outcome, outcome_name)


app = QtGui.QApplication(sys.argv)
rc = RCommanderWindow()
rc.add_tools([nt.NavigateTool(rc), tt.TuckTool(rc)])
rc.show()
sys.exit(app.exec_())

















    #def tuck_cb(self):
    #    #Load properties into properties box.
    #    self.add_node('tuck')
    #def navigate_cb(self):
    #    #Load properties into properties box.
    #    self.empty_properties_box()
    #    pbox = self.ui.behavior_properties_box
    #    formlayout = pbox.layout()

    #    xline = QLineEdit(pbox)
    #    formlayout.addRow("&x", xline)
    #    yline = QLineEdit(pbox)
    #    formlayout.addRow("&y", yline)
    #    tline = QLineEdit(pbox)
    #    formlayout.addRow("&theta", tline)
    #    pbox.update()

    #    # goal x
    #    # goal y
    #    # goal theta
    #    # frame

    #    # turn on markers
    #    # locat current location button
    #    #

    #    self.add_node('navigate')

    #def set_node_style(self, node_name, style):
    #    self.nb_graph.node(node_name).style = style
    #    self.nb_graph.layout.refresh()



   #def delete_node(self, node_name):
    #    #find parents and children
    #    node_obj = self.nb_graph.node(node_name)
    #    children_edges = []
    #    parent_edges = []
    #    for cn in node_obj.links:
    #        edge = self.nb_graph.edge(node_name, cn.id)
    #        if (edge.node1.id == node_name) and (edge.node2.id == node_name):
    #            raise Exception('Self link detected on node %s! This isn\'t supposed to happen.' % node_name)
    #        if edge.node1.id == node_name:
    #            children_edges.append(edge)
    #        elif edge.node2.id == node_name:
    #            parent_edges.append(edge)

    #    #If we have one or more than one parent
    #    if len(parent_edges) >= 1:
    #        #Point edges on children to first parent
    #        parent_node_id = parent_edges[0].node1.id
    #        for e in children_edges:
    #            self.nb_graph.remove_edge(node_name, e.node2.id)
    #            self.nb_graph.add_edge(parent_node_id, e.node2.id)
    #        if node_name == self.selected_node:
    #            self.set_selected_node(parent_node_id)

    #    #If no parents
    #    elif len(parent_edges) == 0:
    #        #just remove children edges
    #        for e in children_edges:
    #            self.nb_graph.remove_edge(node_name, e.node2.id)
    #        if node_name == self.selected_node:
    #            if len(children_edges) > 1:
    #                self.set_selected_node(children_edges[0].node2.id)
    #            else:
    #                if len(self.nb_graph.nodes) > 0:
    #                    self.set_selected_node(self.nb_graph.nodes[0].id)
    #                else:
    #                    self.set_selected_node('start')
    #    self.nb_graph.remove_node(node_name)
    #    self.nb_graph.layout.refresh()

    #def delete_edge(self, edge):
    #    self.nb_graph.remove_edge(edge.node1.id, edge.node2.id)
    #    self.nb_graph.layout.refresh()

    #def add_node(self, name):
    #    if self.selected_node != None:
    #        self.nb_graph.add_edge(self.selected_node, name)
    #        self.set_node_style(name, 'normal')
    #        self.nb_graph.layout.refresh()
    #        self.set_selected_node(name)
    #        return True
    #    else:
    #        return False



    #def draw(self):
    #    cx = self.context
    #    g  = self.nb_graph

    #    if self.selected_edge != None:
    #        def draw_selected():
    #            cx = self.context
    #            g  = self.nb_graph
    #            edge = self.selected_edge 
    #            x0, y0 = edge.node1.x, edge.node1.y
    #            x1, y1 = edge.node2.x, edge.node2.y
    #            coordinates = lambda x, y, d, a: (x+math.cos(math.radians(a))*d, y+math.sin(math.radians(a))*d)

    #            # Find the edge's angle based on node1 and node2 position.
    #            a = math.degrees(math.atan2(y1-y0, x1-x0))
    #            # draw line from node's edge instead of it's center.
    #            r = edge.node2.r
    #            d = math.sqrt(pow(x1-x0, 2) + pow(y1-y0, 2))
    #            x00, y00 = coordinates(x0, y0, r+1, a)
    #            x01, y01 = coordinates(x0, y0, d-r-1, a)

    #            # draw
    #            p1 = [x00, y00]
    #            p2 = [x01, y01]
    #            cx.fill()
    #            cx.strokewidth(1.0)
    #            cx.stroke(1., 153./255., 0, .75)
    #            cx.beginpath(p1[0], p1[1])
    #            cx.lineto(p2[0], p2[1])
    #            path = cx.endpath(False)
    #            gs.edge_arrow(g.styles[edge.node1.style], path, edge, radius=10)
    #            cx.drawpath(path)

    #        g.draw(directed=True, traffic=False, user_draw=draw_selected)
    #    else:
    #        g.draw(directed=True, traffic=False)










        #g.add_edge("roof"        , "house")
        #g.add_edge("garden"      , "house")
        #g.add_edge("room"        , "house")
        #g.add_edge("kitchen"     , "room")
        #g.add_edge("bedroom"     , "room")
        #g.add_edge("bathroom"    , "room")
        #g.add_edge("living room" , "room")
        #g.add_edge("sofa"        , "living room")
        #g.add_edge("table"       , "living room")
