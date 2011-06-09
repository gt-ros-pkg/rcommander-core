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

##
# keeps track of smach state machine, makes sure it is consistent with (G,V) representation
# model at this level has (V,E) reprentation
class RCommanderWindow(RNodeBoxBaseClass):

    def __init__(self):
        RNodeBoxBaseClass.__init__(self)
        self.connect(self.ui.run_button,   SIGNAL('clicked()'), self.run_cb)
        self.connect(self.ui.add_button,   SIGNAL('clicked()'), self.add_cb)
        self.connect(self.ui.reset_button, SIGNAL('clicked()'), self.reset_cb)

        self.connect(self.ui.delete_button, SIGNAL('clicked()'), self.delete_cb)
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
        self.set_selected_node('start')
        self.tool_dict['add_edge'] = {}
        
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

    def connectable_nodes(self):
        return self.graph_model.connectable_nodes()

    #def set_selected_node(self, name):
    #    if self.selected_node != None:
    #        self.set_node_style(self.selected_node, 'normal')

    #    self.selected_node = name
    #    if name != None:
    #        self.set_node_style(self.selected_node, 'selected')

    #    self.nb_graph.layout.refresh()

    def set_selected_node(self, name):
        self.selected_node = name

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
        if tool == None:
            pass
            #TODO: disable buttons

    #####################################################################
    # Callbacks
    #####################################################################
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

    ##################
    # Graph tools
    ##################
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

    def node_cb(self, node):
        if self.selected_graph_tool == 'add_edge':
            td = self.tool_dict['add_edge']
            #Make sure we have another node already and that node is in the graph
            if td.has_key('n1') and self.graph_model.node(td['n1'].id) != None:
                self.graph_model.add_edge(td['n1'].id, node.id)
                self.tool_dict['add_edge'] = {}
                self.set_selected_node(None)
                self.set_selected_edge(td['n1'].id, node.id)
            else:
                td['n1'] = node
                self.set_selected_node(node.id)
                self.set_selected_edge(None, None)
        else:
            self.set_selected_node(node.id)
            self.set_selected_edge(None, None)

    def edge_cb(self, edge):
        self.set_selected_edge(edge.node1.id, edge.node2.id)
        self.set_selected_node(None)

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

        g.node('start').style = 'marked'

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
        self.add_outcome('start')

        self.node = self.gve.node
        self.edge = self.gve.edge

    def connectable_nodes(self):
        return self.smach_states.keys()

    def add_node(self, smach_node, connect_to=None):
        if self.smach_states.has_key(smach_node.name):
            raise RuntimeException('Already has node of the same name.  This case should not happen.')

        #Link this node to all its outcomes
        self.gve.add_node(smach_node.name)
        self.smach_states[smach_node.name] = smach_node
        for outcome in smach_node.get_registered_outcomes():
            if not self.smach_states.has_key(outcome):
                self.smach_states[outcome] = EmptyState(outcome, temporary=True)
            #self.gve.add_edge(smach_node.name, outcome)
            self._add_edge(smach_node.name, outcome, None)

        #TODO figure out sensible way
        #Connect node to its parent
        #if connect_to != None:
        #    self.gve.add_edge(connect_to, smach_node.name)
        #    return True
        #else:
        #    return False

    def add_outcome(self, outcome_name):
        self.gve.add_node(outcome_name)
        self.smach_states[outcome_name] = EmptyState('start', False)

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
            if not self.is_modifiable(e.node2.id):
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
