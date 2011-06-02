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

from rcommander_auto import Ui_RCommanderWindow


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


class RCommanderWindow(RNodeBoxBaseClass):

    def __init__(self):
        RNodeBoxBaseClass.__init__(self)
        self.connect(self.ui.tuck_button, SIGNAL('clicked()'), self.tuck_cb)
        self.connect(self.ui.navigate_button, SIGNAL('clicked()'), self.navigate_cb)

        self.connect(self.ui.aseg_button, SIGNAL('clicked()'), self.delete_cb)
        self.connect(self.ui.oselect_button, SIGNAL('clicked()'), self.add_edge_cb)

    def delete_node(self, node_name):
        #find parents and children
        node_obj = self.nb_graph.node(node_name)
        children_edges = []
        parent_edges = []
        for cn in node_obj.links:
            edge = self.nb_graph.edge(node_name, cn.id)
            if (edge.node1.id == node_name) and (edge.node2.id == node_name):
                raise Exception('Self link detected on node %s! This isn\'t supposed to happen.' % node_name)
            if edge.node1.id == node_name:
                children_edges.append(edge)
            elif edge.node2.id == node_name:
                parent_edges.append(edge)

        #If we have one or more than one parent
        if len(parent_edges) >= 1:
            #Point edges on children to first parent
            parent_node_id = parent_edges[0].node1.id
            for e in children_edges:
                self.nb_graph.remove_edge(node_name, e.node2.id)
                self.nb_graph.add_edge(parent_node_id, e.node2.id)
            if node_name == self.selected_node:
                self.set_selected_node(parent_node_id)

        #If no parents
        elif len(parent_edges) == 0:
            #just remove children edges
            for e in children_edges:
                self.nb_graph.remove_edge(node_name, e.node2.id)
            if node_name == self.selected_node:
                if len(children_edges) > 1:
                    self.set_selected_node(children_edges[0].node2.id)
                else:
                    if len(self.nb_graph.nodes) > 0:
                        self.set_selected_node(self.nb_graph.nodes[0].id)
                    else:
                        self.set_selected_node('start')
        self.nb_graph.remove_node(node_name)
        self.nb_graph.layout.refresh()

    def delete_edge(self, edge):
        self.nb_graph.remove_edge(edge.node1.id, edge.node2.id)
        self.nb_graph.layout.refresh()

    def add_node(self, name):
        if self.selected_node != None:
            self.nb_graph.add_edge(self.selected_node, name)
            self.set_node_style(name, 'normal')
            self.nb_graph.layout.refresh()
            self.set_selected_node(name)
            return True
        else:
            return False

    def set_selected_node(self, name):
        if self.selected_node != None:
            self.set_node_style(self.selected_node, 'normal')

        self.selected_node = name
        if name != None:
            self.set_node_style(self.selected_node, 'selected')

        self.nb_graph.layout.refresh()

    def set_selected_edge(self, n1, n2):
        if n1 == None:
            self.selected_edge = None
        else:
            self.selected_edge = self.nb_graph.edge(n1, n2)

    #####################################################################
    # Callbacks
    #####################################################################
    def delete_cb(self):
        if self.selected_node != None:
            if self.selected_node != 'start':
                self.delete_node(self.selected_node)
            else:
                print 'Can\'t delete start node!'

        if self.selected_edge != None:
            se = self.selected_edge
            self.set_selected_edge(None, None)
            self.delete_edge(se)


    def nothing_cb(self, pt):
        self.set_selected_node(None)
        self.set_selected_edge(None, None)

    def node_cb(self, node):
        if self.selected_tool == 'add_edge':
            td = self.tool_dict['add_edge']
            if td.has_key('n1'):
                print td['n1'].id, node.id
                self.nb_graph.add_edge(td['n1'].id, node.id)
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

    def tuck_cb(self):
        self.add_node('tuck')

    def navigate_cb(self):
        self.add_node('navigate')

    def add_edge_cb(self):
        if self.selected_tool == 'add_edge':
            self.selected_tool = None
        else:
            self.selected_tool = 'add_edge'
            if self.selected_node != None:
                self.set_selected_node(None)
                #self.tool_dict['add_edge']['n1'] = self.nb_graph.node[self.selected_node]

        print 'Tool selected:', self.selected_tool

    #####################################################################
    # Drawing
    #####################################################################
    def setup(self):
        self.context.speed(30.)
        self.context.size(500, 500)
        graph._ctx = self.context
        g = graph.create(depth=True)

        #set looks
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

        #Create initial graph
        g.add_node('start')
        g.node('start').style = 'marked'
        g.solve()
        g.draw(directed=True, traffic=1)
        g.events.click = self.node_cb
        g.events.click_edge = self.edge_cb
        g.events.click_nothing = self.nothing_cb
        self.nb_graph = g

        #create instance variables
        self.tool_dict = {}
        self.selected_tool = None
        self.selected_node = None
        self.selected_edge = None
        self.set_selected_node('start')
        self.tool_dict['add_edge'] = {}

    def set_node_style(self, node_name, style):
        self.nb_graph.node(node_name).style = style
        self.nb_graph.layout.refresh()

    def draw(self):
        cx = self.context
        g  = self.nb_graph

        if self.selected_edge != None:
            def draw_selected():
                cx = self.context
                g  = self.nb_graph
                edge = self.selected_edge 
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

            g.draw(directed=True, traffic=False, user_draw=draw_selected)
        else:
            g.draw(directed=True, traffic=False)


app = QtGui.QApplication(sys.argv)
rc = RCommanderWindow()
rc.show()
sys.exit(app.exec_())


































        #g.add_edge("roof"        , "house")
        #g.add_edge("garden"      , "house")
        #g.add_edge("room"        , "house")
        #g.add_edge("kitchen"     , "room")
        #g.add_edge("bedroom"     , "room")
        #g.add_edge("bathroom"    , "room")
        #g.add_edge("living room" , "room")
        #g.add_edge("sofa"        , "living room")
        #g.add_edge("table"       , "living room")
