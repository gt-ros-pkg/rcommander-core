#import roslib; roslib.load_manifest('rcommander_core')

import graph.style as gs
import graph
import graph.layout as gl

import tool_utils as tu
import graph_model as gm

import numpy as np
import time
import copy

## Copies a NodeBox.graph style
# @param astyle Style to copy from.
# @param bstyle Style to copy to.
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

## View (in MVC) class for GraphModel objects
class GraphView:

    ## Constructor
    # @param context Nodebox drawing context.
    # @param graph_model a GraphModel object to draw.
    def __init__(self, context, graph_model):
        self.graph_model = graph_model
        g = self.graph_model.gve
        self.gve = g
        self.context = context 

        node_outlines = self.context.color(0.4, 0.4, 0.4, 1.)
        text_color = self.context.color(0.3, 0.3, 0.3, 1.)
        node_font_size = 14

        #Customizations
        g.styles.default.depth = True
        g.styles.default.background = self.context.color(1., 1., 1., 1.)
        g.styles.default.stroke = node_outlines
        g.styles.default.text = text_color
        g.styles.default.fontsize = node_font_size
        g.styles.root.text = self.context.color(255/255., 153/255., 51/255., 1.)

        g.styles.important.fontsize = node_font_size
        g.styles.important.text = text_color
        g.styles.important.stroke = node_outlines

        g.styles.marked.fontsize = node_font_size
        g.styles.marked.text = text_color
        g.styles.marked.stroke = node_outlines
        #g.styles.default.fontsize = 12

        self.refresh = self.gve.layout.refresh

        old_outcome_style = g.styles.create('old_outcome')
        active_node_style = g.styles.create('active_node')
        selected_style = g.styles.create('selected')
        normal_style   = g.styles.create('normal')
        normal_edge_style   = g.styles.create('normal_edge')
        selected_edge_style = g.styles.create('selected_edge')
        graph_circle = g.styles.create('graph_circle')
        container = g.styles.create('container')
        container_selected = g.styles.create('container_selected')

        copy_style(g.styles.important, old_outcome_style)
        copy_style(g.styles.important, active_node_style)
        copy_style(g.styles.important, selected_style)
        copy_style(g.styles.default, normal_style)
        copy_style(g.styles.default, normal_edge_style)
        copy_style(g.styles.default, selected_edge_style)
        copy_style(g.styles.default, graph_circle)
        copy_style(g.styles.default, container)
        copy_style(g.styles.default, container_selected)

        graph_circle.fill = self.context.color(.96, .96, .96, .96)
        graph_circle.stroke = self.context.color(.8, .8, .8, 1.)
        graph_circle.strokewidth = 3
        graph_circle.fontsize = 24
        graph_circle.textwidth = 800
        graph_circle.text = self.context.color(.5, .5, .5, 1.)

        container.fill = self.context.color(255./255, 204./255, 102./255., .4)
        container.node = g.styles.important.node

        container_selected.fill = self.context.color(255./255, 204./255, 102./255., 1.)
        container_selected.node = g.styles.important.node

        selected_style.text = text_color
        selected_edge_style.stroke = self.context.color(0.80, 0.00, 0.00, 0.75)
        selected_edge_style.strokewidth = 1.0

        active_node_style.text = text_color
        active_node_style.fill = self.context.color(153./255, 255./255, 51/255, .75)
        active_node_style.strokewidth = 3

        old_outcome_style.text = text_color
        old_outcome_style.fill = self.context.color(153./255, 255./255, 51/255, .4)

        self.radii_increment = 150
        self.fsm_start_color = 1.
        self.fsm_end_color = .96
        self.fsm_stroke_color = .85
        self.fsm_current_context_node = None
        self.fsm_dclick_cb = None

        self.right_clicked = None
        self.dx = 0.
        self.dy = 0.
        self.tx = 0.
        self.ty = 0.

    ## Set the drawing style for given Nodebox node.
    # @param node_name Name of node.
    # @param style Nodebox graph style name (has to be known in nodebox graph
    #               object beforehand)
    def set_node_style(self, node_name, style):
        self.gve.node(node_name).style = style
        self.gve.layout.refresh()

    ## Get style currently used
    # @param node_name Name of node
    def get_node_style(self, node_name):
        return self.gve.node(node_name).style

    ## Moves the view if the background have been dragged
    def _background_drag(self, properties_dict):
        mouse_pose = properties_dict['MOUSEX'], properties_dict['MOUSEY']

        if properties_dict['rightdown']:
            if not self.right_clicked:
                self.right_clicked = mouse_pose
            else:
                self.tx = mouse_pose[0] - self.right_clicked[0]
                self.ty = mouse_pose[1] - self.right_clicked[1]
        else:
            #Commit transform
            self.right_clicked = None
            self.dx += self.tx
            self.dy += self.ty
            self.ty = 0.
            self.tx = 0.

    ## Setup method for NodeBox drawing env
    def setup(self):
        pass

    ## Draw graph (called n times per second)
    def draw(self, properties_dict):
        self.context.size(properties_dict['width'], properties_dict['height'])
        cx = self.context
        g  = self.gve


        for n in g.nodes:
            if properties_dict['selected_node'] == n.id:
                self.set_node_style(n.id, 'selected')
            else:
                self.set_node_style(n.id, 'normal')

            if self.graph_model.get_start_state() == n.id:
                if self.get_node_style(n.id) == 'selected':
                    self.set_node_style(n.id, 'important')
                else:
                    self.set_node_style(n.id, 'marked')

            if hasattr(self.graph_model.get_state(n.id), 'get_child'):
                if self.get_node_style(n.id) == 'selected':
                    self.set_node_style(n.id, 'container_selected')
                else:
                    self.set_node_style(n.id, 'container')

            if self.graph_model.is_running():
		if not self.graph_model.sm_thread.has_key('current_states'):
                    print 'KEYS!', self.graph_model.sm_thread.keys()
                if self.graph_model.sm_thread['current_states'] != None and \
                        (len(set(self.graph_model.sm_thread['current_states']).intersection(set([n.id]))) > 0):
                    self.set_node_style(n.id, 'active_node')

            if self.graph_model.get_last_outcome() != None:
                outcome, t = self.graph_model.get_last_outcome() 
                if outcome == n.id:
                    if time.time() - t < 10.:
                        self.set_node_style(n.id, 'old_outcome')


        draw_func = None

        ## Draw selected element
        def draw_selected():
            if properties_dict['selected_edge'] == None:
                return
            cx = self.context
            g  = self.gve
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

        def draw_fsm_circles():
            g = self.gve

            #figure out where centroids should be
            coords = []
            [coords.append([n.x, n.y]) for n in g.nodes]
            coords = np.matrix(coords).T
            centroid = np.median(coords, 1)
            if len(coords) == 0:
                return

            #calculate where radii should be
            radius = np.max(np.power(np.sum(np.power((coords - centroid), 2), 0), .5)) + gm.GraphModel.NODE_RADIUS*2
            radius = max(radius, 200.)
            container_style = g.styles.graph_circle
            container_stroke = container_style.stroke
            
            ##
            #Draw fsm_stack
            stack = copy.copy(properties_dict['fsm_stack'])
            largest_radii = radius + len(stack) * self.radii_increment
            color = self.fsm_start_color
            if len(stack) > 0:
                color_incre = (self.fsm_start_color - self.fsm_end_color) / len(stack)

            #draw stack
            for el in stack:
                name = el.model.document.get_name()#el.document.get_name()

                #Draw node
                stack_node = graph.node(g, radius = largest_radii, id = name)
                stack_node.x, stack_node.y = centroid[0,0], centroid[1,0]
                el.graph_node = stack_node
                container_style.fill = self.context.color(color, color, color, 1.)
                container_style.stroke = self.context.color(self.fsm_stroke_color, self.fsm_stroke_color, 1.)
                gs.node(container_style, stack_node, g.alpha)

                #Draw label
                node_label_node_ = graph.node(g, radius = largest_radii, id = name)
                node_label_node_.x, node_label_node_.y = centroid[0,0], centroid[1,0] - largest_radii
                gs.node_label(container_style, node_label_node_, g.alpha)

                color -= color_incre
                largest_radii -= self.radii_increment

            ##
            #Draw node

            #Draw node circle
            graph_name_node = graph.node(g, radius=radius, id = properties_dict['name'])
            graph_name_node.x, graph_name_node.y = centroid[0,0], centroid[1,0]
            self.fsm_current_context_node = graph_name_node
            container_style.fill = self.context.color(self.fsm_end_color, self.fsm_end_color, self.fsm_end_color, 1.)
            container_style.stroke = container_stroke
            gs.node(container_style, graph_name_node, g.alpha)

            #draw node label
            node_label_node = graph.node(g, radius=radius, id = properties_dict['name'])
            node_label_node.x, node_label_node.y = centroid[0,0], centroid[1,0] - radius
            gs.node_label(container_style, node_label_node, g.alpha)

        ## Detects if a nested state machine has been selected.
        def detect_fsm_click():
            def in_node(x, y, n):
                return (abs(x - n.x) < n.r) and (abs(y - n.y) < n.r)

            mousex_g = self.context._ns['MOUSEX'] - self.gve.x
            mousey_g = self.context._ns['MOUSEY'] - self.gve.y
            if self.context._ns['mousedoubleclick'] and len(properties_dict['fsm_stack']) > 0:
                if not in_node(mousex_g, mousey_g, self.fsm_current_context_node):
                    stack = copy.copy(properties_dict['fsm_stack'])
                    stack.reverse()
                    selected_el = None
                    for el in stack:
                        if in_node(mousex_g, mousey_g, el.graph_node):
                            selected_el = el
                            break

                    #selected something so load it
                    if selected_el != None and self.fsm_dclick_cb != None:
                        self.fsm_dclick_cb(selected_el)

        ## Function inserted into NodeBox.graph's drawing library to run GraphView code.
        def final_func():
            draw_selected()
            detect_fsm_click()

        self._background_drag(properties_dict)
        properties_dict['MOUSEX'] -= self.dx+self.tx
        properties_dict['MOUSEY'] -= self.dy+self.ty
        g.draw(dx=self.dx+self.tx, dy=self.dy+self.ty, directed=True, traffic=False, user_draw_start=draw_fsm_circles, user_draw_final=final_func)

