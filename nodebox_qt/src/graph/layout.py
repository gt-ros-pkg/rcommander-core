from random import random
from math import pi, sin, cos
from math import sqrt
import scipy.spatial as sp
import scipy.spatial.distance as spd
import numpy as np
import nodebox_springlayout as nbs

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        
INFINITY = 1e20

##### GRAPH LAYOUT ###################################################################################

class layout(object):
    
    """ Graph visualizer that calculates relative node positions.
    """
    
    def __init__(self, graph, iterations=1000):
        
        self.type = None
        self.graph = graph
        self.i = 0
        self.n = iterations
        
        self.__bounds = None

    def copy(self, graph):
        
        """ Returns a copy of the layout for the given graph.
        """
        
        l = self.__class__(graph, self.n)
        l.i = 0
        return l

    def prepare(self):
        
        for n in self.graph.nodes:
            n.vx = 0
            n.vy = 0
            n.force = Point(0,0)    
    
    def _bounds(self):
        
        min = Point(float( INFINITY), float( INFINITY))
        max = Point(float(-INFINITY), float(-INFINITY))
        for n in self.graph.nodes:
            if (n.vx < min.x): min.x = n.vx
            if (n.vy < min.y): min.y = n.vy
            if (n.vx > max.x): max.x = n.vx
            if (n.vy > max.y): max.y = n.vy
      
        return (min, max)

    bounds = property(_bounds)
    
    def _get_done(self):
        
        if self.i >= self.n: 
            return True
        return False
        
    done = property(_get_done)
    
    def iterate(self):
        
        self.i += 1
        return self.done
        
    def solve(self):
        
        while not self.done: 
            self.iterate()
            
    def reset(self):
        
        self.i = 0
        
    def refresh(self):
        
        self.i = self.n / 2

##### GRAPH CIRCLE LAYOUT ############################################################################

class circle_layout(layout):
    
    """ Simple layout with nodes arranged on one or more circles.
    """
    
    def __init__(self, graph, iterations=100):
    
        layout.__init__(self, graph, iterations)
        self.type = "circle"    
        
        self.r = 8    # outer circle radius
        self.c = 2    # number of circles
        self.a = pi/2 # starting angle
    
    def _get_orbits(self): return self.c
    def _set_orbits(self, v): self.c = v
    orbits = property(_get_orbits, _set_orbits)
    
    def copy(self, graph):
        
        l = layout.copy(self, graph)
        l.r, l.c, l.a = self.r, self.c, self.a
        return l
    
    def iterate(self):
        
        if len(self.graph.nodes) == 1: return
        
        # Nodes are sorted by betweenness centrality.
        # Node with a high centrality are on the inner circles.
        # There are logarithmically more nodes on the outer shells.
        circles = []
        nodes = self.graph.nodes_by_traffic(treshold=-1)
        for i in range(self.c):
            t = 1.0 / (self.c-i)**2
            slice = int(t * len(self.graph.nodes))
            slice = max(1, slice)
            circles.append(nodes[:slice])
            nodes = nodes[slice:]

        node_radius = self.graph.nodes[0].__class__(None).r

        i = 0
        for circle in circles:
            i += 1

            # Circle radii expand each iteration.
            # Inner circles have a smaller radius.
            r = self.r * sin(pi/2 * float(self.i) / self.n)
            r *= float(i)/len(circles)
            
            # Calculate circle circumference. 
            # Node diameter / circumference determine how many nodes fit on the shell.
            C = self.r*self.graph.d * 2*pi * float(i)/len(circles)
            s = node_radius*2 / C * 2
            
            a = self.a
            t = min(2*pi*s, 2*pi/len(circle))
            for n in circle:
                n.vx = r * cos(a)
                n.vy = r * sin(a)
                a += t

        #r = self.r * sin(float(self.i) / self.n * pi/2)
        #a = 0
        #i = pi*2 / len(self.graph.nodes)
        #for n in self.graph.nodes:
        #    n.vx = r * cos(a)
        #    n.vy = r * sin(a)
        #    a += i 
            
        layout.iterate(self)
        
    def solve(self):
        
        self.i = self.n
        self.iterate()

##### GRAPH SPRING LAYOUT ############################################################################

class spring_layout(layout):
    
    """ A force-based layout in which edges are regarded as springs.
    http://snipplr.com/view/1950/graph-javascript-framework-version-001/
    """
    
    def __init__(self, graph, iterations=1000):
        
        layout.__init__(self, graph, iterations)    
        self.type = "spring"

        self.k = 2    # force strength
        self.m = 0.01 # force multiplier
        self.w = 15   # edge weight multiplier
        self.d = 0.5  # maximum vertex movement
        self.r = 15   # maximum repulsive force radius
        self.r0 = 1  # repulsive force radius for nodes without edges
    
    def tweak(self, k=2, m=0.01, w=15, d=0.5, r=15):
        self.k = k
        self.m = m
        self.w = w
        self.d = d
        self.r = r
    
    def _get_force(self): return self.m
    def _set_force(self, v): self.m = v
    force = property(_get_force, _set_force)

    def _get_repulsion(self): return self.r
    def _set_repulsion(self, v): self.r = v
    repulsion = property(_get_repulsion, _set_repulsion)
    
    def copy(self, graph):
        
        l = layout.copy(self, graph)
        l.k, l.m, l.d, l.r = self.k, self.m, self.d, self.r
        return l
    
    def iterate(self):
        #xs = []
        #ys = []
        #for i in range(num_nodes):
        #    node = self.graph.nodes[i]
        #    xs.append(node.vx)
        #    ys.append(node.vy)
        #xs = np.matrix(xs)
        #ys = np.matrix(ys)

        #dists = spd.squareform(spd.pdist(np.row_stack((xs,ys)).T, 'euclidean'))
        #F = self.k**2 / np.power(dists,2)

        ##Calc forces
        #dists = np.matrix(spd.squareform(spd.pdist(pos.T, 'euclidean')))
        #rand = np.reshape(np.random.randn(num_nodes*num_nodes), (num_nodes, num_nodes))
        #r01, c01 = np.where(dists < .01)
        #dists[r01.A1, c01.A1] = rand[r01.A1, c01.A1]

        #F = self.k**2 / np.power(dists, 2)

        ##Calc dirs
        #diffs = np.matrix(np.zeros((2*num_nodes, num_nodes)))
        #for i in range(num_nodes): 
        #    diffs[2*i:2*i+2,:] = (pos - pos[:,i])

        ##Merge forces and dir
        #xs = np.multiply(F, diffs[range(0,diffs.shape[0], 2), :])
        #ys = np.multiply(F, diffs[range(1,diffs.shape[0], 2), :])

        #for i in range(num_nodes): 
        #    node = self.graph.nodes[i]
        #    other_nodes_idx = range(num_nodes)
        #    other_nodes_idx.pop(i)
        #    node.force.x = np.sum(xs[i,other_nodes_idx])
        #    node.force.y = np.sum(ys[i,other_nodes_idx])
        
        #xs = diffs[, :]

        #    #add some noise

        #too_small_idx = np.where(dists[i,:] < .01)[1].A1
        #diffs[2*i:2*i+2, too_small_idx] = random()*.1 + .1

        #    #limit influence
        #    in_range = np.where(dists[i,:] < self.r)[1].A1
        #    F_d = np.multiply(diffs[2*i:2*i+2,:], F[i,:])

        #     f = self.k**2 / np.power(dists[i,:], 2)
        #     #print diffs[2*i:2*i+2,:].shape, f.shape

        #     forces[:, :i] = forces[:, :i] + F[:, :i]
        #     forces[:, i+1:] = forces[:, i+1:] + F[:, i+1:]
        #     forces[:,i] = forces[:,i] - np.sum(F, 1)

        # for i in range(num_nodes):
        #     self.graph.nodes[i].force.x = forces[0,i]
        #     self.graph.nodes[i].force.y = forces[1,i]


        #if len(pos) > 1:
        #    tree = sp.KDTree(np.array(pos))
        #    neighbor_result = tree.query_ball_tree(tree, self.r)
            #for idx, neighbors in enumerate(neighbor_result):
            #    n1 = self.graph.nodes[idx]
            #    for jdx in neighbors:
            #        self._repulse(n1, self.graph.nodes[jdx])

        #for idx, jdx in tree.query(self.r):
        #    self._repulse(self.graph.nodes[idx], self.graph.nodes[jdx])

        # Forces on all nodes due to node-node repulsions.
        fast = True
        if fast:
            #print 'ITERRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRr'
            num_nodes = len(self.graph.nodes)
            pos = []
            elen = []
            for i in range(num_nodes):
                node = self.graph.nodes[i]
                pos.append([node.vx, node.vy])
                elen.append(len(node.edges))
            pos = np.matrix(pos).T
            #print 'calling with param\n', pos
            #print 'elen', elen
            #print 'k', self.k, 'r', self.r, 'r0', self.r0
            forces = nbs.spring_force(pos, np.matrix(elen, dtype='float'), self.k, self.r, self.r0)
            for i in range(num_nodes):
                node = self.graph.nodes[i]
                node.force.x = forces[0,i]
                node.force.y = forces[1,i]
        #else:
            #for i in range(len(self.graph.nodes)):
            #    n1 = self.graph.nodes[i]
            #    for j in range(i+1, len(self.graph.nodes)):
            #        n2 = self.graph.nodes[j]             
            #        self._repulse(n1, n2)
            #print 'nodes are'
            #temp = []
            #for n in self.graph.nodes:
            #    temp.append([n.force.x, n.force.y])

            #print 'forces'
            #print forces
            #print 'repulse'
            #print np.matrix(temp).T


        # Forces on nodes due to edge attractions.
        for e in self.graph.edges:
            self._attract(e.node1, e.node2, self.w*e.weight, 1.0/e.length)
            
        # Move by given force.
        for n in self.graph.nodes:
            vx = max(-self.d, min(self.m * n.force.x, self.d))
            vy = max(-self.d, min(self.m * n.force.y, self.d))
            n.vx += vx
            n.vy += vy
            n.force.x = 0
            n.force.y = 0
        
        return layout.iterate(self)
    
    def _distance(self, n1, n2):
        
        dx = n2.vx - n1.vx
        dy = n2.vy - n1.vy
        d2 = dx**2 + dy**2       

        if d2 < 0.01:
            dx = random()*0.1 + 0.1
            dy = random()*0.1 + 0.1
            d2 = dx**2 + dy**2
            #print dx, dy, d2
            
        d = sqrt(d2)
        
        return dx, dy, d
    
    def _repulse(self, n1, n2):

        if len(n1.edges) < 1 or len(n2.edges) < 1:
            r = self.r0
        else:
            r = self.r

        dx, dy, d = self._distance(n1, n2)
        
        if d < r:
            f = self.k**2 / d**2
            n2.force.x += f * dx
            n2.force.y += f * dy
            n1.force.x -= f * dx
            n1.force.y -= f * dy
        
    def _attract(self, n1, n2, k=0, length=1.0):
        
        dx, dy, d = self._distance(n1, n2)
        d = min(d, self.r)
        
        # Take the edge's weight (k) into account.
        f = (d**2 - self.k**2) / self.k * length
        f *= k * 0.5 + 1
        f /= d
        
        n2.force.x -= f * dx
        n2.force.y -= f * dy
        n1.force.x += f * dx
        n1.force.y += f * dy
