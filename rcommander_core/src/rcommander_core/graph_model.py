import roslib; roslib.load_manifest('rcommander_core')
import rospy
import tool_utils as tu
import glob
import os.path as pt
import cPickle as pk
import os
import smach
import outcome_tool as ot
import graph
import sm_thread_runner as smtr
import time

def is_container(node):
    return hasattr(node, 'get_child_name') 

class FSMDocument:
    count = 0
    @staticmethod
    def new_document():
        d = FSMDocument('untitled' + str(FSMDocument.count), False, False)
        FSMDocument.count = FSMDocument.count + 1
        return d

    def __init__(self, filename, modified, real_filename=False):
        self.filename = filename
        self.modified = modified
        self.real_filename = real_filename

    def get_name(self):
        return pt.split(self.filename)[1]

    def get_filename(self):
        return self.filename

    def set_filename(self, fn):
        self.filename = fn

    def has_real_filename(self):
        return self.real_filename

#class UserDataField:
#
#    def __init__(self, name, data_type):
#        self.name = name
#        self.data_type = data_type
#        self.storing_outputs_from = []
#
#class UserData:
#
#    def __init__(self):
#        self.variables = []


class GraphModel:

    #Information about graph connectivity
    EDGES_FILE = 'edges.graph'

    #Misc information about graph itself
    NODES_FILE = 'nodes.graph'

    NODE_RADIUS = 14

    EDGE_LENGTH = 2.

    def __init__(self):

        self.gve = graph.create(depth=True)
        #self.user_data = UserData()
        self.states_dict = {}

        self.document = FSMDocument.new_document()
        self.start_state = None

        self.node = self.gve.node
        self.edge = self.gve.edge

        self.sm_thread = None
        self.status_cb_func = None
        self.last_outcome = None
        #self.add_outcome(tu.InfoStateBase.GLOBAL_NAME)

    def get_start_state(self):
        return self.start_state

    def set_start_state(self, state):
        #if state == tu.InfoStateBase.GLOBAL_NAME or issubclass(self.states_dict[state].__class__, tu.InfoStateBase):
        #if issubclass(self.states_dict[state].__class__, tu.InfoStateBase):
        #    raise RuntimeError("Can\'t make info states start states")
        self.start_state = state

    def set_document(self, document):
        self.document = document

    @staticmethod
    def load(name, robot=None):
        state_pkl_names = glob.glob(pt.join(name, '*.state'))

        gm = GraphModel()
        gm.states_dict = {}

        #Get meta info
        nodes_fn = pt.join(name, GraphModel.NODES_FILE)
        pickle_file = open(nodes_fn, 'r')
        info = pk.load(pickle_file)
        gm.start_state = info['start_state']
        states_to_load = set(info['state_names'])

        #Load individual states
        for fname in state_pkl_names:
            sname = pt.splitext(pt.split(fname)[1])[0]
            if not states_to_load.issuperset([sname]):
                continue

            pickle_file = open(fname, 'r')
            rospy.loginfo('Loading state %s' % sname)
            gm.states_dict[sname] = pk.load(pickle_file)
            gm.gve.add_node(sname, GraphModel.NODE_RADIUS)
            pickle_file.close()

            if is_container(gm.states_dict[sname]):
                print 'FIXME: load_and_recreate might not make sense anymore'
                gm.states_dict[sname] = gm.states_dict[sname].load_and_recreate(name, robot)


        #Reconstruct graph
        edges_filename = pt.join(name, GraphModel.EDGES_FILE)
        edges_pickle_file = open(edges_filename, 'r')
        edges = pk.load(edges_pickle_file)
        edges_pickle_file.close()
        for node1, node2, n1_outcome in edges:
            gm.gve.add_edge(node1, node2, label=n1_outcome, length=GraphModel.EDGE_LENGTH)

        gm.set_document(FSMDocument(name, modified=False, real_filename=True))

        if robot != None:
            for k in gm.states_dict:
                if hasattr(gm.states_dict[k], 'set_robot'): 
                    gm.states_dict[k].set_robot(robot)

        return gm

    def save(self, name):
        rospy.loginfo('GraphModel: saving to %s' % name)
        if not pt.exists(name):
            os.mkdir(name)

        #Save each state
        for state_name in self.states_dict.keys():
            if is_container(self.states_dict[state_name]):
                self.states_dict[state_name].save_child(name)

            state_fname = pt.join(name, state_name) + '.state'
            pickle_file = open(state_fname, 'w')
            pk.dump(self.states_dict[state_name], pickle_file)
            pickle_file.close()

            if is_container(self.states_dict[state_name]):
                print 'document\'s path was', self.states_dict[state_name].document.get_filename()

        #Save connections
        edge_list = []
        for e in self.gve.edges:
            edge_list.append([e.node1.id, e.node2.id, e.label])

        edge_fn = pt.join(name, GraphModel.EDGES_FILE)
        pickle_file = open(edge_fn, 'w')
        pk.dump(edge_list, pickle_file)
        pickle_file.close()

        nodes_fn = pt.join(name, GraphModel.NODES_FILE)
        pickle_file = open(nodes_fn, 'w')
        pk.dump({'start_state': self.start_state, 'state_names': self.states_dict.keys()}, pickle_file)
        pickle_file.close()

        self.document = FSMDocument(name, False, True)

    def create_singleton_statemachine(self, state):
        #print 'FIXME: create singleton state machine needs to be fixed!!!!!'
        #if self.get_start_state() == None:
        #    self.set_start_state(state.name)
        #sm = self.create_state_machine(ignore_start_state=True)
        temp_gm = GraphModel()
        temp_gm.add_node(state)
        temp_gm.set_start_state(state.name)
        return temp_gm.create_state_machine()
        #return temp_gm.create_state_machine(sm.userdata)


    def run(self, name="", state_machine=None, userdata=None):
        #if state_machine == None:
        #    sm = child_gm.create_state_machine(userdata=userdata)
        #else:
        #    sm = state_machine

        sm = state_machine
        sm.register_transition_cb(self._state_machine_transition_cb)
        sm.register_start_cb(self._state_machine_start_cb)
        sm.register_termination_cb(self._state_machine_termination_cb)

        rthread = smtr.ThreadRunSM(name, sm)
        rthread.register_termination_cb(self._sm_thread_termination_cb)

        self.sm_thread = {}
        self.sm_thread['run_sm'] = rthread
        self.sm_thread['preempted'] = None
        self.sm_thread['current_states'] = None
        self.sm_thread['outcome'] = None
        rthread.start()
        return rthread

    def preempt(self):
        if self.is_running():
            self.sm_thread['run_sm'].preempt()
            self.sm_thread['preempted'] = time.time()
            self.sm_thread['run_sm'].preempt()

    def is_running(self):
        return self.sm_thread != None

    def register_status_cb(self, func):
        self.status_cb_func = func

    def _sm_thread_termination_cb(self, exception):
        print 'THREAD TERMINATED CALLED'
        #print 'thread terminated'
        if exception != None:
            if self.status_cb_func != None:
                self.status_cb_func('Error: %s' % str(exception))

        elif self.sm_thread['preempted'] != None:
            if self.status_cb_func != None:
                self.status_cb_func('%s stopped.' % (self.document.get_name()))

        self.sm_thread = None

    def _state_machine_transition_cb(self, user_data, active_states):
        self.sm_thread['current_states'] = active_states
        if self.status_cb_func != None:
            self.status_cb_func('At state %s' % (active_states[0]))

        #print '_state_machine_transition_cb: called back with', args, kwargs
        #print '_state_machine_transition_cb: _data', args[0]._data
        #print '_state_machine_transition_cb: _locks', args[0]._locks
        #print '_state_machine_transition_cb: args[1]', args[1]
        #print '_state_machine_transition_cb initial states', self.sm_thread['run_sm'].sm.get_initial_states()
        #print '_state_machine_transition_cb active states', self.sm_thread['run_sm'].sm.get_active_states()

    def _state_machine_start_cb(self, userdata, initial_states):
        self.sm_thread['current_states'] = initial_states
        if self.status_cb_func != None:
            self.status_cb_func('At state %s.' % (initial_states[0]))
        #print '_state_machine_start_cb userdata      ', userdata
        #print '_state_machine_start_cb initial_states', initial_states

    def get_last_outcome(self):
        return self.last_outcome

    def _state_machine_termination_cb(self, userdata, terminal_states, container_outcome):
        print 'state machine termination CALLED'
        self.sm_thread['current_states'] = terminal_states
        self.sm_thread['outcome'] = container_outcome
        self.last_outcome = [container_outcome, time.time()]

        if self.status_cb_func != None:
            self.status_cb_func('Stopped with outcome %s' % container_outcome)

        #print '_state_machine_termination_cb userdata         ', userdata
        #print '_state_machine_termination_cb terminal_states  ', terminal_states
        #print '_state_machine_termination_cb container_outcome', container_outcome

    def outputs_of_type(self, class_filter):
        filtered_output_variables = []
        for node_name in self.real_states():
            node = self.get_state(node_name)
            output_names = node.output_names()
            for output_name in output_names:
                if issubclass(node.output_type(output_name), class_filter):
                    filtered_output_variables.append(output_name)
        return filtered_output_variables

    def create_state_machine(self, userdata=None, ignore_start_state=False):
        #print '>>>>>>>>>>>>>> create_state_machine', userdata
        sm = smach.StateMachine(outcomes = self.outcomes())
        #print 'sm userdata', sm.userdata, self.outcomes()

        #Deprecated, global nodes are being replaced by nodes with outputs
        #for global_node_name in self.global_nodes(None):
        #    global_node = self.states_dict[global_node_name]
        #    global_variable_name = global_node.get_name()
        #    value = global_node.get_info()
        #    exec_str = "sm.userdata.%s = value" % global_variable_name
        #    print 'executing', exec_str
        #    exec exec_str

        # Copy over input userdata into current state machine so that nodes contained
        # would have access
        if userdata != None:
            #print 'userdata keys', userdata.keys()
            for key in userdata.keys():
                exec ("sm.userdata.%s = userdata.%s" % (key, key))
                #print 'copying key', key
                exec ("print 'data in key is', sm.userdata.%s" % (key))

        with sm:
            for node_name in self.real_states():
                node = self.states_dict[node_name]
                #if issubclass(node.__class__, tu.InfoStateBase):
                #    continue

                #if issubclass(node.__class__, tu.SimpleStateBase):
                node_smach = node.get_smach_state()

                #if issubclass(node.__class__, tu.StateBase):
                #    node_smach = node

                transitions = {}
                #print node_name, 'input keys', node_smach.get_registered_input_keys()
                for e in self.gve.node(node_name).edges:
                    if e.node1.id == node_name:
                        transitions[e.label] = e.node2.id
                        #print e.node1.id, e.label, e.node2.id

                input_set = set(node_smach.get_registered_input_keys())
                output_set = set(node_smach.get_registered_output_keys())
                if len(input_set.intersection(output_set)) > 0:
                    raise RuntimeError('Input keys has the same name as output_keys.')

                remapping = {}
                for input_key in node_smach.get_registered_input_keys():
                    #print 'source for variable', input_key, 'is', node.remapping_for(input_key)
                    remapping[input_key] = node.remapping_for(input_key)
                
                #We assume that each output is to a SEPARATE variable
                #with the same name as the node, we only need the node's
                #name. Ex.  OUTPUT: GLOBAL
                #           NODE_NAME: NODE_NAME
                for output_key in node_smach.get_registered_output_keys():
                    remapping[output_key] = output_key

                #print '>> node_name', node_name, 'transitions', transitions, 'remapping', remapping
                smach.StateMachine.add(node_name, node_smach, transitions=transitions, remapping=remapping)

        if ignore_start_state:
            #print 'IGNORING START STATE'
            return sm

        if self.start_state == None:
            raise RuntimeError('No start state set.')
        #print 'create_state_machine start state is', self.start_state
        sm.set_initial_state([self.start_state])
        #print '<<<<<<<<<<<<<<'
        return sm

    #@return a list of node names and outcomes
    #        e.g. [[edge_name, node_name], ...]
    def current_children_of(self, node_name):
        ret_list = []
        for edge in self.gve.node(node_name).edges:
            if edge.node1.id != node_name:
                continue
            ret_list.append([edge.label, edge.node2.id])
        return ret_list

    ##
    # All nodes that has executable code (not outcomes/EmptyState)
    def real_states(self):
        noc = []
        for node_name in self.states_dict.keys():
            if self.states_dict[node_name].__class__ != tu.EmptyState:
                noc.append(node_name)
        return noc

    def outcomes(self):
        #all empty states are outcomes
        oc = []
        for node_name in self.states_dict.keys():
            if self.states_dict[node_name].__class__ == tu.EmptyState:
                oc.append(node_name)
        return oc

    def pop_state(self, node_name):
        return self.states_dict.pop(node_name)

    def get_state(self, node_name):
        #print self.states_dict.keys()
        return self.states_dict[node_name]

    def set_state(self, node_name, state):
        self.states_dict[node_name] = state

    def replace_node(self, new_node, old_node_name):
        self.states_dict.pop(old_node_name)
        self.states_dict[new_node.get_name()] = new_node
        new_node_name = new_node.get_name()

        #if the new node has the same name (possible to have different connections)
        #If the node is of a different name

        if new_node_name != old_node_name:
            self.gve.add_node(new_node_name, self.NODE_RADIUS)

        #for each existing connection
        new_smach_node = new_node.get_smach_state()
        new_outcomes = new_smach_node.get_registered_outcomes()
        for e in self.gve.node(old_node_name).edges:
        #   if it is an outcome in the new node
            if e.label in new_outcomes:
        #       if it has a different source, remove it and add a new one
                if e.node1.id == old_node_name:
                    self.gve.remove_edge(e.node1.id, e.node2.id, label=e.label)
                    self.gve.add_edge(new_node_name, e.node2.id, label=e.label, length=GraphModel.EDGE_LENGTH)
                elif e.node2.id == old_node_name:
                    self.gve.remove_edge(e.node1.id, e.node2.id, label=e.label)
                    self.gve.add_edge(e.node1.id, new_node_name, label=e.label, length=GraphModel.EDGE_LENGTH)
        #       if it has the same source ignore
        #   if it is not an outcome in our new node
            else:
                if e.node1.id == old_node_name:
                    print 'removing edge', e.node1.id, e.node2.id
                    self.gve.remove_edge(e.node1.id, e.node2.id, label=e.label)
                    if not self.is_modifiable(e.node2.id) and len(e.node2.edges) < 1:
                        self.gve.remove_node(e.node2.id)
                        self.states_dict.pop(e.node2.id)
                else:
                    self.gve.remove_edge(e.node1.id, e.node2.id, label=e.label)
                    self.gve.add_edge(e.node1.id, new_node_name, label=e.label, length=GraphModel.EDGE_LENGTH)
        #   delete it   

        if new_node_name != old_node_name:
            self.gve.remove_node(old_node_name)
                
        #for each new outcome
        #   if we don't have an edge for it, create that edge & its temporary node
        self.restore_node_consistency(new_node.get_name())

    def connectable_nodes(self, node_name, outcome):
        #can't connect to
        #  temporary nodes already connected whose name is not current outcome
        allowed_nodes = []
        #outcome_name = self._outcome_name(node_name, outcome)
        #allowed_nodes.append(outcome_name)
        for k in self.states_dict.keys():
            #If it's a temporary node and does not have the name of this outcome
            #if not self.is_modifiable(k) and k != outcome:
            if (not self.is_modifiable(k)) and (not self._is_type(k, outcome)):
                continue
            #ignore our own name
            if node_name == k:
                continue

            allowed_nodes.append(k)

        if node_name == None:
            allowed_nodes.append(self._create_outcome_name(outcome))
            allowed_nodes = list(set(allowed_nodes))

        return allowed_nodes

    def _create_outcome_name(self, outcome):
        idx = 0
        name = "%s%d" % (outcome, idx)
        while self.states_dict.has_key(name):
            idx = idx + 1
            name = "%s%d" % (outcome, idx)
        return name

    def _is_type(self, state_name, outcome):
        r = state_name.find(outcome)
        if r < 0:
            return False
        else:
            return True

    def add_node(self, node):
        if self.states_dict.has_key(node.get_name()):
            node.set_name(node.get_name() + '_dup')
            #raise RuntimeError('Already has node of the same name.  This case should not happen.')

        #if this is a non-container node 
        if not hasattr(node, 'get_child_name') or \
                not self.states_dict.has_key(node.get_child_name()):

            #Link this node to all its outcomes
            self.gve.add_node(node.get_name(), radius=self.NODE_RADIUS)
            self.states_dict[node.get_name()] = node

            #Check all outcomes and make new nodes if needed
            smach_node = node.get_smach_state()

            #For each outcome
            for outcome in smach_node.get_registered_outcomes():
                #Create an empty state and add an edge to it
                outcome_name = self._create_outcome_name(outcome)
                self.states_dict[outcome_name] = tu.EmptyState(outcome_name, temporary=True)
                self.gve.add_node(outcome_name, radius=self.NODE_RADIUS)
                self._add_edge(node.get_name(), outcome_name, outcome)

        else:
            #If this node has a child node we replace its child node with it instead of performing an add
            print 'FIXME: NESTING STATE MACHINES DOESN\'T WORK YET'
            self.replace_node(node, node.get_child_name())


    #def add_node_smach(self, smach_node):
    #    if self.states_dict.has_key(smach_node.name):
    #        raise RuntimeError('Already has node of the same name.  This case should not happen.')

    #    #if this is a regular singleton node
    #    if not hasattr(smach_node, 'get_child_name') or not self.states_dict.has_key(smach_node.get_child_name()):
    #        #Link this node to all its outcomes
    #        self.gve.add_node(smach_node.name, radius=self.NODE_RADIUS)
    #        self.states_dict[smach_node.name] = smach_node
    #        #print 'adding node', smach_node.name, 'with outcomes', smach_node.get_registered_outcomes()

    #        for outcome in smach_node.get_registered_outcomes():
    #            #print smach_node.name, outcome
    #            #outcome_name = self._outcome_name(smach_node.name, outcome)
    #            if outcome == tu.InfoStateBase.GLOBAL_NAME:
    #                outcome_name = outcome
    #            else:
    #                outcome_name = self._create_outcome_name(outcome)

    #            #if not self.states_dict.has_key(outcome):
    #            self.states_dict[outcome_name] = tu.EmptyState(outcome_name, temporary=True)
    #            self.gve.add_node(outcome_name, radius=self.NODE_RADIUS)
    #            #self.gve.add_edge(smach_node.name, outcome)
    #            self._add_edge(smach_node.name, outcome_name, outcome)
    #            #print '>>> adding edge between', smach_node.name, 'and', outcome_name, 'with label', outcome

    #    #If this node has a child node we replace its child node instead of performing an add
    #    else:
    #        self.replace_node(smach_node, smach_node.get_child_name())
    #        #self.restore_node_consistency(smach_node.name)

    #def add_node(self, node):
    #    #if issubclass(node, tu.StateBase):
    #    #    self.add_node_smach(node)
    #    #elif issubclass(node, tu.SimpleStateBase):
    #    self.add_node2(node)

    def add_outcome(self, outcome_name):
        self.gve.add_node(outcome_name, radius=self.NODE_RADIUS)
        self.states_dict[outcome_name] = tu.EmptyState(outcome_name, False)

    def delete_node(self, node_name):
        node_obj = self.gve.node(node_name)
        children_edges = []
        parent_edges = []

        print 'deleting', node_name
        #Separate edges from parents and edges to children
        for cn in node_obj.links:
            for edge in self.gve.all_edges_between(node_name, cn.id):
                if (edge.node1.id == node_name) and (edge.node2.id == node_name):
                    raise Exception('Self link detected on node %s! This isn\'t supposed to happen.' % node_name)
                if edge.node1.id == node_name:
                    children_edges.append(edge)
                elif edge.node2.id == node_name:
                    parent_edges.append(edge)

        #Remove placeholder children nodes
        filtered_children_edges = []
        for e in children_edges:
            # If the connected node is not modifiable (i.e. a temporary added
            # node) and it doesn't have any other parents.
            #print 'child edge', e.label, e.node1.id, e.node2.id
            if not self.is_modifiable(e.node2.id) and len(e.node2.edges) <= 1:
                #print (not self.is_modifiable(e.node2.id)), (len(e.node2.edges) <= 1)
                #Delete it
                self.gve.remove_edge(node_name, e.node2.id, e.label)
                self.gve.remove_node(e.node2.id)
                self.states_dict.pop(e.node2.id)
            else:
                filtered_children_edges.append(e)

        #If we have one or more than one parent
        if len(parent_edges) >= 1:
            #Pick the first parent
            parent_node_id = parent_edges[0].node1.id
            parent_node = self.gve.node(parent_node_id)
            print 'picked parent', parent_node_id

            #Create an index of siblings
            parents_children = {}
            for parent_outcome_name, sibling_node_name in self.current_children_of(parent_node_id):
                parents_children[parent_outcome_name] = sibling_node_name
            print 'siblings', parents_children

            #For each child edge of ours
            for edge in filtered_children_edges:
                print 'processing child edge', edge.node1.id, edge.label, edge.node2.id
                #node_outcome_name = edge.outcome
                node_outcome_name = edge.label

                #if parent has a similar outcome connected to a temporary node
                if parents_children.has_key(node_outcome_name):
                    parent_outcome_node = parents_children[node_outcome_name]
                    #If parent outcome is connected to a temporary node, replace the temporary node with link to us
                    if not self.is_modifiable(parent_outcome_node):
                        #connect this child node to parent
                        self.gve.remove_edge(parent_node_id, parent_outcome_node, label=node_outcome_name)
                        self.gve.add_edge(parent_node_id, edge.node2.id, label=node_outcome_name, length=GraphModel.EDGE_LENGTH)
                        #e = self.gve.edge(parent_node_id, node_outcome_name)
                        #e.outcome = node_outcome_name
                        #delete parent's temporary node if it is now unconnected
                        if len(self.gve.node(parent_outcome_node).edges) < 1:
                            self.gve.remove_node(parent_outcome_node)
                            self.states_dict.pop(parent_outcome_node)
                #remove this edge
                self.gve.remove_edge(edge.node1.id, edge.node2.id, edge.label)

        #If no parents
        elif len(parent_edges) == 0:
            #just remove children edges
            for e in filtered_children_edges:
                self.gve.remove_edge(node_name, e.node2.id, label=e.label)

        #Remove edge from parents, and restore consistency for parent nodes
        for parent_edge in parent_edges:
            self.gve.remove_edge(parent_edge.node1.id, parent_edge.node2.id, parent_edge.label)
            self.restore_node_consistency(parent_edge.node1.id)

        self.gve.remove_node(node_name)
        self.states_dict.pop(node_name)
        if self.start_state == node_name:
            self.start_state = None

    # For each registered outcome, make sure there exists an edge.  If no
    # edge exists, create it.
    def restore_node_consistency(self, node_name):
        #print 'restoring consistency of node', node_name

        clist = self.current_children_of(node_name)
        cdict = {}
        #print 'outcomes that we have links for'
        for outcome_name, nn in clist:
            cdict[outcome_name] = nn
            #print outcome_name, nn

        print 'current children of', node_name, clist
        print 'registed outcomes are', self.states_dict[node_name].get_smach_state().get_registered_outcomes()

        registered_outcomes = self.states_dict[node_name].get_smach_state().get_registered_outcomes()

        #Remove things that are no longer outcomes
        for outcome in cdict.keys():
            if not (outcome in registered_outcomes):
                self.gve.remove_edge(node_name, cdict[outcome], outcome)
                if (not self.is_modifiable(cdict[outcome])) and len(self.gve.node(cdict[outcome]).edges) < 1:
                    self.gve.remove_node(cdict[outcome])
                    self.states_dict.pop(cdict[outcome])

        #print self.states_dict[node_name].__class__
        #print 'outcomes that we need', self.states_dict[node_name].get_registered_outcomes()

        for outcome in registered_outcomes:
            if not cdict.has_key(outcome):
                #print 'outcome', outcome, 'is missing. restoring connection'
                new_outcome_name = self._create_outcome_name(outcome)
                self._add_temporary_outcome(new_outcome_name)
                self._add_edge(node_name, new_outcome_name, outcome)

    def _add_temporary_outcome(self, outcome):
        self.states_dict[outcome] = tu.EmptyState(outcome, temporary=True)
        self.gve.add_node(outcome, self.NODE_RADIUS)

    def is_modifiable(self, node_name):
        if (self.states_dict[node_name].__class__ == tu.EmptyState) and self.states_dict[node_name].temporary:
            return False
        else:
            return True

    def _add_edge(self, n1, n2, n1_outcome):
        if not self.states_dict.has_key(n1) or not self.states_dict.has_key(n2):
            raise RuntimeError('One of the specified nodes does not exist.  Can\'t add edge.')

        if self.gve.edge(n1, n2, n1_outcome) != None:
            rospy.loginfo("Edge between %s and %s exists, ignoring connnection request" % (n1, n2))
            return False

        #Don't add edges to "temporary" nodes
        if n1_outcome == None and self.is_modifiable(n2):
            raise RuntimeError('Must specify outcome as goal node is not a temporary node.')

        self.gve.add_edge(n1, n2, label=n1_outcome, length=GraphModel.EDGE_LENGTH)
        #print 'actually added edge'
        #self.gve.edge(n1, n2).outcome = n1_outcome
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
            self.gve.remove_edge(edge.node1.id, edge.node2.id, e.label)
            return True

    def connection_changed(self, node_name, outcome_name, new_node):
        #node is not valid or hasn't been created yet

        if node_name == None:
            return

        if not self.states_dict.has_key(new_node):
            raise RuntimeError('Doesn\'t have state: %s' % new_node)
        #self.get_state(node_name).outcome_choices[outcome_name] = new_node

        #find the old edge
        old_edge = None
        for edge in self.gve.node(node_name).edges:
            #if edge.outcome == outcome_name and edge.node1.id == node_name:
            #print 'edge', edge.node1.id, edge.node2.id, edge.label
            if edge.label == outcome_name and edge.node1.id == node_name:
                if old_edge != None:
                    raise RuntimeError('Two edges detected for one outcome named %s. %s -> %s and %s -> %s' % (outcome_name, old_edge.node1.id, old_edge.node2.id, edge.node1.id, edge.node2.id))
                old_edge = edge

        #print node_name, outcome_name, new_node
        if old_edge.node2.id == new_node:
            return

        #print 'connection_changed', node_name, outcome_name, new_node
        #remove the old connection
        self.gve.remove_edge(node_name, old_edge.node2.id, label=old_edge.label)
        #remove the old node if it's temporary 
        #print 'The old edge is named', old_edge.node2.id, not self.is_modifiable(old_edge.node2.id)
        if not self.is_modifiable(old_edge.node2.id):
            #and not connected
            #print 'it has this many edges', len(self.gve.node(old_edge.node2.id).edges)
            if len(self.gve.node(old_edge.node2.id).edges) <= 0:
                self.gve.remove_node(old_edge.node2.id)
                self.states_dict.pop(old_edge.node2.id)

        #add new connection
        if self.gve.node(new_node) == None:
            #print 'recreated node', new_node
            self.states_dict[new_node] = tu.EmptyState(new_node, temporary=True)
            self.gve.add_node(new_node, self.NODE_RADIUS)
        #print 'calling add_edge with a', node_name, 'b', new_node, 'outcome', outcome_name
        self._add_edge(node_name, new_node, outcome_name)









        #print 'THE KEYS ARE'
        #for k in self.states_dict.keys():
        #    print k

        #print 'OUR NEW EDGES ARE'
        #for e in self.gve.node(node_name).edges:
        #    print e.node1.id, e.node2.id, e.label
    #def delete_node_old(self, node_name):
    #    #temporary nodes are only removable when the state transitions are linked to something else
    #    if not self.is_modifiable(node_name):
    #        return 

    #    #Find parents and children
    #    node_obj = self.gve.node(node_name)
    #    children_edges = []
    #    parent_edges = []
    #    for cn in node_obj.links:
    #        edge = self.gve.edge(node_name, cn.id)
    #        if (edge.node1.id == node_name) and (edge.node2.id == node_name):
    #            raise Exception('Self link detected on node %s! This isn\'t supposed to happen.' % node_name)
    #        if edge.node1.id == node_name:
    #            children_edges.append(edge)
    #        elif edge.node2.id == node_name:
    #            parent_edges.append(edge)

    #    #Remove placeholder children nodes
    #    filtered_children_edges = []
    #    for e in children_edges:
    #        if not self.is_modifiable(e.node2.id) and len(e.node2.edges) <= 1:
    #            self.gve.remove_edge(node_name, e.node2.id)
    #            self.gve.remove_node(e.node2.id)
    #            self.states_dict.pop(e.node2.id)
    #        else:
    #            filtered_children_edges.append(e)

    #    new_selected_node = None
    #    #If we have one or more than one parent
    #    if len(parent_edges) >= 1:
    #        #Point edges on children to first parent
    #        parent_node_id = parent_edges[0].node1.id
    #        for e in filtered_children_edges:
    #            self.gve.remove_edge(node_name, e.node2.id)
    #            self.gve.add_edge(parent_node_id, e.node2.id)
    #        new_selected_node = parent_node_id

    #        #On each one of the parent, check to see if we are the terminal state
    #        for e in parent_edges:
    #            parent_id = e.node1.id
    #            outcome_set = set(self.get_state(parent_id).get_registered_outcomes())
    #            if e.outcome in outcome_set:
    #                self.connection_changed(parent_id, e.outcome, e.outcome)
    #                #jjself.states_dict[e.outcome] = tu.EmptyState(e.outcome, temporary=True)
    #                #self.gve.add_node(e.outcome)
    #                #self._add_edge(parent_id, e.outcome, e.outcome)

    #    #If no parents
    #    elif len(parent_edges) == 0:
    #        #just remove children edges
    #        for e in filtered_children_edges:
    #            self.gve.remove_edge(node_name, e.node2.id)

    #        if len(filtered_children_edges) > 1:
    #            new_selected_node = filtered_children_edges[0].node2.id
    #        else:
    #            if len(self.gve.nodes) > 0:
    #                new_selected_node = self.gve.nodes[0].id
    #            else:
    #                new_selected_node = 'start'

    #    self.gve.remove_node(node_name)
    #    self.states_dict.pop(node_name)
    #    return new_selected_node

        #if new_node_name != old_node_name:
        #    self.gve.add_node(new_node_name, radius=self.NODE_RADIUS)
        #    #remove edges to old node, add edges that point to the new node
        #    for e in self.gve.node(old_node_name).edges:
        #        self.gve.remove_edge(e.node1.id, e.node2.id, label=e.label)
        #        if e.node1.id == old_node_name:
        #            self.gve.add_edge(new_node_name, e.node2.id, label=e.label, length=GraphModel.EDGE_LENGTH)
        #        else:
        #            self.gve.add_edge(e.node1.id, new_node_name, label=e.label, length=GraphModel.EDGE_LENGTH)
        #    self.gve.remove_node(old_node_name)

    #def _outcome_name(self, node_name, outcome):
    #    return node_name + '_' + outcome


