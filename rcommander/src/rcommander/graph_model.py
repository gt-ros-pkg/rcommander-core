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
import shutil

##
# Checks to see if a node can contain other nodes
def is_container(node):
    return hasattr(node, 'get_child_name') 

## Represents a FSM (finite state machine document), contains logic for
# managing filenames.
class FSMDocument:
    count = 0
    
    ## Factory method to create a new blank document object.
    @staticmethod
    def new_document():
        d = FSMDocument('untitled' + str(FSMDocument.count), False, False)
        FSMDocument.count = FSMDocument.count + 1
        return d

    ## Constructor
    # @param filename Filename where this document resides (string).
    # @param modified Whether this document has been modified since last loaded
    #                   (bool)
    # @param real_filename Whether this is a tentative name or if the file
    #                   exist on disk (bool)
    def __init__(self, filename, modified, real_filename=False):
        self.filename = filename
        self.modified = modified
        self.real_filename = real_filename

    ## Getter for just the filename
    def get_name(self):
        return pt.split(self.filename)[1]

    ## Getter for the full qualified path of filename
    def get_filename(self):
        return self.filename

    ## Setter for filename
    def set_filename(self, fn):
        self.filename = fn

    ## Getter for real_filename flag
    def has_real_filename(self):
        return self.real_filename

## Model (as in MVP) for graphs in ROS Commander
# Responsible for:
#
# 1) Keeping its internal model (self.states_dict) in sync with Nodebox.graph's
# internal representation of graphs (self.gve).
#
# 2) Executing SMACH state machines, keeping track of execution status.
class GraphModel:

    #Information about graph connectivity
    EDGES_FILE = 'edges.graph'

    #Misc information about graph itself
    NODES_FILE = 'nodes.graph'

    NODE_RADIUS = 16

    OUTCOME_NODE_RADIUS = 10

    EDGE_LENGTH = 2.

    ## Constructor
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
        self.transition_cb = None
        self.last_outcome = None
        self.start_cb = None

    ## Getter for the current start state of this model
    def get_start_state(self):
        return self.start_state

    ## Setter for current start state
    def set_start_state(self, state):
        self.start_state = state

    ## Sets the FSMDocument associated with this model
    def set_document(self, document):
        self.document = document

    ## Gets the FSMDocument associated with this model
    def get_document(self):
        return self.document

    ## Factory method that creates a GraphModel from files stored on disk
    # @param name Name of directory containing graph to load.
    @staticmethod
    def load(name):
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
            rospy.logdebug('Loading state %s' % sname)
            try:
                gm.states_dict[sname] = pk.load(pickle_file)
                pickle_file.close()
                rospy.logdebug('Got an instance of %s' %\
                        str(gm.states_dict[sname].__class__))
                if is_container(gm.states_dict[sname]):
                    gm.states_dict[sname] = gm.states_dict[sname].load_and_recreate(name)

                if gm.states_dict[sname].__class__ == tu.EmptyState:
                    gm.gve.add_node(sname, GraphModel.OUTCOME_NODE_RADIUS)
                else:
                    gm.gve.add_node(sname, GraphModel.NODE_RADIUS)

            except Exception, e:
                rospy.loginfo('Exception encountered while loading %s: %s. Omitting.'\
                        % (fname, str(e)))
                if gm.states_dict.has_key(sname):
                    gm.states_dict.pop(sname)

        #Reconstruct graph
        edges_filename = pt.join(name, GraphModel.EDGES_FILE)
        edges_pickle_file = open(edges_filename, 'r')
        edges = pk.load(edges_pickle_file)
        edges_pickle_file.close()
        for node1, node2, n1_outcome in edges:
            if gm.states_dict.has_key(node1) and gm.states_dict.has_key(node2):
                gm.gve.add_edge(node1, node2, label=n1_outcome, 
                        length=GraphModel.EDGE_LENGTH)

        gm.set_document(FSMDocument(name, modified=False, real_filename=True))

        return gm

    ## Saves GraphModel to disk
    # @param name Path to GraphModel on disk.
    def save(self, name):
        rospy.loginfo('GraphModel: saving to %s' % name)
        if not pt.exists(name):
            os.mkdir(name)
        else:
            shutil.rmtree(name)
            os.mkdir(name)

        #Save each state
        for state_name in self.states_dict.keys():
            containerp = is_container(self.states_dict[state_name])

            if containerp:
                self.states_dict[state_name].save_child(name)
                child = self.states_dict[state_name].abort_child()

            state_fname = pt.join(name, state_name) + '.state'
            pickle_file = open(state_fname, 'w')
            pk.dump(self.states_dict[state_name], pickle_file)
            pickle_file.close()

            if containerp:
                self.states_dict[state_name].set_child(child)

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
        pk.dump({'start_state': self.start_state, 
            'state_names': self.states_dict.keys()}, pickle_file)
        pickle_file.close()

        self.document = FSMDocument(name, False, True)

    ## Create a SMACH state machine from a single state
    # @param state State object (object which inherits from StateBase).
    # @param robot Robot object (as defined in RCommander class).
    def create_singleton_statemachine(self, state, robot):
        temp_gm = GraphModel()
        temp_gm.add_node(state)
        temp_gm.set_start_state(state.name)
        return temp_gm.create_state_machine(robot), temp_gm

    ## Runs a SMACH state machine, hook up its callback to this GraphModel
    # object.
    # @param name Name of state machine.
    # @param state_machine SMACH state machine to run.
    # @param userdata User data field for SMACH.
    def run(self, name="", state_machine=None, userdata=None):

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

    ## Preempts the currently running SMACH state machine when called
    def preempt(self):
        if self.is_running():
            self.sm_thread['run_sm'].preempt()
            self.sm_thread['preempted'] = time.time()
            #self.sm_thread['run_sm'].preempt()

    ## Checks if there is a currently running SMACH state machine
    def is_running(self):
        return self.sm_thread != None

    ## Sets a callback that notifies listeners that the interal SMACH state
    # machine finished
    # @param func A function that takes in a string.
    def register_status_cb(self, func):
        self.status_cb_func = func

    ## Sets a callback that gets called whenever the SMACH state machine transitions.
    # @param func a function that takes in a list of strings (each element is
    # the name of an active state)
    def register_transition_cb(self, func):
        self.transition_cb = func

    ## Sets a callback that gets called whenever the SMACH state machine starts.
    # @parm func A function that takes in a list of strings (each element is
    # the name of a start state)
    def register_start_cb(self, func):
        self.start_cb = func

    ## Callback function for SMACH state machine
    # @param exception Exception encountered during execution
    def _sm_thread_termination_cb(self, exception):
        if exception != None:
            if self.status_cb_func != None:
                self.status_cb_func('Error: %s' %\
                        str(exception.__class__), exception)

        elif self.sm_thread['preempted'] != None:
            if self.status_cb_func != None:
                self.status_cb_func('%s stopped.' %\
                        (self.document.get_name()))

        self.sm_thread = None

    ## Callback when SMACH transitions
    def _state_machine_transition_cb(self, user_data, active_states):
        self.sm_thread['current_states'] = active_states
        if self.status_cb_func != None:
            self.status_cb_func('At state %s' % (active_states[0]))
        if self.transition_cb != None:
            self.transition_cb(active_states)

    ## Callback when SMACH starts
    def _state_machine_start_cb(self, userdata, initial_states):
        self.sm_thread['current_states'] = initial_states
        if self.status_cb_func != None:
            self.status_cb_func('At state %s.' % (initial_states[0]))
        if self.start_cb != None:
            self.start_cb(initial_states)

    ## Get the last outcome returned by executing SMACH state machine.
    def get_last_outcome(self):
        return self.last_outcome

    ## Callback when SMACH state machine terminates.
    def _state_machine_termination_cb(self, userdata, terminal_states, 
            container_outcome):
        self.sm_thread['current_states'] = terminal_states
        self.sm_thread['outcome'] = container_outcome
        self.last_outcome = [container_outcome, time.time()]

        if self.status_cb_func != None:
            self.status_cb_func('Finished with outcome %s' % container_outcome)

    ## Gets outputs of a given class type
    # @param class_filter A Python class used for filtering.
    # @return A list strings representing outputs of the given types from nodes
    #           in this state machine.
    def outputs_of_type(self, class_filter):
        filtered_output_variables = []
        for node_name in self.real_states():
            node = self.get_state(node_name)
            output_names = node.output_names()
            for output_name in output_names:
                if issubclass(node.output_type(output_name), class_filter):
                    filtered_output_variables.append(output_name)
        return filtered_output_variables

    ## Create an executable SMACH state machine
    # @param robot Robot object.
    # @param userdata SMACH userdata object.
    # @param ignore_start_state Does not set the start state in state machine returned.
    def create_state_machine(self, robot, userdata=None, \
            ignore_start_state=False):
        sm = smach.StateMachine(outcomes = self.outcomes())

        # Copy over input userdata into current state machine so that nodes
        # contained would have access.
        if userdata != None:
            for key in userdata.keys():
                exec ("sm.userdata.%s = userdata.%s" % (key, key))
                exec ("print 'data in key is', sm.userdata.%s" % (key))

        with sm:
            for node_name in self.real_states():
                node = self.states_dict[node_name]

                node_smach = node.get_smach_state()
                if hasattr(node_smach, 'set_robot'): 
                    node_smach.set_robot(robot)

                transitions = {}
                for e in self.gve.node(node_name).edges:
                    if e.node1.id == node_name:
                        transitions[e.label] = e.node2.id

                input_set = set(node_smach.get_registered_input_keys())
                output_set = set(node_smach.get_registered_output_keys())
                if len(input_set.intersection(output_set)) > 0:
                    raise RuntimeError('Input keys has the same name as output_keys.')

                remapping = {}
                for input_key in node_smach.get_registered_input_keys():
                    remapping[input_key] = node.remapping_for(input_key)
                
                #We assume that each output is to a SEPARATE variable
                #with the same name as the node, we only need the node's
                #name. Ex.  OUTPUT: GLOBAL
                #           NODE_NAME: NODE_NAME
                for output_key in node_smach.get_registered_output_keys():
                    remapping[output_key] = output_key

                smach.StateMachine.add(node_name, node_smach, 
                        transitions=transitions, remapping=remapping)

        if ignore_start_state:
            return sm

        if self.start_state == None:
            raise RuntimeError('No start state set.')
        sm.set_initial_state([self.start_state])
        return sm

    ## Gets the current nodes linked to outcomes of a given node.
    # @param node_name Name of node to check for outcomes.
    # @return A list of node names and outcomes
    #         e.g. [[edge_name, node_name], ...]
    def current_children_of(self, node_name):
        ret_list = []
        for edge in self.gve.node(node_name).edges:
            if edge.node1.id != node_name:
                continue
            ret_list.append([edge.label, edge.node2.id])
        return ret_list

    ## All nodes that has executable code (not outcomes/EmptyState)
    # @return a list of strings, each element is the name of a node.
    def real_states(self):
        noc = []
        for node_name in self.states_dict.keys():
            if self.states_dict[node_name].__class__ != tu.EmptyState:
                noc.append(node_name)
        return noc

    ## Gets all outcome states (complementary set to real_states)
    # @return a list of strings, each element is the name of a node.
    def outcomes(self):
        #all empty states are outcomes
        oc = []
        for node_name in self.states_dict.keys():
            if self.states_dict[node_name].__class__ == tu.EmptyState:
                oc.append(node_name)
        return oc

    ## Takes a state out of the states dictionary.
    # @param node_name A node's name.
    # @return StateBase object.
    def pop_state(self, node_name):
        return self.states_dict.pop(node_name)

    ## Gets a state of given name
    # @param node_name A node's name.
    # @return StateBase object.
    def get_state(self, node_name):
        return self.states_dict[node_name]

    ## Insert a state of the given name
    # @param node_name A node's name
    # @param state A StateBase object.
    def set_state(self, node_name, state):
        self.states_dict[node_name] = state

    ## Replace a node in the graph with a new one. Connects the new node to all
    #of the old node's connections if possible.
    # @param new_node StateBase object.
    # @param old_node_name name of the old node to replace.
    def replace_node(self, new_node, old_node_name):
        if new_node.get_name() != old_node_name \
                and self.states_dict.has_key(new_node.get_name()):
            raise RuntimeError('There is already a node named %s.' \
                    % new_node.get_name())

        self.states_dict.pop(old_node_name)
        self.states_dict[new_node.get_name()] = new_node
        new_node_name = new_node.get_name()

        #If the new node has the same name (possible to have different connections)
        #If the node is of a different name
        if new_node_name != old_node_name:
            self.gve.add_node(new_node_name, self.NODE_RADIUS)

        #for each existing connection
        new_smach_node = new_node.get_smach_state()
        if hasattr(new_smach_node, 'set_robot'): 
            new_smach_node.set_robot(None)

        new_outcomes = new_smach_node.get_registered_outcomes()
        for e in self.gve.node(old_node_name).edges:
        #   if it is an outcome in the new node
            if e.label in new_outcomes:
        #       if it has a different source, remove it and add a new one
                if e.node1.id == old_node_name:
                    self.gve.remove_edge(e.node1.id, e.node2.id, label=e.label)
                    self.gve.add_edge(new_node_name, e.node2.id, label=e.label, 
                            length=GraphModel.EDGE_LENGTH)
                elif e.node2.id == old_node_name:
                    self.gve.remove_edge(e.node1.id, e.node2.id, label=e.label)
                    self.gve.add_edge(e.node1.id, new_node_name, label=e.label, 
                            length=GraphModel.EDGE_LENGTH)
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
                    self.gve.add_edge(e.node1.id, new_node_name, label=e.label, 
                            length=GraphModel.EDGE_LENGTH)
        #   delete it   

        if new_node_name != old_node_name:
            self.gve.remove_node(old_node_name)
                
        #for each new outcome
        #   if we don't have an edge for it, create that edge & its temporary
        #   node
        self.restore_node_consistency(new_node.get_name())

    ## Finds all nodes that can be connected to the given outcome of this node.
    # @param node_name Node whose outcomes we need to search for.
    # @param outcome Outcome of the node that needs to be connected.
    def connectable_nodes(self, node_name, outcome):
        #can't connect to
        #  temporary nodes already connected whose name is not current outcome
        allowed_nodes = []

        for state_name in self.states_dict.keys():
	    #  An outcome node             AND (is not the type of outcome)
            if (not self.is_modifiable(state_name)) \
                    and (not self._is_type(state_name, outcome)):
                continue

            #ignore our own name
            if node_name == state_name:
                continue

            allowed_nodes.append(state_name)

        if node_name == None:
            return []
        else:
    	    for edge in self.gve.node(node_name).edges:
                #if this node currently has outcome remapped to a node that is
                #not a dummy outcome
                if edge.label == outcome and edge.node1.id == node_name \
                        and (not self._is_type(edge.node2.id, outcome)):
                    #make the outcome an option
                    allowed_nodes.append(self._create_outcome_name(outcome))

            allowed_nodes.sort()
            return allowed_nodes


    ## Create an outcome node of a given type 
    # @param outcome Outcome type (a string).
    # @return Actual name of outcome created (string).
    def _create_outcome_name(self, outcome):
        idx = 0
        name = "%s%d" % (outcome, idx)
        while self.states_dict.has_key(name):
            idx = idx + 1
            name = "%s%d" % (outcome, idx)
        return name

    ## Checks if a given state is of the same type as given outcome.  Returns
    # true if the outcome
    # type string is found in the state_name.
    # @param state_name Name of state.
    # @param outcome 
    def _is_type(self, state_name, outcome):
        r = state_name.find(outcome)
        if r < 0:
            return False
        else:
            return True

    ## Checks to see if state machine as the node of the given name
    def has_node_name(self, name):
        return self.states_dict.has_key(name)

    ## Adds a node to this model. Creates outcome nodes for added node if needed.
    # @param node A StateBase object to add.
    def add_node(self, node):
        if self.states_dict.has_key(node.get_name()):
            node.set_name(node.get_name() + '_dup')

        #if this is a non-container node 
        if not hasattr(node, 'get_child_name') or \
                not self.states_dict.has_key(node.get_child_name()):

            #Check all outcomes and make new nodes if needed
            smach_node = node.get_smach_state()
            if hasattr(smach_node, 'set_robot'): 
                smach_node.set_robot(None)

            #Link this node to all its outcomes
            if smach_node.__class__ == tu.EmptyState:
                self.gve.add_node(node.get_name(), \
                        radius=self.OUTCOME_NODE_RADIUS)
            else:
                self.gve.add_node(node.get_name(), radius=self.NODE_RADIUS)

            self.states_dict[node.get_name()] = node

            if smach_node.__class__ == tu.EmptyState:
                return

            #For each outcome
            for outcome in smach_node.get_registered_outcomes():
                #Create an empty state and add an edge to it
                outcome_name = self._create_outcome_name(outcome)
                self.states_dict[outcome_name] = tu.EmptyState(outcome_name, 
                        temporary=True)
                self.gve.add_node(outcome_name, 
                        radius=self.OUTCOME_NODE_RADIUS)
                self._add_edge(node.get_name(), outcome_name, outcome)

        else:
            # If this node has a child node we replace its child node with it
            # instead of performing an add
            self.replace_node(node, node.get_child_name())

    ## Creates and add an outcome of the given name
    # @param outcome_name Name of outcome to add.
    def add_outcome(self, outcome_name):
        self.gve.add_node(outcome_name, radius=self.OUTCOME_NODE_RADIUS)
        self.states_dict[outcome_name] = tu.EmptyState(outcome_name, False)

    ## Deletes node from graph, deletes its outcome nodes too if needed.
    # @param node_name Name of node (StateBase obj) to delete
    def delete_node(self, node_name):
        node_obj = self.gve.node(node_name)
        children_edges = []
        parent_edges = []

        #Separate edges from parents and edges to children
        for cn in node_obj.links:
            for edge in self.gve.all_edges_between(node_name, cn.id):
                if (edge.node1.id == node_name) \
                        and (edge.node2.id == node_name):
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
            if not self.is_modifiable(e.node2.id) and len(e.node2.edges) <= 1:
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

            #Create an index of siblings
            parents_children = {}
            for parent_outcome_name, sibling_node_name in\
                    self.current_children_of(parent_node_id):
                parents_children[parent_outcome_name] = sibling_node_name

            #For each child edge of ours
            for edge in filtered_children_edges:
                node_outcome_name = edge.label

                #if parent has a similar outcome connected to a temporary node
                if parents_children.has_key(node_outcome_name):
                    parent_outcome_node = parents_children[node_outcome_name]
                    #If parent outcome is connected to a temporary node,
                    #replace the temporary node with link to us
                    if not self.is_modifiable(parent_outcome_node):
                        #connect this child node to parent
                        self.gve.remove_edge(parent_node_id, 
                                parent_outcome_node, label=node_outcome_name)
                        self.gve.add_edge(parent_node_id, edge.node2.id, 
                                label=node_outcome_name, 
                                length=GraphModel.EDGE_LENGTH)
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
            self.gve.remove_edge(parent_edge.node1.id, parent_edge.node2.id, 
                    parent_edge.label)
            self.restore_node_consistency(parent_edge.node1.id)

        self.gve.remove_node(node_name)
        self.states_dict.pop(node_name)
        if self.start_state == node_name:
            self.start_state = None

    # For each registered outcome, make sure there exists an edge for that outcome.  If no
    # edge exists, create it.
    # @param node_name to restore consistency to.
    def restore_node_consistency(self, node_name):
        clist = self.current_children_of(node_name)
        cdict = {}
        for outcome_name, nn in clist:
            cdict[outcome_name] = nn

        smach_state = self.states_dict[node_name].get_smach_state()
        if hasattr(smach_state, 'set_robot'): 
            smach_state.set_robot(None)
        registered_outcomes = smach_state.get_registered_outcomes()

        #Remove things that are no longer outcomes
        for outcome in cdict.keys():
            if not (outcome in registered_outcomes):
                self.gve.remove_edge(node_name, cdict[outcome], outcome)
                if (not self.is_modifiable(cdict[outcome])) \
                        and len(self.gve.node(cdict[outcome]).edges) < 1:
                    self.gve.remove_node(cdict[outcome])
                    self.states_dict.pop(cdict[outcome])

        for outcome in registered_outcomes:
            if not cdict.has_key(outcome):
                new_outcome_name = self._create_outcome_name(outcome)
                self._add_temporary_outcome(new_outcome_name)
                self._add_edge(node_name, new_outcome_name, outcome)

    ## Adds a temporary placeholder outcome (disappears when use connect this node to others).
    # @param outcome Name of outcome to add.
    def _add_temporary_outcome(self, outcome):
        self.states_dict[outcome] = tu.EmptyState(outcome, temporary=True)
        self.gve.add_node(outcome, self.OUTCOME_NODE_RADIUS)

    ## Checks to see if given node is outcome node or not (outcome nodes are not modifiable)
    # @param node_name Name of node.
    def is_modifiable(self, node_name):
        if (self.states_dict[node_name].__class__ == tu.EmptyState) \
                and self.states_dict[node_name].temporary:
            return False
        else:
            return True

    ## Adds an edge between two nodes
    # @param n1 first node's name (string)
    # @param n2 second node's name (string)
    # @param n1_outcome outcome to connect (string)
    def _add_edge(self, n1, n2, n1_outcome):
        if not self.states_dict.has_key(n1) or not self.states_dict.has_key(n2):
            raise RuntimeError('One of the specified nodes does not exist.  Can\'t add edge.')

        if self.gve.edge(n1, n2, n1_outcome) != None:
            rospy.loginfo("Edge between %s and %s exists, ignoring connnection request" % (n1, n2))
            return False

        #Don't add edges to "temporary" nodes
        if n1_outcome == None and self.is_modifiable(n2):
            raise RuntimeError('Must specify outcome as goal node is not a temporary node.')

        self.gve.add_edge(n1, n2, label=n1_outcome, 
                length=GraphModel.EDGE_LENGTH)
        return True

    ## Public add edge function.  Adds an edge between two nodes (checks to make sure that the
    # nodes aren't outcome nodes.
    # @param n1 first node's name (string)
    # @param n2 second node's name (string)
    def add_edge(self, n1, n2, n1_outcome):
        if not self.is_modifiable(n1) or not self.is_modifiable(n2):
            return False
        else:
            return self._add_edge(n1, n2, n1_outcome)

    ## Delete an edge
    # @param edge a NodeBox.graph edge object.
    def delete_edge(self, edge):
        if not self.is_modifiable(edge.node1.id) or\
                not self.is_modifiable(edge.node2.id):
            return False
        else:
            self.gve.remove_edge(edge.node1.id, edge.node2.id, e.label)
            return True

    ## Call back from RCommander to notify that a node's connection has been changed.
    # @param node_name Name of node.
    # @param outcome_name Name of outcome on node that has changed.
    # @param new_node Name of new node to connect outcome to.
    def connection_changed(self, node_name, outcome_name, new_node):
        if node_name == None:
            return

        #if the new node is not in our database, just create it as an outcome node
        if not self.states_dict.has_key(new_node):
            self.states_dict[new_node] = tu.EmptyState(new_node, 
                    temporary=True)
            self.gve.add_node(new_node, radius=self.OUTCOME_NODE_RADIUS)


        #find the old edge
        old_edge = None
        for edge in self.gve.node(node_name).edges:
            if edge.label == outcome_name and edge.node1.id == node_name:
                if old_edge != None:
                    raise RuntimeError('Two edges detected for one outcome named %s. %s -> %s and %s -> %s' % (outcome_name, old_edge.node1.id, old_edge.node2.id, edge.node1.id, edge.node2.id))
                old_edge = edge

        #print node_name, outcome_name, new_node
        if old_edge != None:
            if old_edge.node2.id == new_node:
                return

            #remove the old connection
            self.gve.remove_edge(node_name, old_edge.node2.id, 
                    label=old_edge.label)
            #remove the old node if it's temporary 
            if not self.is_modifiable(old_edge.node2.id):
                #and not connected
                if len(self.gve.node(old_edge.node2.id).edges) <= 0:
                    self.gve.remove_node(old_edge.node2.id)
                    self.states_dict.pop(old_edge.node2.id)

        #add new connection
        if self.gve.node(new_node) == None:
            self.states_dict[new_node] = tu.EmptyState(new_node, 
                    temporary=True)
            self.gve.add_node(new_node, self.OUTCOME_NODE_RADIUS)
        self._add_edge(node_name, new_node, outcome_name)
