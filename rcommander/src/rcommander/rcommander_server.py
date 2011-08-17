import roslib; roslib.load_manifest('rcommander')

import os.path
import os

import actionlib
import rospy
import pr2_interactive_manipulation.msg as pim 
import geometry_msgs.msg as geo
import sys

#from srv import *
from pr2_interactive_manipulation.srv import *
import sm_thread_runner as smtr
import graph_model as gm
import point_tool as pt


class PoseStampedScriptedActionServer:

    def __init__(self, action_name, path_to_action, robot):
        self.action_name = action_name
        self.path_to_action = path_to_action
        #rospy.loginfo('Starting server for %s with path %s' %(action_name, path_to_action))
        self.robot = robot
        #Setup ROS Action Server
        #self._action_name = action_name
        #self._as = actionlib.SimpleActionServer(self._action_name, pim.PoseStampedScriptedAction, execute_cb=self.execute_cb, auto_start=False)
        #self._as.start()
        #rospy.loginfo('%s server up!' % action_name)

    def setup_sm(self):
        #Setup state machine
        self.graph_model = gm.GraphModel.load(self.path_to_action, self.robot)

        #Find the first global node that is of the right type
        self.point_field_name = None
        for node_name in self.graph_model.global_nodes(None):
            node = self.graph_model.get_smach_state(node_name)
            if node.__class__ == pt.Point3DState:
                self.point_field_name = node_name
        if self.point_field_name == None:
            raise RuntimeError('This statemachine doesn\'t have any nodes of type Point3DState')

    def execute(self, pose_stamped, actserver):
        self.setup_sm()
        r = rospy.Rate(30)
        position = [pose_stamped.pose.position.x, 
                    pose_stamped.pose.position.y, 
                    pose_stamped.pose.position.z]
        frame = pose_stamped.header.frame_id
        print 'Position', position, 'frame', frame

        self.graph_model.get_smach_state(self.point_field_name).set_info((position, frame))
        state_machine = self.graph_model.create_state_machine()
        rthread = smtr.ThreadRunSM(self.action_name, state_machine)
        rthread.start()

        #Do something
        while True:
            if actserver.is_preempt_requested():
                actserver.set_preempted()
                success = False
                break

            if rthread.exception:
                raise rthread.exception

            if rthread.outcome != None:
                success = True
                break

            if not rthread.isAlive():
                raise RuntimeError("Thread died unexpectedly.")

            r.sleep()

        if success:
            state_machine_output = rthread.outcome
            #result = pim.PoseStampedScriptedResult(state_machine_output)
            result = pim.RunScriptResult(state_machine_output)
            rospy.loginfo("%s: succeeded with %s" % (self.action_name, state_machine_output))
            actserver.set_succeeded(result)
        else:
            actserver.set_aborted()


# rviz talks to server, which loads and serves everything in a directory
# assume all the actions take a click 
class RCommanderServer:

    def __init__(self, robot):
        self.robot = robot
        self.action_dict = {}
        rospy.Service('list_rcommander_actions', ActionInfo, self.list_action_cb)
        self.actserv = actionlib.SimpleActionServer('run_rcommander_action', pim.RunScriptAction, execute_cb=self.execute_cb, auto_start=False)
        self.actserv.start()

    def execute_cb(self, goal):
        rospy.loginfo('Requested: group ' + goal.group_name + ' action: ' + goal.action_name)
        rospy.loginfo('waiting for click..')
        ps_msg = rospy.wait_for_message('/cloud_click_point', geo.PoseStamped)
        self.action_dict[goal.group_name][goal.action_name].execute(ps_msg, self.actserv)

    def load_actions_in_groups(self, action_groups):
        for group_name, path_name in action_groups:
            self.load_group(group_name, path_name)

    def load_group(self, group_name, path_name):
        rospy.loginfo('Loading group: ' + group_name)
        dirs = []#[d for d in os.listdir(path_name) if os.path.isdir(d)]
        for d in os.listdir(path_name):
            if os.path.isdir(os.path.join(path_name, d)):
                dirs.append(d)
        rospy.loginfo('Found actions: ' + str(dirs))
        self.action_dict[group_name] = {}
        for action_pt in dirs:
            self.serve_action(action_pt, os.path.join(path_name, action_pt), group_name)

    def serve_action(self, action_name, action_path, group_name):
        #Actually, they're all pose stamped scripted actions...
        self.action_dict[group_name][action_name] = PoseStampedScriptedActionServer(action_name, action_path, self.robot)

    def list_action_cb(self, req):
        actions = self.action_dict[req.group_name].keys()
        actions.sort()
        return ActionInfoResponse(actions)

    #def start_server(self):
    #    pass

    #def stop_server(self):
    #    pass

    #def refresh_server(self):
    #    pass

#class RCommanderGUI:

#need to be robot specific
#Can load from cmd line & gui 
#cmd line version takes a path that contains everything you want to serve and serve it
#gui allows cmd line too!
def run(robot):
    from optparse import OptionParser

    p = OptionParser()
    p.add_option("-d", "--dir", dest="dir", action="append", default=[], help="directory to find saved actions from")
    p.add_option("-n", "--name", dest="name", action="append", default=[], help="name for action group")
    options, args = p.parse_args()

    server_man = RCommanderServer(robot)
    #print options.name, options.dir
    server_man.load_actions_in_groups(zip(options.name, options.dir))
    rospy.loginfo('server up!')
    rospy.spin()



