#!/usr/bin/python
import roslib; roslib.load_manifest('rcommander_core')
import rospy
import actionlib
import pr2_interactive_manipulation.msg as pim
import graph_model as gm
import sm_thread_runner as smtr
import point_tool as pt
import pr2_utils as pu
import tf

class PoseStampedScriptedActionServer:

    def __init__(self, action_name, path_to_action):
        rospy.loginfo('Starting server for %s with path %s' %(action_name, path_to_action))
        self.tf_listener = tf.TransformListener()
        self.pr2 = pu.PR2(self.tf_listener)

        #Setup ROS Action Server
        self._action_name = action_name
        self._as = actionlib.SimpleActionServer(self._action_name, pim.PoseStampedScriptedAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

        rospy.loginfo('%s server up!' % action_name)

    def setup_sm(self):
        #Setup state machine
        self.graph_model = gm.GraphModel.load(path_to_action, self.pr2)
        #for k in self.graph_model.smach_states:
        #    if hasattr(self.graph_model.smach_states[k], 'set_robot'):
        #        self.graph_model.smach_states[k].set_robot(self.pr2)

        #Find the first global node that is of the right type
        self.point_field_name = None
        for node_name in self.graph_model.global_nodes(None):
            node = self.graph_model.get_smach_state(node_name)
            if node.__class__ == pt.Point3DState:
                self.point_field_name = node_name
        if self.point_field_name == None:
            raise RuntimeError('This statemachine doesn\'t have any nodes of type Point3DState')

    def execute_cb(self, goal):
        self.setup_sm()
        r = rospy.Rate(30)
        position = [goal.pose_stamped.pose.position.x, 
                goal.pose_stamped.pose.position.y, 
                goal.pose_stamped.pose.position.z]
        frame = goal.pose_stamped.header.frame_id
        print 'Position', position, 'frame', frame

        self.graph_model.get_smach_state(self.point_field_name).set_info((position, frame))
        state_machine = self.graph_model.create_state_machine()
        rthread = smtr.ThreadRunSM(self._action_name, state_machine)
        rthread.start()

        #Do something
        while True:
            if self._as.is_preempt_requested():
                self._as.set_preempted()
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
            result = pim.PoseStampedScriptedResult(state_machine_output)
            rospy.loginfo("%s: Succeeded" % self._action_name)
            self._as.set_succeeded(result)
        else:
            self._as.set_aborted()


if __name__ == '__main__':
    import sys
    action_name    = sys.argv[1]
    path_to_action = sys.argv[2]
    rospy.init_node(action_name)
    action_server = PoseStampedScriptedActionServer(action_name, path_to_action)
    rospy.spin()


