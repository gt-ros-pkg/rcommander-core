#!/usr/bin/python
import roslib; roslib.load_manifest('rcommander')
import rospy
import actionlib
import pr2_interactive_manipulation.msg as pim
import sys

action_name = sys.argv[1]
rospy.init_node(action_name + '_client', anonymous=True)
client = actionlib.SimpleActionClient(action_name, pim.PoseStampedScriptedAction)
client.wait_for_server()

goal = pim.PoseStampedScriptedGoal()
goal.pose_stamped.pose.position.x = .742
goal.pose_stamped.pose.position.y = -.026
goal.pose_stamped.pose.position.z = .4955
goal.pose_stamped.header.frame_id = '/base_link'
client.send_goal(goal)
client.wait_for_result()
print client.get_result()
