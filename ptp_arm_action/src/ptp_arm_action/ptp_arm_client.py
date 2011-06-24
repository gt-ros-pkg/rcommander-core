import roslib; roslib.load_manifest('ptp_arm_action')
import rospy
import actionlib
import ptp_arm_action.msg as ptp
import hrl_pr2_lib.pr2 as hpr2
import numpy as np
from object_manipulator.convert_functions import *
        
rospy.init_node('ptp_client')
pr2 = hpr2.PR2(kinematics=False)
base_link_mat_pose = pr2.left.pose_cartesian()
print base_link_mat_pose
trans = base_link_mat_pose[0:3,3]
print 'trans', trans.T
base_link_mat_pose[0:3, 3] = base_link_mat_pose[0:3, 3] + np.matrix([-.08,0,0]).T
print 'moved', base_link_mat_pose[0:3,3].T

client = actionlib.SimpleActionClient('left_ptp', ptp.LinearMovementAction)
client.wait_for_server()
goal = ptp.LinearMovementGoal()
goal.goal = stamp_pose(mat_to_pose(base_link_mat_pose), 'base_link')
client.send_goal(goal)

client.wait_for_result()
r = client.get_result()
print r
#print r.__class__, r
print 'done!'
#rospy.spin()


