import roslib; roslib.load_manifest('ptp_arm_action')
import rospy
import actionlib
import ptp_arm_action.msg as ptp
import numpy as np
from object_manipulator.convert_functions import *
import tf.transformations as tr

rospy.init_node('ptp_client')
client = actionlib.SimpleActionClient('right_ptp', ptp.LinearMovementAction)
client.wait_for_server()

#Construct goal
goal = ptp.LinearMovementGoal()
goal.relative = True
goal.trans_vel = 0.02
goal.rot_vel = 0.02
motion = tr.identity_matrix()
motion[0,3] = .1
print motion
goal.goal = stamp_pose(mat_to_pose(motion), 'base_link')
print 'goal is\n', goal
client.send_goal(goal)

client.wait_for_result()
r = client.get_result()
print r
#print r.__class__, r
print 'done!'
#rospy.spin()


