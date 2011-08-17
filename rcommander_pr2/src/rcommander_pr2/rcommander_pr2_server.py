#!/usr/bin/python
import roslib; roslib.load_manifest('rcommander_pr2')
import pr2_utils as pu
import tf 
import rcommander.rcommander_server as rcs
import rospy

print dir(rcs)

#rospy.init_node('rcommander_actions_server')
#tf = tf.TransformListener()
#pr2 = pu.PR2(tf)
#rcs.run(pr2)
