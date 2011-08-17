#!/usr/bin/python
import roslib; roslib.load_manifest('rcommander_pr2')
import rcommander.rcommander as rc
import pr2_utils as pu
import rospy
import tf 
import rcommander.rcommander_server as rcs

rospy.init_node('rcommander', anonymous=True)
tf = tf.TransformListener()
pr2 = pu.PR2(tf)
rc.run(pr2, tf, ['all', 'pr2'])
