#!/usr/bin/python
import roslib; roslib.load_manifest('rcommander_pr2')

import tf 
import rospy

import pr2_utils as pu
import sys
import rcommander.rcommander_server as rcs

rospy.init_node('rcommander_actions_server')
tf = tf.TransformListener()
pr2 = pu.PR2(tf)
rcs.run(pr2)
