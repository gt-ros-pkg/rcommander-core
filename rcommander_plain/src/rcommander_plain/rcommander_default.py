#!/usr/bin/python
import roslib; roslib.load_manifest('rcommander_plain')
import rcommander.rcommander as rc
import rospy
import tf

rospy.init_node('rcommander_plain', anonymous=True)
robot = None
tf    = tf.TransformListener()
rc.run_rcommander(['default', 'default_frame', 'plain'], robot, tf)
