#!/usr/bin/python
import roslib; roslib.load_manifest('rcommander_plain')
import rcommander.rcommander as rc
import rospy
import tf

rospy.init_node('rcommander_plain', anonymous=True)
rc.run_rcommander(['default'])

