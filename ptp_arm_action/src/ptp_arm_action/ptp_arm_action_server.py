#! /usr/bin/python
import roslib; roslib.load_manifest('ptp_arm_action')
import rospy
import actionlib

import geometry_msgs.msg as gm
import ptp_arm_action.msg as ptp

from object_manipulator.convert_functions import *
from math import sqrt, pi, fabs
import tf_utils as tfu
import numpy as np
#import scipy
import math
import tf.transformations as tr
import tf
from pycontroller_manager.pycontroller_manager import ControllerManager


##
# Measures the distance between two pose stamps, independently factors out
# distance in rotation and distance in translation. Converts both poses into
# the same coordinate frame before comparison.
def pose_distance(ps_a, ps_b, tflistener):
    #put both into the same frame
    ps_a_fb = change_pose_stamped_frame(tflistener, ps_a, ps_b.header.frame_id)
    g_T_a = pose_to_mat(ps_a_fb.pose)
    g_T_b = pose_to_mat(ps_b.pose)

    a_T_b = g_T_a**-1 * g_T_b

    desired_trans = a_T_b[0:3, 3].copy()
    a_T_b[0:3, 3] = 0
    desired_angle, desired_axis, _ = tf.transformations.rotation_from_matrix(a_T_b)
    return desired_trans, desired_angle, desired_axis


class PTPArmActionServer:

    def __init__(self, name, arm):
        if arm == 'left':
            self.controller = 'l_cart'
            self.joint_controller = 'l_arm_controller'
            #self.tool_frame = 'l_gripper_tool_frame'
            self.tool_frame = rospy.get_param('/l_cart/tip_name')
        elif arm == 'right':
            self.controller = 'r_cart'
            self.joint_controller = 'r_arm_controller'
            #self.tool_frame = 'r_gripper_tool_frame'
            self.tool_frame = rospy.get_param('/r_cart/tip_name')
        else:
            raise RuntimeError('Invalid parameter for arm: %s' % arm)

        self.arm = arm
        self.target_pub = rospy.Publisher(self.controller + '/command_pose', gm.PoseStamped)
        #self.pose_sub = rospy.Subscriber(self.controller + '/state/x', gm.PoseStamped, self.pose_callback)

        self.tf = tf.TransformListener()
        self.trans_tolerance = rospy.get_param("~translation_tolerance")
        self.rot_tolerance = math.radians(rospy.get_param("~rotation_tolerance"))
        self.stall_time = rospy.get_param("~stall_time")
        #rospy.loginfo('trans tolerance ' + str(self.trans_tolerance))
        self.time_out = rospy.get_param("~timeout")

        self.controller_manager = ControllerManager()
        #self.controller_manager.cart_mode(self.arm)
        self._action_name = name
        self.linear_movement_as = actionlib.SimpleActionServer(self._action_name, ptp.LinearMovementAction, 
					execute_cb=self.action_cb, auto_start=False)
        self.linear_movement_as.start()

        rospy.loginfo('Action name: %s Arm: %s' % (self._action_name, self.arm))


    def action_cb(self, msg):
        rospy.loginfo('message that we got:\n' + str(msg))
        self.controller_manager.cart_mode(self.arm)
        #self._wait_for_pose_message()
        self.trans_tolerance = rospy.get_param("~translation_tolerance")
        self.rot_tolerance = math.radians(rospy.get_param("~rotation_tolerance"))
        self.time_out = rospy.get_param("~timeout")

        success = False
        r = rospy.Rate(100)

        goal_ps = msg.goal
        relative_movement = msg.relative
        trans_vel = msg.trans_vel
        rot_vel = msg.rot_vel
        if trans_vel <= 0:
            trans_vel = .02
        if rot_vel <= 0:
            rot_vel = pi/20.

        if relative_movement:
            rospy.loginfo('Received relative motion.')

            ref_T_tip = tfu.tf_as_matrix(self.tf.lookupTransform(goal_ps.header.frame_id, self.tool_frame, rospy.Time(0)))
            tll_T_ref = tfu.tf_as_matrix(self.tf.lookupTransform('torso_lift_link', goal_ps.header.frame_id, rospy.Time(0)))

            tip_R_tp  = pose_to_mat(goal_ps.pose)
            ref_T_tp  = ref_T_tip * tip_R_tp
            tll_T_tp  = tll_T_ref * ref_T_tp

            goal_ps = stamp_pose(mat_to_pose(tll_T_tp), 'torso_lift_link')


        tstart = rospy.get_time()
        tmax = tstart + self.time_out
        self.controller_manager = ControllerManager()
        rospy.loginfo('Goal is x %f y %f z %f in %s' % (goal_ps.pose.position.x, goal_ps.pose.position.y, 
            goal_ps.pose.position.z, goal_ps.header.frame_id))

        goal_torso = change_pose_stamped_frame(self.tf, goal_ps, 'torso_lift_link')
        rospy.loginfo('Goal is x %f y %f z %f in %s' % (goal_torso.pose.position.x, goal_torso.pose.position.y, 
            goal_torso.pose.position.z, goal_torso.header.frame_id))

        verbose = False

        time_ang = None
        min_ang_error = None
        time_trans = None
        min_trans_error = None

        while True:
            cur_time = rospy.get_time()

            gripper_matrix = tfu.tf_as_matrix(self.tf.lookupTransform('torso_lift_link', self.tool_frame, rospy.Time(0)))
            gripper_ps = stamp_pose(mat_to_pose(gripper_matrix), 'torso_lift_link')
            #Someone preempted us!
            if self.linear_movement_as.is_preempt_requested():
                #Stop our motion
                self.target_pub.publish(stamp_pose(gripper_ps.pose, gripper_ps.header.frame_id))
                self.linear_movement_as.set_preempted()
                rospy.loginfo('action_cb: preempted!')
                break

            #Calc feedback
            if verbose:
                print 'current_pose %.3f %.3f %.3f, rot %.3f %.3f %.3f %.3f in %s' % (gripper_ps.pose.position.x, gripper_ps.pose.position.y, gripper_ps.pose.position.z, 
                        gripper_ps.pose.orientation.x, gripper_ps.pose.orientation.y, gripper_ps.pose.orientation.z, 
                        gripper_ps.pose.orientation.w, gripper_ps.header.frame_id)
                print 'goal_pose %.3f %.3f %.3f, rot %.3f %.3f %.3f %.3f in %s' % (goal_ps.pose.position.x, goal_ps.pose.position.y, goal_ps.pose.position.z, 
                        goal_ps.pose.orientation.x, goal_ps.pose.orientation.y, goal_ps.pose.orientation.z, 
                        goal_ps.pose.orientation.w, goal_ps.header.frame_id)

            trans, ang, _ = pose_distance(gripper_ps, goal_ps, self.tf)
            feedback = ptp.LinearMovementFeedback(gm.Vector3(trans[0,0], trans[1,0], trans[2,0]))
            self.linear_movement_as.publish_feedback(feedback)

            #Reached goal
            trans_mag = np.linalg.norm(trans)
            if verbose:
                print trans.T, 'trans_mag', trans_mag, 'ang', abs(ang), 'rot toler', self.rot_tolerance

            if min_trans_error == None or min_trans_error == None:
                min_trans_error = trans_mag
                min_ang_error = abs(ang)
                time_ang = cur_time
                time_trans = cur_time

            if trans_mag < min_trans_error:
                min_trans_error = trans_mag
                time_trans = cur_time

            if abs(ang) < min_ang_error:
                min_ang_error = abs(ang)
                time_ang = cur_time

            if self.trans_tolerance > trans_mag and self.rot_tolerance > abs(ang):
                rospy.loginfo('action_cb: reached goal.')
                break

            #Timed out! is this a failure?
            if cur_time > tmax:
                rospy.loginfo('action_cb: timed out.')
                break

            #if it has been a while since we made progress
            if trans_mag > min_trans_error and (cur_time - time_trans) > self.stall_time:
                rospy.loginfo('action_cb: stalled.')
                break

            if abs(ang) > min_ang_error and (cur_time - time_ang) > self.stall_time:
                rospy.loginfo('action_cb: stalled.')
                break

            #Send controls
            clamped_target = self.clamp_pose(goal_ps, trans_vel, rot_vel, ref_pose=gripper_ps)
            if verbose:
                print 'clamped_target', clamped_target.pose.position.x, clamped_target.pose.position.y, 
                print clamped_target.pose.position.z, clamped_target.header.frame_id, '\n'

            self.target_pub.publish(clamped_target)
            #break

        trans, ang, _ = pose_distance(gripper_ps, goal_ps, self.tf)
        result = ptp.LinearMovementResult(gm.Vector3(trans[0,0], trans[1,0], trans[2,0]))
        if self.trans_tolerance > np.linalg.norm(trans):
            rospy.loginfo( 'SUCCEEDED! %.3f ang %.3f' % (np.linalg.norm(trans), np.degrees(ang)))
            self.linear_movement_as.set_succeeded(result)
        else:
            rospy.loginfo('ABORTED! %.3f ang %.3f' % (np.linalg.norm(trans), np.degrees(ang)))
            self.linear_movement_as.set_aborted(result)

    
    def clamp_pose(self, desired_pose, max_trans, max_rot, ref_pose):
        current_pose_d = change_pose_stamped_frame(self.tf, ref_pose, desired_pose.header.frame_id) 
        g_T_c  = pose_to_mat(current_pose_d.pose)
            
        desired_trans, desired_angle, desired_axis = pose_distance(ref_pose, desired_pose, self.tf)
        desired_trans_mag = np.linalg.norm(desired_trans)
        frac_trans = fabs(desired_trans_mag / max_trans)
        frac_rot = fabs(desired_angle / max_rot)

        if frac_trans <= 1 and frac_rot <= 1:
            return desired_pose
        frac = max(frac_rot, frac_trans)

        clamped_angle = desired_angle / frac
        clamped_trans = desired_trans / frac


        c_T_d = np.matrix(tf.transformations.rotation_matrix(clamped_angle, desired_axis))
        c_T_d[0:3, 3] = clamped_trans
        g_T_d = g_T_c * c_T_d
        clamped_pose = stamp_pose(mat_to_pose(g_T_d), desired_pose.header.frame_id)
        clamped_pose.header.stamp = rospy.Time.now()
        return clamped_pose

if __name__ == '__main__':
    import sys

    if len(sys.argv) < 2:
        arm = 'left'
    else:
        arm = sys.argv[1]

    rospy.init_node('ptp')
    left_as = PTPArmActionServer(arm +'_ptp', arm)
    rospy.loginfo('PTP action server started.')
    rospy.spin()


