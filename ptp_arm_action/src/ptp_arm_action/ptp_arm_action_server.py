#! /usr/bin/python
import roslib; roslib.load_manifest('ptp_arm_action')
import rospy
import actionlib

import geometry_msgs.msg as gm
import pr2_mechanism_msgs.srv as pmm
import ptp_arm_action.msg as ptp

from object_manipulator.convert_functions import *
from math import sqrt, pi, fabs
import tf_utils as tfu
import numpy as np
#import scipy
import math
import tf
import tf.transformations as tr


class ControllerManager:

    def __init__(self):
        # LoadController        
        self.load = rospy.ServiceProxy('pr2_controller_manager/load_controller', pmm.LoadController)

        # UnloadController        
        self.unload = rospy.ServiceProxy('pr2_controller_manager/unload_controller', pmm.UnloadController)

        # SwitchController
        self._switch_controller = rospy.ServiceProxy('pr2_controller_manager/switch_controller', pmm.SwitchController)

        self.joint_controllers = {}
        self.cart_controllers = {}
        for arm in ['l', 'r']:
            self.joint_controllers[arm] = arm + '_arm_controller'
            self.cart_controllers[arm] = cart_controller_name = arm + '_cart'

    def switch(self, start_con, stop_con):
        print 'switching to', start_con, 'from', stop_con
        for n in start_con:
            self.load(n)
        resp = self._switch_controller(start_con, stop_con, pmm.SwitchControllerRequest.STRICT)
        for n in stop_con:
            self.unload(n)
        return resp.ok

    def joint_mode(self, arm):
        #get current state
        if arm == 'left' or arm == 'both':
            self.switch([self.joint_controllers['l']], [self.cart_controllers['l']])
        if arm == 'right' or arm == 'both':
            self.switch([self.joint_controllers['r']], [self.cart_controllers['r']])

    def cart_mode(self, arm):
        if arm == 'left' or arm == 'both':
            #print 'switchleft'
            self.switch([self.cart_controllers['l']], [self.joint_controllers['l']])
        if arm == 'right' or arm == 'both':
            #print 'switchright'
            self.switch([self.cart_controllers['r']], [self.joint_controllers['r']])


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
            self.tool_frame = 'l_gripper_tool_frame'
        elif arm == 'right':
            self.controller = 'r_cart'
            self.joint_controller = 'r_arm_controller'
            self.tool_frame = 'r_gripper_tool_frame'
        else:
            raise RuntimeError('Invalid parameter for arm: %s' % arm)

        self.arm = arm
        self.target_pub = rospy.Publisher(self.controller + '/command_pose', gm.PoseStamped)
        #self.pose_sub = rospy.Subscriber(self.controller + '/state/x', gm.PoseStamped, self.pose_callback)

        self.tf = tf.TransformListener()
        self.trans_tolerance = rospy.get_param("~translation_tolerance")
        self.rot_tolerance = math.radians(rospy.get_param("~rotation_tolerance"))
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

            #Motion we want in given reference frame
            #ref_T_pose = pose_to_mat(goal_ps.pose)
            #rospy.loginfo('Motion we want in given reference frame\n' + str(ref_T_pose[:,3].T))

            ##Rotation that converts it from that random reference frame to the arm's tool frame
            #tip_T_ref = tfu.tf_as_matrix(self.tf.lookupTransform(self.tool_frame, goal_ps.header.frame_id, rospy.Time(0)))
            ##tip_T_ref = tfu.tf_as_matrix(self.tf.lookupTransform(goal_ps.header.frame_id, self.tool_frame, rospy.Time(0)))
            #tip_R_ref = tip_T_ref.copy()
            #tip_R_ref[0:3,3] = 0


            ##Motion we want in tool frame
            #tip_T_pose = tip_R_ref * ref_T_pose
            #print 'tip_R_ref pos %.2f %.2f %.2f'  % tuple([np.degrees(l) for l in tr.euler_from_quaternion(tr.quaternion_from_matrix(tip_R_ref))])
            #print 'ref_T_pose pos %.2f %.2f %.2f' % tuple([np.degrees(l) for l in tr.euler_from_quaternion(tr.quaternion_from_matrix(ref_T_pose))])
            #print 'tip_T_pose pos %.2f %.2f %.2f' % tuple([np.degrees(l) for l in tr.euler_from_quaternion(tr.quaternion_from_matrix(tip_T_pose))])

            #rospy.loginfo('Motion we want in tool frame %.3f %.3f %.3f\n' % (tip_T_pose[0,3], tip_T_pose[1,3], tip_T_pose[2,3]))
            ##Current position of tool frame in torso_lift_link frame 
            ## (this choice is arbitrary, but affects behavior of cartesian controller)
            #tll_T_tip  = tfu.tf_as_matrix(self.tf.lookupTransform('torso_lift_link', self.tool_frame, rospy.Time(0)))
            ##tll_T_tip  = tfu.tf_as_matrix(self.tf.lookupTransform(self.tool_frame, 'torso_lift_link', rospy.Time(0)))
            #rospy.loginfo('Current position of tool frame in torso_lift_link frame\n' + str(tll_T_tip[:,3].T))

            #tll_T_pose = tll_T_tip * tip_T_pose
            #print 'tll_T_tip %.2f %.2f %.2f' %  tuple([np.degrees(l) for l in tr.euler_from_quaternion(tr.quaternion_from_matrix(tll_T_tip))])
            #print 'tip_T_pose  pose %.2f %.2f %.2f' %  tuple([np.degrees(l) for l in tr.euler_from_quaternion(tr.quaternion_from_matrix(tip_T_pose))])
            #print 'tll_T_pose  pose %.2f %.2f %.2f' % tuple([np.degrees(l) for l in tr.euler_from_quaternion(tr.quaternion_from_matrix(tll_T_pose))])
    
	    ############################################
	    ############################################
	    ############################################

            ref_T_tip = tfu.tf_as_matrix(self.tf.lookupTransform(goal_ps.header.frame_id, self.tool_frame, rospy.Time(0)))
            tll_T_ref = tfu.tf_as_matrix(self.tf.lookupTransform('torso_lift_link', goal_ps.header.frame_id, rospy.Time(0)))

            tip_R_tp  = pose_to_mat(goal_ps.pose)
            ref_T_tp  = ref_T_tip * tip_R_tp
            tll_T_tp  = tll_T_ref * ref_T_tp
            #print 'tll_T_ref\n', tll_T_ref
            #print 'ref_T_tp \n', ref_T_tp
            #print 'result\n', tll_T_tp

            #print '>>>>'
            #print 'ref_T_tip rot %.2f %.2f %.2f' % tuple([np.degrees(l) for l in tr.euler_from_quaternion(tr.quaternion_from_matrix(ref_T_tip))])
            #print 'tip_R_tp rot %.2f %.2f %.2f' % tuple([np.degrees(l) for l in tr.euler_from_quaternion(tr.quaternion_from_matrix(tip_R_tp))])
            #print 'ref_T_tp rot %.2f %.2f %.2f' % tuple([np.degrees(l) for l in tr.euler_from_quaternion(tr.quaternion_from_matrix(ref_T_tp))])
            #print ''
            #print 'tll_T_ref rot %.2f %.2f %.2f' % tuple([np.degrees(l) for l in tr.euler_from_quaternion(tr.quaternion_from_matrix(tll_T_ref))])
            #print '* tll_T_tp rot %.2f %.2f %.2f' % tuple([np.degrees(l) for l in tr.euler_from_quaternion(tr.quaternion_from_matrix(tll_T_tp))])
            #print '* tll_T_tp tr', tll_T_tp[0:3,3].T
            #print '* tll_T_pose', tll_T_pose[0:3,3]
            #print '>>>>'

	    ############################################
	    ############################################
	    ############################################

            #print '>>>>'
            #tll_T_tip  = tfu.tf_as_matrix(self.tf.lookupTransform('torso_lift_link', self.tool_frame, rospy.Time(0)))
            #bl_T_tip  = tfu.tf_as_matrix(self.tf.lookupTransform('base_link', self.tool_frame, rospy.Time(0)))
            #print 'tll_T_tip  pose %.2f %.2f %.2f' % tuple([np.degrees(l) for l in tr.euler_from_quaternion(tr.quaternion_from_matrix(tll_T_tip))])
            #print 'bl_T_tip  pose %.2f %.2f %.2f' % tuple([np.degrees(l) for l in tr.euler_from_quaternion(tr.quaternion_from_matrix(bl_T_tip))])
            #print '>>>>'

            #Translation that we want
            #goal_trans = tll_T_pose[:,3]
            #rospy.loginfo('New position in torso lift link' + str(tll_T_pose[:,3].T))
            #goal_ps = stamp_pose(mat_to_pose(tll_T_pose), 'torso_lift_link')

            goal_ps = stamp_pose(mat_to_pose(tll_T_tp), 'torso_lift_link')
            #self.linear_movement_as.set_aborted(ptp.LinearMovementResult(gm.Vector3(0,0,0)))

        tstart = rospy.get_time()
        tmax = tstart + self.time_out
        self.controller_manager = ControllerManager()
        rospy.loginfo('Goal is x %f y %f z %f in %s' % (goal_ps.pose.position.x, goal_ps.pose.position.y, goal_ps.pose.position.z, goal_ps.header.frame_id))

        goal_torso = change_pose_stamped_frame(self.tf, goal_ps, 'torso_lift_link')
        rospy.loginfo('Goal is x %f y %f z %f in %s' % (goal_torso.pose.position.x, goal_torso.pose.position.y, goal_torso.pose.position.z, goal_torso.header.frame_id))

        verbose = True

        while True:
            #tfu.tf_as_matrix(self.tf.lookupTransform('base_link', self.tool_frame
            gripper_ps = stamp_pose(mat_to_pose(tfu.tf_as_matrix(self.tf.lookupTransform('torso_lift_link', self.tool_frame, rospy.Time(0)))), 'torso_lift_link')
            #Someone preempted us!
            if self.linear_movement_as.is_preempt_requested():
                #Stop our motion
                self.target_pub.publish(stamp_pose(gripper_ps.pose, gripper_ps.header.frame_id))
                self.linear_movement_as.set_preempted()
                rospy.loginfo('action_cb: preempted!')
                break

            #Calc feedback
            if verbose:
                print 'current_pose %.3f %.3f %.3f in %s' % (gripper_ps.pose.position.x, gripper_ps.pose.position.y, gripper_ps.pose.position.z, gripper_ps.header.frame_id)

            trans, ang, _ = pose_distance(gripper_ps, goal_ps, self.tf)
            #TODO What is trans, verify its type
            #print trans, trans.__class__
            feedback = ptp.LinearMovementFeedback(gm.Vector3(trans[0,0], trans[1,0], trans[2,0]))
            self.linear_movement_as.publish_feedback(feedback)

            #Reached goal
            trans_mag = np.linalg.norm(trans)
            if verbose:
                print trans.T, 'trans_mag', trans_mag, abs(ang), self.rot_tolerance

            if self.trans_tolerance > trans_mag and self.rot_tolerance > abs(ang):
                rospy.loginfo('action_cb: reached goal.')
                break

            #Timed out! is this a failure?
            if rospy.get_time() > tmax:
                break

            #Send controls
            clamped_target = self.clamp_pose(goal_ps, trans_vel, rot_vel, ref_pose=gripper_ps)
            if verbose:
                print 'clamped_target', clamped_target.pose.position.x, clamped_target.pose.position.y, 
                print clamped_target.pose.position.z, clamped_target.header.frame_id, '\n'
            #return
            self.target_pub.publish(clamped_target)
            #if verbose:
            #    print 'sending controls'

        trans, ang, _ = pose_distance(gripper_ps, goal_ps, self.tf)
        result = ptp.LinearMovementResult(gm.Vector3(trans[0,0], trans[1,0], trans[2,0]))
        if self.trans_tolerance > np.linalg.norm(trans):
            rospy.loginfo( 'SUCCEEDED! %.3f ang %.3f' % (np.linalg.norm(trans), np.degrees(ang)))
            self.linear_movement_as.set_succeeded(result)
        else:
            rospy.loginfo('ABORTED! %.3f ang %.3f' % (np.linalg.norm(trans), np.degrees(ang)))
            self.linear_movement_as.set_aborted(result)
        self.controller_manager.joint_mode(self.arm)

        #loop
        # while not at goal and not timed out
        #   clamp pose
        #   send to controller
    
    def clamp_pose(self, desired_pose, max_trans, max_rot, ref_pose):
        current_pose_d = change_pose_stamped_frame(self.tf, ref_pose, desired_pose.header.frame_id) 
        g_T_c  = pose_to_mat(current_pose_d.pose)
        #target_mat = pose_to_mat(pose.pose)

        #target_to_current = current_mat**-1 * target_mat
        ##desired_rot = tf.transformations.quaternion_from_matrix(target_to_current)
        #desired_trans = target_to_current[0:3, 3].copy()
        #target_to_current[0:3, 3] = 0

        #desired_angle, desired_axis, point = tf.transformations.rotation_from_matrix(target_to_current)
            
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

    #controller_manager = ControllerManager()
    #controller_manager.cart_mode('both')
    #rospy.spin()


