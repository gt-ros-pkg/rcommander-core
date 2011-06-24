import roslib; roslib.load_manifest('ptp_arm_action')
import rospy
import actionlib

import geometry_msgs.msg as gm
import pr2_mechanism_msgs.srv as pmm
import ptp_arm_action.msg as ptp

from object_manipulator.convert_functions import *
from math import sqrt, pi, fabs
import math
#import scipy
import numpy as np
import tf


class ControllerManager:

    def __init__(self):
        # LoadController        
        self.load = rospy.ServiceProxy('pr2_controller_manager/load_controller', pmm.LoadController)

        # UnloadController        
        self.unload = rospy.ServiceProxy('pr2_controller_manager/unload_controller', pmm.UnloadController)

        # SwitchController
        self._switch_controller = rospy.ServiceProxy('pr2_controller_manager/switch_controller', pmm.SwitchController)
        #self._list_controllers = rospy.ServiceProxy('pr2_controller_manager/list_controllers', pmm.ListControllers)

        self.joint_controllers = {}
        self.cart_controllers = {}
        for arm in ['l', 'r']:
            self.joint_controllers[arm] = arm + '_arm_controller'
            self.cart_controllers[arm] = cart_controller_name = arm + '_cart'

    def switch(self, start_con, stop_con):
        print 'switching from', start_con, 'to', stop_con
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
# Measures the distance between two poses, independently factors out distance in rotation and distance in translation
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
        elif arm == 'right':
            self.controller = 'r_cart'
            self.joint_controller = 'r_arm_controller'
        else:
            raise RuntimeError('Invalid parameter for arm: %s' % arm)

        self.target_pub = rospy.Publisher(self.controller + '/command_pose', gm.PoseStamped)
        self.pose_sub = rospy.Subscriber(self.controller + '/state/x', gm.PoseStamped, self.pose_callback)
        self.tf = tf.TransformListener()
        self.trans_tolerance = .005
        self.rot_tolerance = math.radians(5.)
        self.time_out = 30.

        self.controller_manager = ControllerManager()
        self.controller_manager.cart_mode('both')
        self._action_name = name
        self.linear_movement_as = actionlib.SimpleActionServer(self._action_name, ptp.LinearMovementAction, execute_cb=self.action_cb, auto_start=False)
        self.linear_movement_as.start()

    def pose_callback(self, data):
        self.last_pose_msg = data

    def action_cb(self, msg):
        self.controller_manager.cart_mode('both')
        success = False
        r = rospy.Rate(100) 
        goal_ps = msg.goal
        tstart = rospy.get_time()
        tmax = tstart + self.time_out
        self.controller_manager = ControllerManager()
        print 'goal is', goal_ps.pose.position

        while True:
            if self.linear_movement_as.is_preempt_requested():
                #Someone preempted us!
                self.linear_movement_as.set_preempted()
                rospy.loginfo('action_cb: preempted!')
                break

            #Calc feedback
            #print self.last_pose_msg.__class__, goal_ps.__class__
            print 'curent_pose', self.last_pose_msg.pose.position
            trans, ang, _ = pose_distance(self.last_pose_msg, goal_ps, self.tf)
            #TODO What is trans, verify its type
            #print trans, trans.__class__
            feedback = ptp.LinearMovementFeedback(gm.Vector3(trans[0,0], trans[1,0], trans[2,0]))
            self.linear_movement_as.publish_feedback(feedback)

            #Reached goal
            trans_mag = np.linalg.norm(trans)
            print 'trans_mag', trans_mag
            if self.trans_tolerance > trans_mag:
                rospy.loginfo('action_cb: reached goal.')
                break

            #Timed out! is this a failure?
            if rospy.get_time() > tmax:
                break

            #Send controls
            clamped_target = self.clamp_pose(goal_ps)
            self.target_pub.publish(clamped_target)
            print 'sending controls'

        trans, ang, _ = pose_distance(self.last_pose_msg, goal_ps, self.tf)
        result = ptp.LinearMovementResult(gm.Vector3(trans[0,0], trans[1,0], trans[2,0]))
        if self.trans_tolerance < np.linalg.norm(trans):
            self.linear_movement_as.set_succeeded(result)
        else:
            self.linear_movement_as.set_aborted(result)

        #loop
        # while not at goal and not timed out
        #   clamp pose
        #   send to controller
    
    def clamp_pose(self, desired_pose, max_trans=.02, max_rot=pi/20):
        current_pose_d = change_pose_stamped_frame(self.tf, self.last_pose_msg, desired_pose.header.frame_id) 
        g_T_c  = pose_to_mat(current_pose_d.pose)
        #target_mat = pose_to_mat(pose.pose)

        #target_to_current = current_mat**-1 * target_mat
        ##desired_rot = tf.transformations.quaternion_from_matrix(target_to_current)
        #desired_trans = target_to_current[0:3, 3].copy()
        #target_to_current[0:3, 3] = 0

        #desired_angle, desired_axis, point = tf.transformations.rotation_from_matrix(target_to_current)
            
        desired_trans, desired_angle, desired_axis = pose_distance(self.last_pose_msg, desired_pose, self.tf)
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
    rospy.init_node('ptp')
    left_as = PTPArmActionServer('left_ptp', 'left')
    rospy.loginfo('PTP action server started.')
    rospy.spin()

    #controller_manager = ControllerManager()
    #controller_manager.cart_mode('both')
    #rospy.spin()


