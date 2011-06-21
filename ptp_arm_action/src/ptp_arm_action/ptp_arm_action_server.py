import roslib; roslib.load_manifest('ptp_arm_action')
import rospy
import actionlib

import geometry_msgs.msg as gm
import pr2_mechanism_msgs.srv as pmm
import ptp_arm_action.msg as ptp


class ControllerManager:

    def __init__(self):
        # LoadController        
        self.load = rospy.ServiceProxy('pr2_controller_manager/load_controller', pmm.LoadController)
        # UnloadController        
        self.unload = rospy.ServiceProxy('pr2_controller_manager/unload_controller', pmm.UnloadController)
        # SwitchController
        self._switch_controller = rospy.ServiceProxy('pr2_controller_manager/switch_controller', pmm.SwitchController)

    def switch(self, start_con, stop_con):
        for n in start_con:
            self.load(n)
        resp = self._switch_controller(start_con, stop_con, pmm.SwitchControllerRequest.STRICT)
        for n in stop_con:
            self.unload(n)
        return resp.ok


class PTPArmActionServer:

    def __init__(self, name, arm):
        if arm == 'left':
            self.controller = 'l_cart'
            self.joint_controller = 'l_arm_controller'
        elif
            self.controller = 'r_cart'
            self.joint_controller = 'r_arm_controller'

        self.target_pub = rospy.Publisher(controller + '/command_pose', PoseStamped)
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, ptp.LinearMovementAction, execute_cb=self.action_cb)
        self._as.start()


    def action_cb(self, goal):

    def clamp_pose(self, pose):
        current_pose = change_pose_stamped_frame(self.tf, self.last_pose_msg, pose.header.frame_id) 
        current_mat = pose_to_mat(current_pose.pose)
        target_mat = pose_to_mat(pose.pose)

        target_to_current = current_mat**-1 * target_mat
        #desired_rot = tf.transformations.quaternion_from_matrix(target_to_current)
        desired_trans = target_to_current[0:3, 3].copy()
        desired_trans_mag = scipy.linalg.norm(desired_trans)
        target_to_current[0:3, 3] = 0

        desired_angle, desired_axis, point = tf.transformations.rotation_from_matrix(target_to_current)
            
        # TODO These are magic numbers... and are probably rate dependent...
        MAX_TRANS = 0.02
        MAX_ROT = pi/20

        frac_trans = fabs(desired_trans_mag / MAX_TRANS)
        frac_rot = fabs(desired_angle / MAX_ROT)

        if frac_trans <= 1 and frac_rot <= 1:
            return pose
        frac = max(frac_rot, frac_trans)


        clamped_angle = desired_angle / frac
        clamped_trans = desired_trans / frac

        clamped_transformation = scipy.matrix(tf.transformations.rotation_matrix(clamped_angle, desired_axis))
        clamped_transformation[0:3, 3] = clamped_trans
        clamped_mat = current_mat * clamped_transformation
        clamped_pose = stamp_pose(mat_to_pose(clamped_mat), pose.header.frame_id)
        ''' 
        print "current_mat:\n", ppmat(current_mat)
        print "target_mat:\n", ppmat(target_mat)
        print "target_to_current:\n", ppmat(target_to_current)
        print "desired_trans:", ppmat(desired_trans)
        print "desired_trans_mag:", desired_trans_mag
        print "desired_angle:", desired_angle
        print "desired_axis:", pplist(desired_axis)
        print "frac_trans:", frac_trans
        print "frac_rot:", frac_rot
        print "frac:", frac
        print "clamped_angle:", clamped_angle
        print "clamped_trans:", clamped_trans
        print "clamped_transformation:\n", ppmat(clamped_transformation)
        print "clamped_mat:\n", ppmat(clamped_mat)
        print 'Clamped_pose: \n', clamped_pose
        '''
        return clamped_pose
