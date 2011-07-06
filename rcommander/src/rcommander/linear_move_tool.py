import tool_utils as tu
#import smach_ros
from PyQt4.QtGui import *
from PyQt4.QtCore import *
import rospy
import tf_utils as tfu
import tf.transformations as tr
import numpy as np
#import move_base_msgs.msg as mm
#import math
from object_manipulator.convert_functions import *
import ptp_arm_action.msg as ptp

##
# Options: stopping criteria
#          relative or absolute
##

#
# controller and view
# create and edits smach states
class LinearMoveTool(tu.ToolBase):

    LEFT_TIP = 'l_gripper_tool_frame'
    RIGHT_TIP = 'r_gripper_tool_frame'

    def __init__(self, rcommander):
        tu.ToolBase.__init__(self, rcommander, 'linear_move', 'Linear Move')
        self.default_frame = 'base_link'
        self.tf_listener = rcommander.tf_listener

    def fill_property_box(self, pbox):
        formlayout = pbox.layout()

        self.arm_box = QComboBox(pbox)
        self.arm_box.addItem('left')
        self.arm_box.addItem('right')

        self.motion_box = QComboBox(pbox)
        self.motion_box.addItem('relative')
        self.motion_box.addItem('absolute')

        self.xline = QLineEdit(pbox)
        self.yline = QLineEdit(pbox)
        self.zline = QLineEdit(pbox)

        self.phi_line   = QLineEdit(pbox)
        self.theta_line = QLineEdit(pbox)
        self.psi_line   = QLineEdit(pbox)
        self.frameline  = QLineEdit(pbox)
        self.frameline.setText(self.default_frame)
        self.pose_button = QPushButton(pbox)
        self.pose_button.setText('Current Pose')
        self.rcommander.connect(self.pose_button, SIGNAL('clicked()'), self.get_current_pose)

        formlayout.addRow("&Arm", self.arm_box)
        formlayout.addRow("&Mode", self.motion_box)
        formlayout.addRow("&x", self.xline)
        formlayout.addRow("&y", self.yline)
        formlayout.addRow("&z", self.zline)

        formlayout.addRow("&phi",   self.phi_line)
        formlayout.addRow("&theta", self.theta_line)
        formlayout.addRow("&psi",   self.psi_line)
        formlayout.addRow("&frame", self.frameline)
        formlayout.addRow(self.pose_button)
        self.reset()

    def get_current_pose(self):
        frame_described_in = str(self.frameline.text())
        left = ('Left' == str(self.arm_box.currentText()))
        right = False
        if not left:
            right = True
            arm_tip_frame = LinearMoveTool.RIGHT_TIP
        else:
            arm_tip_frame = LinearMoveTool.LEFT_TIP
        
        self.tf_listener.waitForTransform(frame_described_in, arm_tip_frame, rospy.Time(), rospy.Duration(2.))
        p_arm = tfu.tf_as_matrix(self.tf_listener.lookupTransform(frame_described_in, arm_tip_frame, rospy.Time(0))) * tr.identity_matrix()
        trans, rotation = tr.translation_from_matrix(p_arm), tr.quaternion_from_matrix(p_arm)

        for value, vr in zip(trans, [self.xline, self.yline, self.zline]):
            vr.setText(str(value))
        for value, vr in zip(tr.euler_from_quaternion(rotation), [self.phi_line, self.theta_line, self.psi_line]):
            vr.setText(str(np.degrees(value)))

    def _create_node(self, name=None):
        trans  = [float(vr.text()) for vr in [self.xline, self.yline, self.zline]]
        angles = [float(vr.text()) for vr in [self.phi_line, self.theta_line, self.psi_line]]
        frame  = str(self.frameline.text())

        if name == None:
            nname = self.name + str(self.counter)
        else:
            nname = name

        return LinearMoveState(nname, trans, angles, 
                str(self.arm_box.currentText()), 
                str(self.motion_box.currentText()), frame)

        #state = NavigateState(nname, xy, theta, frame)
        #return state

    def _node_selected(self, node):
        for value, vr in zip(node.trans, [self.xline, self.yline, self.zline]):
            vr.setText(str(value))
        for value, vr in zip(node.angles, [self.phi_line, self.theta_line, self.psi_line]):
            vr.setText(str(value))
        self.frameline.setText(node.frame)
        self.motion_box.setCurrentIndex(self.motion_box.findText(str(node.motion_type)))

    def reset(self):
        for vr in [self.xline, self.yline, self.zline]:
            vr.setText(str(0.))
        for vr in [self.phi_line, self.theta_line, self.psi_line]:
            vr.setText(str(0.))
        self.frameline.setText(self.default_frame)
        self.motion_box.setCurrentIndex(self.motion_box.findText('relative'))


#
# name maps to tool used to create it
# model
# is a state that can be stuffed into a state machine
class LinearMoveState(tu.SimpleStateBase): # smach_ros.SimpleActionState):


    ##
    #
    # @param name
    # @param trans list of 3 floats
    # @param angles in euler list of 3 floats
    # @param frame 
    def __init__(self, name, trans, angles, arm, motion_type, frame):
        tu.SimpleStateBase.__init__(self, name, \
                arm + '_ptp', ptp.LinearMovementAction, 
                goal_cb_str = 'ros_goal') 
        self.trans = trans
        self.angles = angles #convert angles to _quat
        self.arm = arm
        self.motion_type = motion_type
        self.frame = frame

    def ros_goal(self, userdata, default_goal):
        goal = ptp.LinearMovementGoal()
        if self.motion_type == 'relative':
            goal.relative = True
        elif self.motion_type == 'absolute':
            goal.relative = False
        else:
            raise RuntimeError('Invalid motion type given.')
        pose = mat_to_pose(np.matrix(tr.translation_matrix(self.trans)) * np.matrix(tr.quaternion_matrix(self._quat)))
        goal.goal = stamp_pose(pose, self.frame)
        return goal

    def _set_angles(self, euler_angs):
        ang_rad = [np.radians(e) for e in euler_angs]
        #self._quat = tr.quaternion_from_euler(euler_angs[0], euler_angs[1], euler_angs[2])
        self._quat = tr.quaternion_from_euler(*ang_rad)
    
    def _get_angles(self):
        return [np.degrees(e) for e in tr.euler_from_quaternion(self._quat)]
        
    angles = property(_get_angles, _set_angles)

    def __getstate__(self):
        state = tu.SimpleStateBase.__getstate__(self)
        my_state = [self.trans, self._quat, self.arm, self.motion_type, self.frame]
        return {'simple_state': state, 'self': my_state}

    def __setstate__(self, state):
        tu.SimpleStateBase.__setstate__(self, state['simple_state'])
        self.trans, self._quat, self.arm, self.motion_type, self.frame = state['self']

