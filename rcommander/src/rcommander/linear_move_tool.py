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
import math
import point_tool as ptl

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

        self.source_box = QComboBox(pbox)
        self.source_box.addItem(' ')
        nodes = self.rcommander.global_nodes(ptl.Point3DState)
        for n in nodes:
            self.source_box.addItem(n)
        self.rcommander.connect(self.source_box, SIGNAL('currentIndexChanged(int)'), self.source_changed)

        self.xline = QLineEdit(pbox)
        self.yline = QLineEdit(pbox)
        self.zline = QLineEdit(pbox)

        self.phi_line   = QLineEdit(pbox)
        self.theta_line = QLineEdit(pbox)
        self.psi_line   = QLineEdit(pbox)

        self.trans_vel_line = QLineEdit(pbox)
        self.rot_vel_line = QLineEdit(pbox)

        self.frameline  = QLineEdit(pbox)
        self.frameline.setText(self.default_frame)
        self.pose_button = QPushButton(pbox)
        self.pose_button.setText('Current Pose')
        self.rcommander.connect(self.pose_button, SIGNAL('clicked()'), self.get_current_pose)

        formlayout.addRow("&Arm", self.arm_box)
        formlayout.addRow("&Mode", self.motion_box)
        formlayout.addRow("&Point Input", self.source_box)
        formlayout.addRow("&x", self.xline)
        formlayout.addRow("&y", self.yline)
        formlayout.addRow("&z", self.zline)

        formlayout.addRow("&phi",   self.phi_line)
        formlayout.addRow("&theta", self.theta_line)
        formlayout.addRow("&psi",   self.psi_line)

        formlayout.addRow('&Translational Vel', self.trans_vel_line)
        formlayout.addRow('&Rotational Vel', self.rot_vel_line)
        formlayout.addRow("&frame", self.frameline)
        formlayout.addRow(self.pose_button)
        self.reset()

    def source_changed(self, index):
        self.source_box.setCurrentIndex(index)
        if str(self.source_box.currentText()) != ' ':
            self.xline.setEnabled(False)
            self.yline.setEnabled(False)
            self.zline.setEnabled(False)
            self.frameline.setEnabled(False)
            self.pose_button.setEnabled(False)
        else:
            self.xline.setEnabled(True)
            self.yline.setEnabled(True)
            self.zline.setEnabled(True)
            self.frameline.setEnabled(True)
            self.pose_button.setEnabled(True)

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
        trans_vel = float(str(self.trans_vel_line.text()))
        rot_vel   = float(str(self.rot_vel_line.text()))
        source_name = str(self.source_box.currentText())
        if source_name == ' ':
            source_name = None

        if name == None:
            nname = self.name + str(self.counter)
        else:
            nname = name

        return LinearMoveState(nname, trans, angles, 
                str(self.arm_box.currentText()), [trans_vel, rot_vel],
                str(self.motion_box.currentText()), source_name, frame)

        #state = NavigateState(nname, xy, theta, frame)
        #return state

    def _node_selected(self, node):
        for value, vr in zip(node.trans, [self.xline, self.yline, self.zline]):
            vr.setText(str(value))
        for value, vr in zip(node.angles, [self.phi_line, self.theta_line, self.psi_line]):
            vr.setText(str(value))
        self.frameline.setText(node.frame)
        self.motion_box.setCurrentIndex(self.motion_box.findText(str(node.motion_type)))
        self.arm_box.setCurrentIndex(self.arm_box.findText(node.arm))

        source_name = node.source_for('point')
        if source_name == None:
            source_name = ' '
        index = self.source_box.findText(source_name)
        self.source_changed(index)

    def reset(self):
        for vr in [self.xline, self.yline, self.zline]:
            vr.setText(str(0.0))
        for vr in [self.phi_line, self.theta_line, self.psi_line]:
            vr.setText(str(0.0))
        self.frameline.setText(self.default_frame)
        self.motion_box.setCurrentIndex(self.motion_box.findText('relative'))
        self.trans_vel_line.setText(str(.02))
        self.rot_vel_line.setText(str(.16))

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
    def __init__(self, name, trans, angles, arm, vels, motion_type, source, frame):
        tu.SimpleStateBase.__init__(self, name, \
                arm + '_ptp', ptp.LinearMovementAction, 
                goal_cb_str = 'ros_goal', input_keys=['point']) 
        self.set_source_for('point', source)
        #self.register_input_keys(['point'])
        #print 'registered input keys', self.get_registered_input_keys()

        self.trans = trans
        self.angles = angles #convert angles to _quat
        self.arm = arm
        self.vels = vels
        self.motion_type = motion_type
        self.frame = frame

    def ros_goal(self, userdata, default_goal):
        #print 'LinearMoveState: rosgoal called!!!!!!!!!!!!!!1'
        goal = ptp.LinearMovementGoal()
        if self.motion_type == 'relative':
            goal.relative = True
        elif self.motion_type == 'absolute':
            goal.relative = False
        else:
            raise RuntimeError('Invalid motion type given.')

        if self.source_for('point') != None:
            trans, frame = userdata.point
        else:
            trans = self.trans
            frame = self.frame

        pose = mat_to_pose(np.matrix(tr.translation_matrix(trans)) * np.matrix(tr.quaternion_matrix(self._quat)))
        goal.goal = stamp_pose(pose, frame)
        goal.trans_vel = self.vels[0]
        goal.rot_vel = self.vels[1]
        #print 'returned goal'
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
        my_state = [self.trans, self._quat, self.arm, self.vels, self.motion_type, self.frame]
        return {'simple_state': state, 'self': my_state}

    def __setstate__(self, state):
        tu.SimpleStateBase.__setstate__(self, state['simple_state'])
        self.trans, self._quat, self.arm, self.vels, self.motion_type, self.frame = state['self']

