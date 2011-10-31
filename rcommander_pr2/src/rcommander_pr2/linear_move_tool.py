import roslib; roslib.load_manifest('rcommander_pr2')
import rcommander_core.tool_utils as tu
#import smach_ros
from PyQt4.QtGui import *
from PyQt4.QtCore import *
import rospy
import tf_utils as tfu
import tf.transformations as tr
import numpy as np
from object_manipulator.convert_functions import *
import ptp_arm_action.msg as ptp
import math
import geometry_msgs.msg as geo
import actionlib 
import smach
import actionlib_msgs.msg as am

#import rcommander_core.point_tool as ptl

#import move_base_msgs.msg as mm
#import math
##
# Options: stopping criteria
#          relative or absolute
##

#
# controller and view
# create and edits smach states
class LinearMoveTool(tu.ToolBase):

    #LEFT_TIP = 'l_gripper_tool_frame'
    #RIGHT_TIP = 'r_gripper_tool_frame'
    LEFT_TIP = rospy.get_param('/l_cart/tip_name')
    RIGHT_TIP = rospy.get_param('/r_cart/tip_name')

    def __init__(self, rcommander):
        tu.ToolBase.__init__(self, rcommander, 'linear_move', 'Linear Move', LinearMoveState)
        self.default_frame = '/torso_lift_link'
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
        #nodes = self.rcommander.outputs_of_type(ptl.Point3DState)
        nodes = self.rcommander.outputs_of_type(geo.PointStamped)
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

        self.frame_box = QComboBox(pbox)
        for f in self.tf_listener.getFrameStrings():
            self.frame_box.addItem(f)

        self.pose_button = QPushButton(pbox)
        self.pose_button.setText('Current Pose')
        self.rcommander.connect(self.pose_button, SIGNAL('clicked()'), self.get_current_pose)

        self.time_box = QDoubleSpinBox(pbox)
        self.time_box.setMinimum(0)
        self.time_box.setMaximum(1000.)
        self.time_box.setSingleStep(.2)

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
        formlayout.addRow("&frame", self.frame_box)
        formlayout.addRow('&Time Out', self.time_box)
        formlayout.addRow(self.pose_button)
        self.reset()

    def source_changed(self, index):
        self.source_box.setCurrentIndex(index)
        if str(self.source_box.currentText()) != ' ':
            self.xline.setEnabled(False)
            self.yline.setEnabled(False)
            self.zline.setEnabled(False)

            self.phi_line.setEnabled(False)
            self.theta_line.setEnabled(False)
            self.psi_line.setEnabled(False)

            self.frame_box.setEnabled(False)
            self.pose_button.setEnabled(False)
            self.motion_box.setCurrentIndex(self.motion_box.findText('absolute'))

        else:
            self.xline.setEnabled(True)
            self.yline.setEnabled(True)
            self.zline.setEnabled(True)

            self.phi_line.setEnabled(True)
            self.theta_line.setEnabled(True)
            self.psi_line.setEnabled(True)

            self.frame_box.setEnabled(True)
            self.pose_button.setEnabled(True)

    def get_current_pose(self):
        frame_described_in = str(self.frame_box.currentText())
        left = ('left' == str(self.arm_box.currentText()))
        if not left:
            arm_tip_frame = LinearMoveTool.RIGHT_TIP
        else:
            arm_tip_frame = LinearMoveTool.LEFT_TIP
        
        #print 'getting pose for', arm_tip_frame, left, str(self.arm_box.currentText())
        self.tf_listener.waitForTransform(frame_described_in, arm_tip_frame, rospy.Time(), rospy.Duration(2.))
        p_arm = tfu.tf_as_matrix(self.tf_listener.lookupTransform(frame_described_in, arm_tip_frame, rospy.Time(0)))
        trans, rotation = tr.translation_from_matrix(p_arm), tr.quaternion_from_matrix(p_arm)

        for value, vr in zip(trans, [self.xline, self.yline, self.zline]):
            vr.setText("%.3f" % value)
        for value, vr in zip(tr.euler_from_quaternion(rotation), [self.phi_line, self.theta_line, self.psi_line]):
            vr.setText("%.3f" % np.degrees(value))

        self.motion_box.setCurrentIndex(self.motion_box.findText('absolute'))

    def new_node(self, name=None):
        trans  = [float(vr.text()) for vr in [self.xline, self.yline, self.zline]]
        angles = [float(vr.text()) for vr in [self.phi_line, self.theta_line, self.psi_line]]
        frame  = str(self.frame_box.currentText())
        trans_vel = float(str(self.trans_vel_line.text()))
        rot_vel   = float(str(self.rot_vel_line.text()))
        source_name = str(self.source_box.currentText())
        timeout = self.time_box.value()

        if source_name == ' ':
            source_name = None

        if name == None:
            nname = self.name + str(self.counter)
        else:
            nname = name

        return LinearMoveState(nname, trans, angles, 
                str(self.arm_box.currentText()), [trans_vel, rot_vel],
                str(self.motion_box.currentText()), source_name, frame, timeout)

        #state = NavigateState(nname, xy, theta, frame)
        #return state

    def set_node_properties(self, node):
        for value, vr in zip(node.trans, [self.xline, self.yline, self.zline]):
            vr.setText(str(value))
        for value, vr in zip(node.get_angles(), [self.phi_line, self.theta_line, self.psi_line]):
            vr.setText(str(value))

        self.frame_box.setCurrentIndex(self.frame_box.findText(str(node.frame)))
        self.motion_box.setCurrentIndex(self.motion_box.findText(str(node.motion_type)))
        self.arm_box.setCurrentIndex(self.arm_box.findText(node.arm))

        self.trans_vel_line.setText(str(node.vels[0]))
        self.rot_vel_line.setText(str(node.vels[1]))
        self.time_box.setValue(node.timeout)

        source_name = node.remapping_for('point')
        if source_name == None:
            source_name = ' '
            index = self.source_box.findText(source_name)
        else:
            index = self.source_box.findText(source_name)
            if index == -1:
                self.source_box.addItem(source_name)
                index = self.source_box.findText(source_name)

        #print '>>>>>>>>>>>>>>>>> Source for linear node is', index
        self.source_changed(index)

    def reset(self):
        for vr in [self.xline, self.yline, self.zline]:
            vr.setText(str(0.0))
        for vr in [self.phi_line, self.theta_line, self.psi_line]:
            vr.setText(str(0.0))

        self.frame_box.setCurrentIndex(self.frame_box.findText(self.default_frame))
        self.motion_box.setCurrentIndex(self.motion_box.findText('relative'))
        self.trans_vel_line.setText(str(.02))
        self.rot_vel_line.setText(str(.16))
        self.time_box.setValue(20)


class LinearMoveState(tu.StateBase): # smach_ros.SimpleActionState):
    ##
    #
    # @param name
    # @param trans list of 3 floats
    # @param angles in euler list of 3 floats
    # @param frame 
    def __init__(self, name, trans, angles, arm, vels, motion_type, source, frame, timeout):
        tu.StateBase.__init__(self, name)
        self.set_remapping_for('point', source)
        self.trans = trans
        #self.angles = angles #convert angles to _quat
        self.set_angles(angles)
        self.arm = arm
        self.vels = vels
        self.motion_type = motion_type
        self.frame = frame
        self.timeout = timeout

    def set_angles(self, euler_angs):
        ang_rad = [np.radians(e) for e in euler_angs]
        #self._quat = tr.quaternion_from_euler(euler_angs[0], euler_angs[1], euler_angs[2])
        self.quat = tr.quaternion_from_euler(*ang_rad)
    
    def get_angles(self):
        return [np.degrees(e) for e in tr.euler_from_quaternion(self.quat)]
        
    #angles = property(_get_angles, _set_angles)

    def get_smach_state(self):
        return LinearMovementSmach(motion_type = self.motion_type, arm = self.arm, trans = self.trans, 
                quat = self.quat, frame = self.frame, vels = self.vels, 
                source_for_point = self.remapping_for('point'), timeout=self.timeout)

class LinearMovementSmach(smach.State):

    def __init__(self, motion_type, arm, trans, quat, frame, vels, source_for_point, timeout):
        smach.State.__init__(self, outcomes = ['succeeded', 'preempted', 'failed'], input_keys = ['point'], output_keys = [])

        self.motion_type = motion_type
        self.arm = arm

        self.trans = trans
        self.quat = quat
        self.frame = frame

        self.vels = vels
        self.source_for_point = source_for_point
        self.timeout = timeout

        self.action_client = actionlib.SimpleActionClient(arm + '_ptp', ptp.LinearMovementAction)

    def set_robot(self, robot):
        self.pr2 = robot

    def ros_goal(self, userdata):
        goal = ptp.LinearMovementGoal()
        if self.motion_type == 'relative':
            goal.relative = True
        elif self.motion_type == 'absolute':
            goal.relative = False
        else:
            raise RuntimeError('Invalid motion type given.')

        #quat = self._quat
        quat = self.quat
        if self.source_for_point != None:
            trans, frame = userdata.point
            if self.arm == 'left':
                tip = rospy.get_param('/l_cart/tip_name')
            if self.arm == 'right':
                tip = rospy.get_param('/r_cart/tip_name')
            quat = self.pr2.tf_listener.lookupTransform(frame, tip, rospy.Time(0))[1]
        else:
            trans = self.trans
            frame = self.frame

        pose = mat_to_pose(np.matrix(tr.translation_matrix(trans)) * np.matrix(tr.quaternion_matrix(quat)))
        goal.goal = stamp_pose(pose, frame)
        goal.trans_vel = self.vels[0]
        goal.rot_vel = self.vels[1]
        return goal

    #TODO abstract this out!
    def execute(self, userdata):
        goal = self.ros_goal(userdata)
        print 'goal sent is', goal
        self.action_client.send_goal(goal)
       
        succeeded = False
        preempted = False
        r = rospy.Rate(30)
        start_time = rospy.get_time()

        while True:
            #we have been preempted
            if self.preempt_requested():
                rospy.loginfo('LinearMoveStateSmach: preempt requested')
                self.action_client.cancel_goal()
                self.service_preempt()
                preempted = True
                break

            if (rospy.get_time() - start_time) > self.timeout:
                self.action_client.cancel_goal()
                rospy.loginfo('LinearMoveStateSmach: timed out!')
                succeeded = False
                break

            #print tu.goal_status_to_string(state)
            state = self.action_client.get_state()
            if (state not in [am.GoalStatus.ACTIVE, am.GoalStatus.PENDING]):
                if state == am.GoalStatus.SUCCEEDED:
                    rospy.loginfo('LinearMoveStateSmach: Succeeded!')
                    succeeded = True
                break

            r.sleep()

        if preempted:
            return 'preempted'

        if succeeded:
            return 'succeeded'

        return 'failed'


##
## name maps to tool used to create it
## model
## is a state that can be stuffed into a state machine
#class LinearMoveState(tu.SimpleStateBase): # smach_ros.SimpleActionState):
#    ##
#    #
#    # @param name
#    # @param trans list of 3 floats
#    # @param angles in euler list of 3 floats
#    # @param frame 
#    def __init__(self, name, trans, angles, arm, vels, motion_type, source, frame):
#        tu.SimpleStateBase.__init__(self, name, \
#                arm + '_ptp', ptp.LinearMovementAction, 
#                goal_cb_str = 'ros_goal', input_keys=['point']) 
#        self.set_remapping_for('point', source)
#        #self.register_input_keys(['point'])
#        #print 'registered input keys', self.get_registered_input_keys()
#
#        self.trans = trans
#        self.angles = angles #convert angles to _quat
#        self.arm = arm
#        self.vels = vels
#        self.motion_type = motion_type
#        self.frame = frame
#
#    def set_robot(self, pr2):
#        self.pr2 = pr2
#
#    def get_smach_state(self):
#        state = tu.SimpleStateBase.get_smach_state(self)
#        state.set_robot = self.
#
#    def ros_goal(self, userdata, default_goal):
#        #print 'LinearMoveState: rosgoal called!!!!!!!!!!!!!!1'
#        goal = ptp.LinearMovementGoal()
#        if self.motion_type == 'relative':
#            goal.relative = True
#        elif self.motion_type == 'absolute':
#            goal.relative = False
#        else:
#            raise RuntimeError('Invalid motion type given.')
#
#        quat = self._quat
#        if self.source_for('point') != None:
#            trans, frame = userdata.point
#            if self.arm == 'left':
#                tip = rospy.get_param('/l_cart/tip_name')
#            if self.arm == 'right':
#                tip = rospy.get_param('/r_cart/tip_name')
#            quat = self.pr2.tf_listener.lookupTransform(frame, tip, rospy.Time(0))[1]
#        else:
#            trans = self.trans
#            frame = self.frame
#
#        pose = mat_to_pose(np.matrix(tr.translation_matrix(trans)) * np.matrix(tr.quaternion_matrix(quat)))
#        goal.goal = stamp_pose(pose, frame)
#        goal.trans_vel = self.vels[0]
#        goal.rot_vel = self.vels[1]
#        #print 'returned goal'
#        return goal
#
#    def _set_angles(self, euler_angs):
#        ang_rad = [np.radians(e) for e in euler_angs]
#        #self._quat = tr.quaternion_from_euler(euler_angs[0], euler_angs[1], euler_angs[2])
#        self._quat = tr.quaternion_from_euler(*ang_rad)
#    
#    def _get_angles(self):
#        return [np.degrees(e) for e in tr.euler_from_quaternion(self._quat)]
#        
#    angles = property(_get_angles, _set_angles)
#
#    #def __getstate__(self):
#    #    state = tu.SimpleStateBase.__getstate__(self)
#    #    my_state = [self.trans, self._quat, self.arm, self.vels, self.motion_type, self.frame]
#    #    return {'simple_state': state, 'self': my_state}
#
#    #def __setstate__(self, state):
#    #    tu.SimpleStateBase.__setstate__(self, state['simple_state'])
#    #    self.trans, self._quat, self.arm, self.vels, self.motion_type, self.frame = state['self']
#
#class SimpleStateBaseSmach(smach_ros.SimpleActionState):
#
#    def __init__(self, action_name, action_type, goal_obj, goal_cb_str, input_keys, output_keys):
#        smach_ros.SimpleActionState.__init__(self, action_name, action_type, 
#                goal_cb = SimpleStateCB(eval('goal_obj.%s' % goal_cb_str), input_keys, output_keys))
#        self.goal_obj = goal_obj
#
#    def __call__(self, userdata, default_goal): 
#        f = eval('self.goal_obj.%s' % self.goal_cb_str)
#        return f(userdata, default_goal)
#
#===============================================================================
#===============================================================================
#===============================================================================
#===============================================================================
