import roslib; roslib.load_manifest('rcommander')
import rospy
from PyQt4.QtGui import *
from PyQt4.QtCore import *
import tool_utils as tu
import arm_navigation_msgs.msg as an
import actionlib
import actionlib_msgs.msg as am
import numpy as np
import smach


class SafeMoveArmTool(tu.ToolBase):

    def __init__(self, rcommander):
        tu.ToolBase.__init__(self, rcommander, 'save_move', 'Safe Move')
        #self.left, self.right = pu.PR2Arm.create_arms(rcommander.tf_listener, 'both')
        self.joint_name_fields = ["shoulder_pan_joint", "shoulder_lift_joint", "upper_arm_roll_joint", 
                                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]

    def fill_property_box(self, pbox):
        formlayout = pbox.layout()

        self.arm_box = QComboBox(pbox)
        self.arm_box.addItem('left')
        self.arm_box.addItem('right')
        formlayout.addRow('&Arm', self.arm_box)

        #Controls for displaying the current joint states
        for name in self.joint_name_fields:
            exec("self.%s = QLineEdit(pbox)" % name)
            exec("formlayout.addRow(\"&\" + name, self.%s)" % name)

        #Controls for getting the current joint states
        self.pose_button = QPushButton(pbox)
        self.pose_button.setText('Current Joint Angles')
        self.rcommander.connect(self.pose_button, SIGNAL('clicked()'), self.get_current_joint_angles)
        formlayout.addRow(self.pose_button)
        self.reset()


    def get_current_joint_angles(self):
        if ('Left' == str(self.arm_box.currentText())):
            arm_obj = self.rcommander.left_arm
        else:
            arm_obj = self.rcommander.right_arm

        pose_mat = arm_obj.pose()
        for idx, name in enumerate(self.joint_name_fields):
            deg = np.degrees(pose_mat[idx, 0])
            exec('line_edit = self.%s' % name)
            line_edit.setText(str(deg))


    def _create_node(self, name=None):
        if name == None:
            nname = self.name + str(self.counter)
        else:
            nname = name

        joints = []
        for name in self.joint_name_fields:
            exec('rad = np.radians(float(str(self.%s.text())))' % name)
            joints.append(rad)
        return SafeMoveArmState(nname, str(self.arm_box.currentText()), joints)

    def _node_selected(self, my_node):
        self.arm_box.setCurrentIndex(self.arm_box.findText(my_node.arm))
        for idx, name in enumerate(self.joint_name_fields):
            deg = np.degrees(my_node.joints[idx])
            exec('line_edit = self.%s' % name)
            line_edit.setText(str(deg))

    def reset(self):
        self.arm_box.setCurrentIndex(self.arm_box.findText('left'))
        for name in self.joint_name_fields:
            exec('self.%s.setText(str(0.))' % name)


class SafeMoveArmState(smach.State, tu.StateBase): 

    TIME_OUT = 60

    # @param name
    # @param arm 'left' or 'right'
    def __init__(self, name, arm, joints):
        self.name = name
        self.arm = arm
        self.joints = joints
        self.__init_unpicklables__()

    def _still_going(self):
        state = self.action_client.get_state()
        gripper_event_detected = state not in [am.GoalStatus.ACTIVE, am.GoalStatus.PENDING]
        return gripper_event_detected

    def execute(self, userdata):        
        #Construct goal and send it
        goal = an.MoveArmGoal()
        goal.motion_plan_request.group_name = self.group_name
        goal.motion_plan_request.num_planning_attempts = 2;
        goal.motion_plan_request.allowed_planning_time = rospy.Duration(5.0);
        goal.motion_plan_request.planner_id = ""
        goal.planner_service_name = "ompl_planning/plan_kinematic_path"

        for (joint_name, joint_angle) in zip(self.joint_names, self.joints):
            joint_constraint = an.JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = joint_angle
            joint_constraint.tolerance_below = .1
            joint_constraint.tolerance_above = .1
            goal.motion_plan_request.goal_constraints.joint_constraints.append(joint_constraint) 
        self.move_arm_client.send_goal(goal)
    
        #Wait for action to finish
        r = rospy.Rate(30)
        start_time = rospy.get_time()
        state = self.move_arm_client.get_state()
        preempted = False
        succeeded = False
        while True:
            #we have been preempted
            if self.preempt_requested():
                rospy.loginfo('SafeMoveArmState: preempt requested')
                self.move_arm_client.cancel_goal()
                self.service_preempt()
                preempted = True
                break
            
            #we timed out
            if (rospy.get_time() - start_time) > SafeMoveArmState.TIME_OUT:
                self.move_arm_client.cancel_goal()
                rospy.loginfo('SafeMoveArmState: timed out!')
                break

            if (state not in [am.GoalStatus.ACTIVE, am.GoalStatus.PENDING]):
                if state == am.GoalStatus.SUCCEEDED:
                    if result.error_code.val == 1:
                        rospy.loginfo('SafeMoveArmState: Succeeded!')
                        succeeded = True
                    #elif result.error_code.val == ArmNavigationErrorCodes.START_STATE_IN_COLLISION:
                    #    succeeded = False
                break
            r.sleep()

        if preempted:
            return 'preempted'

        if succeeded:
            return 'succeeded'

        return 'failed'


    def __init_unpicklables__(self):
        tu.StateBase.__init__(self, self.name)
        smach.State.__init__(self, outcomes = ['succeeded', 'preempted', 'failed'], input_keys = [], output_keys = [])

        if self.arm == 'left':
            self.move_arm_client = actionlib.SimpleActionClient('move_left_arm', an.MoveArmAction)
            self.joint_names = rospy.get_param('/l_arm_controller/joints')
            self.group_name = 'left_arm'

        if self.arm == 'right':
            self.move_arm_client = actionlib.SimpleActionClient('move_right_arm', an.MoveArmAction)
            self.joint_names = rospy.get_param('/r_arm_controller/joints')
            self.group_name = 'right_arm'


    def __getstate__(self):
        state = tu.StateBase.__getstate__(self)
        my_state = [self.name, self.arm, self.joints] #Change this
        return {'state_base': state, 'self': my_state}


    def __setstate__(self, state):
        tu.StateBase.__setstate__(self, state['state_base'])
        self.name, self.arm, self.joints = state['self']
        self.__init_unpicklables__()





