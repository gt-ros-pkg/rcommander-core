import roslib; roslib.load_manifest('rcommander')
import rospy
from PyQt4.QtGui import *
from PyQt4.QtCore import *
import tool_utils as tu
import pr2_utils as pu
import numpy as np
import actionlib_msgs.msg as am
import smach


class MoveArmTool(tu.ToolBase):

    def __init__(self, rcommander):
        tu.ToolBase.__init__(self, rcommander, 'move', 'Move')
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

        sstate = MoveArmState(nname, str(self.arm_box.currentText()), joints)
        sstate.set_arm(self.rcommander.left_arm, self.rcommander.right_arm)
        return sstate

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


class MoveArmState(smach.State, tu.StateBase): 

    TIME_OUT = 60

    def __init__(self, name, arm, joints):
        self.name = name
        self.arm = arm
        self.joints = joints
        self.arm_obj = None
        self.__init_unpicklables__()

    def execute(self, userdata):
        self.arm_obj.set_poses(np.matrix(self.joints).T, block=False)
        client = self.arm_obj.client

        succeeded = False
        preempted = False
        r = rospy.Rate(30)
        start_time = rospy.get_time()
        state = client.get_state()
        while True:
            #we have been preempted
            if self.preempt_requested():
                rospy.loginfo('MoveArmState: preempt requested')
                client.cancel_goal()
                self.service_preempt()
                preempted = True
                break

            if (rospy.get_time() - start_time) > MoveArmState.TIME_OUT:
                client.cancel_goal()
                rospy.loginfo('MoveArmState: timed out!')
                break

            if (state not in [am.GoalStatus.ACTIVE, am.GoalStatus.PENDING]):
                if state == am.GoalStatus.SUCCEEDED:
                    rospy.loginfo('MoveArmState: Succeeded!')
                    succeeded = True
                break
            r.sleep()

        if preempted:
            return 'preempted'

        if succeeded:
            return 'succeeded'

        return 'failed'


    def set_arm(self, left, right):
        if self.arm == 'left':
            self.arm_obj = left

        if self.arm == 'right':
            self.arm_obj = right

    def __init_unpicklables__(self):
        tu.StateBase.__init__(self, self.name)
        smach.State.__init__(self, outcomes = ['succeeded', 'preempted', 'failed'], input_keys = [], output_keys = [])

    def __getstate__(self):
        state = tu.StateBase.__getstate__(self)
        my_state = [self.name] #Change this
        return {'state_base': state, 'self': my_state}

    def __setstate__(self, state):
        tu.StateBase.__setstate__(self, state['state_base'])
        self.name = state['self'][0] #Change this
        self.__init_unpicklables__()




