import rospy
from PyQt4.QtGui import *
from PyQt4.QtCore import *
import tool_utils as tu
import numpy as np

class MoveHeadTool(tu.ToolBase):

    def __init__(self, rcommander):
        tu.ToolBase.__init__(self, rcommander, 'move_head_ang', 'Move Head (ang)', MoveHeadState)

    def fill_property_box(self, pbox):
        formlayout = pbox.layout()

        joint_names = self.rcommander.robot.head.get_joint_names()
        self.joint_boxes = []
        for jname in joint_names:
            joint_box = QDoubleSpinBox(pbox)
            joint_box.setMinimum(-9999.)
            joint_box.setMaximum(9999.)
            joint_box.setSingleStep(1.)
            self.joint_boxes.append(joint_box)
            formlayout.addRow('&%s' % jname, self.joint_box)

        self.current_pose_button = QPushButton(pbox)
        self.current_pose_button.setText('Current Pose')
        self.rcommander.connect(self.current_pose_button, 
                SIGNAL('clicked()'), self.get_current_joint_angles_cb)
        formlayout.addRow('    ', self.current_pose_button)
        self.reset()

    def get_current_joint_angles_cb(self):
        poses = self.rcommander.robot.head.pose()
        self.joint_boxes[0].setValue(np.degrees(poses[0,0]))
        self.joint_boxes[1].setValue(np.degrees(poses[1,0]))

    def set_node_properties(self, my_node):
        self.joint_boxes[0].setValue(np.degrees(my_node.poses[0,0]))
        self.joint_boxes[1].setValue(np.degrees(my_node.poses[1,0]))

    def reset(self):
        for box in self.joint_boxes:
            box.setValue(0)

    def new_node(self, name=None):
        if name == None:
            nname = self.name + str(self.counter)
        else:
            nname = name

        poses = np.matrix([self.joint_boxes[0], self.joint_boxes[1]]).T
        return MoveHeadState(nname, poses)


class MoveHeadState(tu.StateBase):

    def __init__(self, name, arm, poses):
        tu.StateBase.__init__(name)
        self.poses = poses

    def get_smach_state(self):
        return MoveHeadState(poses)


class MoveHeadStateSmach(smach.State):

    def __init__(self, poses, mot_time):
        smach.State.__init__(self, outcomes=['done'], input_keys=[], output_keys=[])
        self.mot_time = mot_time
        self.poses = poses
        self.robot = None

    def set_robot(self, robot):
        self.head_obj = robot.head

    def execute(self, userdata):
        self.head_obj.set_pose(self.poses, self.mot_time)
        return 'done'











