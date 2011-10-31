import rospy
from PyQt4.QtGui import *
from PyQt4.QtCore import *
import rcommander_core.tool_utils as tu
import numpy as np
import smach

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
            formlayout.addRow('&%s' % jname, joint_box)

        self.time_box = QDoubleSpinBox(pbox)
        self.time_box.setMinimum(0)
        self.time_box.setMaximum(1000.)
        self.time_box.setSingleStep(.5)
        formlayout.addRow('&Time', self.time_box)

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
        self.time_box.setValue(my_node.mot_time)

    def reset(self):
        for box in self.joint_boxes:
            box.setValue(0)
        self.time_box.setValue(1.)

    def new_node(self, name=None):
        if name == None:
            nname = self.name + str(self.counter)
        else:
            nname = name

        poses = np.matrix([np.radians(self.joint_boxes[0].value()),
			   np.radians(self.joint_boxes[1].value())]).T
        return MoveHeadState(nname, poses, self.time_box.value())


class MoveHeadState(tu.StateBase):

    def __init__(self, name, poses, mot_time):
        tu.StateBase.__init__(self, name)
        self.poses = poses 
        self.mot_time = mot_time

    def get_smach_state(self):
        return MoveHeadStateSmach(self.poses, self.mot_time)


class MoveHeadStateSmach(smach.State):

    def __init__(self, poses, mot_time):
        smach.State.__init__(self, outcomes=['preempted', 'done'], 
			     input_keys=[], output_keys=[])
        self.mot_time = mot_time
        self.poses = poses
        self.robot = None

    def set_robot(self, robot):
        self.head_obj = robot.head

    def execute(self, userdata):
        self.head_obj.set_pose(self.poses, self.mot_time)

        r = rospy.Rate(30)
        start_time = rospy.get_time()
        while (rospy.get_time() - start_time) < self.mot_time:
            r.sleep()
            if self.preempt_requested():
                self.services_preempt()
                return 'preempted'
        return 'done'











