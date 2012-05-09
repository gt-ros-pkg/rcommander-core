import tool_utils as tu
from PyQt4.QtGui import *
from PyQt4.QtCore import *
import smach
import rospy
from tf_broadcast_server.srv import BroadcastTransform, GetTransforms, ClearTransforms
#from object_manipulator.convert_functions import mat_to_pose, stamp_pose
import tf_utils as tfu
import tool_utils as tu

class BaseFrameBox(tu.FrameBox):

    def __init__(self, frame_service):
        tu.FrameBox.__init__(self, frame_service)
        self.baseframes = ['/map', '/base_link']

    def create_box(self, pbox):
        tu.ComboBox.create_box(self, pbox)
        possible_frames = self.frames_service().frames
        for f in self.baseframes:
            if f in possible_frames:
                self.combobox.addItem(f)
        self.setEnabled = self.combobox.setEnabled
        return self.combobox


#class PointCloudClickTool(tu.ToolBase):
class FreezeFrameTool(tu.ToolBase):

    def __init__(self, rcommander):
        tu.ToolBase.__init__(self, rcommander, 'freeze_frame', 'Freeze Frame', FreezeFrameState)
        self.default_frame = '/l_gripper_tool_frame'
        self.default_baseframe = '/base_link'
        self.tf_listener = rcommander.tf_listener
        self.frames_service = rospy.ServiceProxy('get_transforms', GetTransforms)
        self.clear_frames_service = rospy.ServiceProxy('clear_all_transforms', ClearTransforms)

    def fill_property_box(self, pbox):
        formlayout = pbox.layout()
        self.base_frame_box = BaseFrameBox(self.frames_service)
        self.frame_box = tu.FrameBox(self.frames_service)

        #self.pose_button = QPushButton(pbox)
        #self.pose_button.setText('Get Point')

        self.clear_frames_button = QPushButton(pbox)
        self.clear_frames_button.setText('Clear Frames')
        self.rcommander.connect(self.clear_frames_button, SIGNAL('clicked()'), self.clear_frames_button_cb)

        formlayout.addRow("&Base Frame", self.base_frame_box.create_box(pbox))
        formlayout.addRow("&Frame", self.frame_box.create_box(pbox))
        formlayout.addRow(self.clear_frames_button)
        
        self.reset()
        pbox.update()

    def clear_frames_button_cb(self):
        self.clear_frames_service()

    def new_node(self, name=None):
        if name == None:
            nname = self.name + str(self.counter)
        else:
            nname = name
        return FreezeFrameState(nname, self.frame_box.text(), self.base_frame_box.text())

    def set_node_properties(self, node):
        self.base_frame_box.set_text(node.base_frame)
        self.frame_box.set_text(node.frame_to_clone)

    def reset(self):
        self.base_frame_box.set_text(self.default_baseframe, create=False)
        self.frame_box.set_text(self.default_frame, create=False)


class FreezeFrameState(tu.StateBase):

    def __init__(self, name, frame_to_clone, base_frame):
        tu.StateBase.__init__(self, name)
        self.frame_to_clone = frame_to_clone
        self.base_frame = base_frame

    def get_smach_state(self):
        return FreezeFrameStateSmach(self.get_name(), self.frame_to_clone, self.base_frame)


class FreezeFrameStateSmach(smach.State):

    def __init__(self, new_frame_name, frame_to_clone, base_frame):
        smach.State.__init__(self, outcomes = ['succeeded'], input_keys = [], output_keys = [])
        self.new_frame_name = new_frame_name
        self.base_frame = base_frame
        self.frame_to_clone = frame_to_clone
        self.broadcast_transform_srv = rospy.ServiceProxy('broadcast_transform', BroadcastTransform)
    
    def set_robot(self, robot):
        self.robot = robot

    def execute(self, userdata):
        base_T_clone = tfu.tf_as_matrix(self.robot.tf_listener.lookupTransform(self.base_frame, self.frame_to_clone, rospy.Time(0)))
        #print 'BASE FRAME', self.base_frame, 'FRAME TO CLONE', self.frame_to_clone
        #print base_T_clone
        pose = tfu.stamp_pose(tfu.mat_to_pose(base_T_clone), self.base_frame)
        self.broadcast_transform_srv(self.new_frame_name, pose)
        return 'succeeded'

