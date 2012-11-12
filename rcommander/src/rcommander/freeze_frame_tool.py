##
# Takes a current frame and makes a copy of it, effectively freezing it in time.
#
import tool_utils as tu
from PyQt4.QtGui import *
from PyQt4.QtCore import *
import smach
import rospy
from tf_broadcast_server.srv import BroadcastTransform, GetTransforms, ClearTransforms
import tf_utils as tfu
import tool_utils as tu

## Like a FrameBox but adds the map and base link to list of allowed frames
class BaseFrameBox(tu.FrameBox):

    ## Constructor
    # @param frame_service A ServiceProxy to get_transforms
    def __init__(self, frame_service):
        tu.FrameBox.__init__(self, frame_service)
        self.baseframes = ['/map', '/base_link']

    ## Create ComboBox with extra frames
    # @param pbox properties box
    def create_box(self, pbox):
        tu.ComboBox.create_box(self, pbox)
        possible_frames = self.frames_service().frames
        for f in self.baseframes:
            if f in possible_frames:
                self.combobox.addItem(f)
        self.setEnabled = self.combobox.setEnabled
        return self.combobox


## Takes a current frame and makes a clone of it at the current moment in time.
class FreezeFrameTool(tu.ToolBase):

    ## Inherited constructor
    def __init__(self, rcommander):
        tu.ToolBase.__init__(self, rcommander, 'freeze_frame', 'Freeze Frame', 
                FreezeFrameState)
        self.default_frame = '/l_gripper_tool_frame'
        self.default_baseframe = '/base_link'
        self.tf_listener = rcommander.tf_listener
        self.frames_service = rospy.ServiceProxy('get_transforms', 
                GetTransforms, persistent=True)
        self.clear_frames_service = rospy.ServiceProxy('clear_all_transforms', 
                ClearTransforms, persistent=True)

    ## Inherited
    def fill_property_box(self, pbox):
        formlayout = pbox.layout()
        self.base_frame_box = BaseFrameBox(self.frames_service)
        self.frame_box = tu.FrameBox(self.frames_service)

        self.clear_frames_button = QPushButton(pbox)
        self.clear_frames_button.setText('Clear Frames')
        self.rcommander.connect(self.clear_frames_button, SIGNAL('clicked()'), 
                self.clear_frames_button_cb)

        formlayout.addRow("&Base Frame", self.base_frame_box.create_box(pbox))
        formlayout.addRow("&Frame", self.frame_box.create_box(pbox))
        formlayout.addRow(self.clear_frames_button)
        
        self.reset()
        pbox.update()

    ## Calls clear frame service to clear frames in the system so far.
    def clear_frames_button_cb(self):
        self.clear_frames_service()

    ## Inherited
    def new_node(self, name=None):
        if name == None:
            nname = self.name + str(self.counter)
        else:
            nname = name
        return FreezeFrameState(nname, self.frame_box.text(), self.base_frame_box.text())

    ## Inherited
    def set_node_properties(self, node):
        self.base_frame_box.set_text(node.base_frame)
        self.frame_box.set_text(node.frame_to_clone)

    ## Inherited
    def reset(self):
        self.base_frame_box.set_text(self.default_baseframe, create=False)
        self.frame_box.set_text(self.default_frame, create=False)

## Inherited intermediate state (for saving)
class FreezeFrameState(tu.StateBase):

    def __init__(self, name, frame_to_clone, base_frame):
        tu.StateBase.__init__(self, name)
        self.frame_to_clone = frame_to_clone
        self.base_frame = base_frame

    def get_smach_state(self):
        return FreezeFrameStateSmach(self.get_name(), self.frame_to_clone, self.base_frame)

## Inherited SMACH state
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
        pose = tfu.stamp_pose(tfu.mat_to_pose(base_T_clone), self.base_frame)
        self.broadcast_transform_srv(self.new_frame_name, pose)
        return 'succeeded'

