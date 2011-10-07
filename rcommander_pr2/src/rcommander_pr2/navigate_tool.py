import roslib; roslib.load_manifest('rcommander_pr2')
import rcommander_core.tool_utils as tu
#import smach_ros
from PyQt4.QtGui import *
from PyQt4.QtCore import *
import rospy
import tf_utils as tfu
import tf.transformations as tr
import move_base_msgs.msg as mm
import math


#
# controller and view
# create and edits smach states
class NavigateTool(tu.ToolBase):

    def __init__(self, rcommander):
        tu.ToolBase.__init__(self, rcommander, 'navigate', 'Navigate', NavigateState)
        self.tf_listener = rcommander.tf_listener
        self.default_frame = 'map'
        self.robot_frame_name = 'base_link'

    def fill_property_box(self, pbox):
        formlayout = pbox.layout()
        self.xline = QLineEdit(pbox)
        self.yline = QLineEdit(pbox)
        self.tline = QLineEdit(pbox)
        self.frameline = QLineEdit(pbox)
        self.frameline.setText(self.default_frame)
        self.pose_button = QPushButton(pbox)
        self.pose_button.setText('Current Pose')
        self.pose_tool = QPushButton(pbox)
        self.pose_tool.setText('Specify Pose')
        self.reset()

        formlayout.addRow("&x", self.xline)
        formlayout.addRow("&y", self.yline)
        formlayout.addRow("&theta", self.tline)
        formlayout.addRow("&frame", self.frameline)
        formlayout.addRow(self.pose_button)
        self.rcommander.connect(self.pose_button, SIGNAL('clicked()'), self.get_current_pose)
        formlayout.addRow(self.pose_tool)
        self.pose_tool.setEnabled(False)
        #self.rcommander.connect(self.pose_tool, SIGNAL('clicked()'), self.
        pbox.update()
        #print 'outcomes!', self.create_node()._outcomes

    def get_current_pose(self):
        frame = str(self.frameline.text())
        self.tf_listener.waitForTransform(frame, self.robot_frame_name, rospy.Time(), rospy.Duration(2.))
        p_base = tfu.transform(frame, self.robot_frame_name, self.tf_listener) \
                    * tfu.tf_as_matrix(([0., 0., 0., 1.], tr.quaternion_from_euler(0,0,0)))
        t, r = tfu.matrix_as_tf(p_base)
        x = t[0]
        y = t[1]
        theta = tr.euler_from_quaternion(r)[2]
        print x,y,theta
        
        self.xline.setText(str(x))
        self.yline.setText(str(y))
        self.tline.setText(str(math.degrees(theta)))

    def new_node(self, name=None):
        xy = [float(self.xline.text()), float(self.yline.text())]
        theta = math.radians(float(self.tline.text()))
        frame = str(self.frameline.text())
        if name == None:
            nname = self.name + str(self.counter)
        else:
            nname = name
        state = NavigateState(nname, xy, theta, frame)
        return state

    def set_node_properties(self, node):
        xy = node.xy
        self.xline.setText(str(xy[0]))
        self.yline.setText(str(xy[1]))
        self.tline.setText(str(math.degrees(node.theta)))
        self.frameline.setText(node.frame)
        #print 'node_selected called', xy

    def reset(self):
        self.xline.setText('0.')
        self.yline.setText('0.')
        self.tline.setText('0.')
        self.frameline.setText(self.default_frame)

#
# name maps to tool used to create it
# model
# is a state that can be stuffed into a state machine
class NavigateState(tu.SimpleStateBase): # smach_ros.SimpleActionState):

    def __init__(self, name, xy, theta, frame):
        tu.SimpleStateBase.__init__(self, name, \
                'move_base', mm.MoveBaseAction, 
                goal_cb_str = 'ros_goal') 

        self.xy = xy
        self.theta = theta #stored as r internally
        self.frame = frame

    def ros_goal(self, userdata, default_goal):
        g = mm.MoveBaseGoal()
        p = g.target_pose
        
        p.header.frame_id = 'map'
        p.header.stamp = rospy.get_rostime()
        p.pose.position.x = self.xy[0]
        p.pose.position.y = self.xy[1]
        p.pose.position.z = 0
        
        p.pose.orientation.x = self.r[0]
        p.pose.orientation.y = self.r[1]
        p.pose.orientation.z = self.r[2]
        p.pose.orientation.w = self.r[3]
        return g

    def _get_theta(self):
        return tr.euler_from_quaternion(self.r)[2]

    def _set_theta(self, theta):
        self.r = tr.quaternion_from_euler(0, 0, theta)

    #    def _get_xy(self):
    #        return [self.xy[0], self.xy[1]]
    #
    #    def _set_xy(self, xy):
    #        self.xy = xy
    #        #self.t = [xy[0], xy[1], 0]
    #
    theta = property(_get_theta, _set_theta)

    #xy = property(_get_xy, _set_xy)

    def __getstate__(self):
        state = tu.SimpleStateBase.__getstate__(self)
        my_state = [self.xy, self.r, self.frame]
        return {'simple_state': state, 'self': my_state}

    def __setstate__(self, state):
        #print 'NavigateState setting state', state
        tu.SimpleStateBase.__setstate__(self, state['simple_state'])
        xy, r, fr = state['self']
        self.xy = xy
        self.r = r
        self.frame = fr
        #NavigateState.__init__(self, name, xy, t, fr)



