import tool_utils as tu
from PyQt4.QtGui import *
from PyQt4.QtCore import *
import numpy as np
import geometry_msgs.msg as geo
import smach
import rospy
import tf.transformations as tr
from tf_broadcast_server.srv import BroadcastTransform, GetTransforms, ClearTransforms
import tf_utils as tfu

##
# Allow users to click on a point in point clouds and use it to define the
# translation part of 3D frames.
class PointCloudClickTool(tu.ToolBase):

    ## Inherited
    def __init__(self, rcommander):
        tu.ToolBase.__init__(self, rcommander, 'create_origin', 'Create Origin', Point3DState)
        self.default_frame = '/base_link'
        self.tf_listener = rcommander.tf_listener
        self.frames_service = rospy.ServiceProxy('get_transforms', GetTransforms, persistent=True)
        self.clear_frames_service = rospy.ServiceProxy('clear_all_transforms', ClearTransforms, persistent=True)

    ## Inherited
    def fill_property_box(self, pbox):
        formlayout = pbox.layout()

        self.xline = QLineEdit(pbox)
        self.yline = QLineEdit(pbox)
        self.zline = QLineEdit(pbox)

        self.time_out_box = QDoubleSpinBox(pbox)
        self.time_out_box.setMinimum(1.)
        self.time_out_box.setMaximum(1000.)
        self.time_out_box.setSingleStep(1.)

        self.frame_box = tu.FrameBox(self.frames_service)
        self.orientation_frame_box = tu.FrameBox(self.frames_service)

        self.pose_button = QPushButton(pbox)
        self.pose_button.setText('Get Point')

        self.clear_frames_button = QPushButton(pbox)
        self.clear_frames_button.setText('Clear Frames')

        self.wait_check = QCheckBox(pbox)
        self.wait_check.setTristate(False)

        pos_group = QGroupBox('Position', pbox)
        pos_layout = QFormLayout(pos_group)
        pos_group.setLayout(pos_layout)
        pos_layout.addRow("&Frame", self.frame_box.create_box(pbox))
        pos_layout.addRow("&X", self.xline)
        pos_layout.addRow("&Y", self.yline)
        pos_layout.addRow("&Z", self.zline)
        formlayout.addRow(pos_group)

        ori_group = QGroupBox("Orientation", pbox)
        ori_layout = QFormLayout(ori_group)
        ori_layout.addRow("Frame", self.orientation_frame_box.create_box(pbox))
        formlayout.addRow(ori_group)

        formlayout.addRow('Wait For Point', self.wait_check)
        formlayout.addRow('Wait Time Out', self.time_out_box)
        formlayout.addRow(self.pose_button)
        formlayout.addRow(self.clear_frames_button)

        self.rcommander.connect(self.pose_button, SIGNAL('clicked()'), self.get_current_pose)
        self.rcommander.connect(self.wait_check, SIGNAL('stateChanged(int)'), self.box_checked)
        self.rcommander.connect(self.clear_frames_button, SIGNAL('clicked()'), self.clear_frames_button_cb)
        
        self.reset()
        pbox.update()

    ## Inherited
    def clear_frames_button_cb(self):
        self.clear_frames_service()

    ## Inherited
    def new_node(self, name=None):
        point = [float(str(self.xline.text())), float(str(self.yline.text())), float(str(self.zline.text()))]
        point_frame = self.frame_box.text()
        orientation_frame = self.orientation_frame_box.text()
        if name == None:
            nname = self.name + str(self.counter)
        else:
            nname = name

        wait_for_msg = self.wait_check.isChecked()
        time_out = self.time_out_box.value()
        return Point3DState(nname, point, #angle, 
                            point_frame, orientation_frame, 
                            wait_for_msg=wait_for_msg, time_out=time_out)

    ## Change GUI to respond to wait for point being checked.
    def box_checked(self, state):
        if state == 0:
            self.xline.setEnabled(True)
            self.yline.setEnabled(True)
            self.zline.setEnabled(True)

            self.frame_box.setEnabled(True)
            self.pose_button.setEnabled(True)
            
        if state == 2:
            self.xline.setEnabled(False)
            self.yline.setEnabled(False)
            self.zline.setEnabled(False)

            self.frame_box.setEnabled(False)
            self.pose_button.setEnabled(False)

    ## Inhertied
    def set_node_properties(self, node):
        self.xline.setText(str(node.point[0]))
        self.yline.setText(str(node.point[1]))
        self.zline.setText(str(node.point[2]))
        self.frame_box.set_text(node.point_frame)
        self.orientation_frame_box.set_text(node.orientation_frame)
        self.time_out_box.setValue(node.time_out)
        if node.wait_for_msg:
            self.wait_check.setCheckState(2)

    ## Inhertied
    def reset(self):
        self.xline.setText(str(0.))
        self.yline.setText(str(0.))
        self.zline.setText(str(0.))

        self.time_out_box.setValue(120.)
        self.wait_check.setCheckState(False)
        self.frame_box.set_text(self.default_frame, create=False)
        self.orientation_frame_box.set_text(self.default_frame, create=False)


    #Get the current pose from clicked point cloud, displays it in SE3 boxes
    def get_current_pose(self):
        pose_stamped = rospy.wait_for_message('/cloud_click_point', geo.PoseStamped, 5.)
        self.xline.setText('%.3f' % pose_stamped.pose.position.x)
        self.yline.setText('%.3f' % pose_stamped.pose.position.y)
        self.zline.setText('%.3f' % pose_stamped.pose.position.z)

        q = pose_stamped.pose.orientation
        phi, theta, psi = tr.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.psi_line.setText('%.3f' % phi)
        self.theta_line.setText('%.3f' % theta)
        self.psi_line.setText('%.3f' % psi)
        self.frame_box.set_text(pose_stamped.header.frame_id)
        self.orientation_frame_box.set_text(pose_stamped.header.frame_id)

## Waits for a message to arrive in given topic.
class WaitForMessage:

    ## Constructor
    # @param topic Topic to listen on for message.
    # @param message_type Data type of topic.
    def __init__(self, topic, message_type):
        self.subscriber = rospy.Subscriber(topic, message_type, self.message_cb)
        self.msg = None

    ##
    # Callback for when there's a message
    def message_cb(self, message):
        self.msg = message

    ## Return the message received (none if nothing received so far)
    def get_message(self):
        return self.msg

    ##
    # Unregister after this object disappear.
    def __del__(self):
        self.subscriber.unregister()

## StateBase for Point3DState
class Point3DState(tu.StateBase):

    ## Constructor
    # @param name Name of state (used as frame name).
    # @param point Point used for the center of the frame.
    # @param point_frame Frame that the point is in.
    # @param orientation_frame Since one point doesn't define a frame, we copy
    #                           the orientation from a related frame on the robot.
    # @param wait_for_msg Whether to wait for a point message to come on the
    #                     'cloud_click_point' topic (boolean).  If true,
    #                     ignores the value for point parameter.
    # @param time_out
    def __init__(self, name, point, #angle, 
                point_frame, orientation_frame, wait_for_msg, time_out):
        tu.StateBase.__init__(self, name)
        self.point = point
        self.point_frame = point_frame
        self.orientation_frame = orientation_frame
        self.time_out = time_out
        self.wait_for_msg = wait_for_msg

    def get_smach_state(self):
        if self.wait_for_msg:
            return Point3DStateSmach(self.get_name(), self.orientation_frame, message = None, time_out = self.time_out)
        else:
            ps = geo.PointStamped()
            ps.header.frame_id = self.point_frame
            ps.header.stamp = rospy.Time.now()
            ps.point.x = self.point[0]
            ps.point.y = self.point[1]
            ps.point.z = self.point[2]

            return Point3DStateSmach(self.get_name(), self.orientation_frame, message = ps, time_out = None)


class Point3DStateSmach(smach.State):

    ## Constructor
    # @param frame_name Name of frame to create.
    # @param orientation_frame Name of frame to copy orientation information from.
    # @param message If self.message is None we will wait for a message, else use provided message.
    # @param time_out If we have to wait for a message specify how long to wait before timing out
    def __init__(self, frame_name, orientation_frame, message = None, time_out = None):
        smach.State.__init__(self, 
                outcomes = ['succeeded', 'preempted', 'timed_out'], 
                input_keys = [], output_keys = [])

        self.frame_name = frame_name
        self.orientation_frame = orientation_frame
        self.time_out = time_out
        self.message = message
        self.broadcast_transform_srv = rospy.ServiceProxy('broadcast_transform', BroadcastTransform)

    ## Inherited
    def set_robot(self, robot):
        self.robot = robot

    def execute(self, userdata):
        #Listen for a point if we don't have one.  
        point_stamped = self.message
        t_start = rospy.get_time()
        r = rospy.Rate(10)
        waitobj = WaitForMessage('/cloud_click_point', geo.PoseStamped)
        pose_stamped = None
        while point_stamped == None:
            try:
                pose_stamped = waitobj.get_message()
                if pose_stamped != None:
                    point_stamped = geo.PointStamped()
                    point_stamped.header = pose_stamped.header
                    point_stamped.point = pose_stamped.pose.position
                
                r.sleep()
            except rospy.ROSException, e:
                pass

            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'

            if (self.time_out != None) and ((rospy.get_time() - t_start) > self.time_out):
                return 'timed_out'
        del waitobj

        # The given point transforms from this NEW FRAME (self.frame_name) 
        # to original parent frame (header.frame_id).
        frame_id_T_frame_name = tfu.origin_to_frame(point_stamped, self.orientation_frame, 
                self.robot.tf_listener, point_stamped.header.frame_id)
        pose = tfu.stamp_pose(tfu.mat_to_pose(frame_id_T_frame_name), point_stamped.header.frame_id)
        self.broadcast_transform_srv(self.frame_name, pose)

        return 'succeeded'




