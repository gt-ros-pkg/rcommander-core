import tool_utils as tu
from PyQt4.QtGui import *
from PyQt4.QtCore import *
import numpy as np
import geometry_msgs.msg as geo
import smach
import rospy
import tf.transformations as tr


class PointCloudClickTool(tu.ToolBase):

    def __init__(self, rcommander):
        tu.ToolBase.__init__(self, rcommander, 'retrieve_point3d', 'Point Cloud Select', Point3DState)
        self.default_frame = '/base_link'
        self.tf_listener = rcommander.tf_listener

    def fill_property_box(self, pbox):
        formlayout = pbox.layout()

        self.xline = QLineEdit(pbox)
        self.yline = QLineEdit(pbox)
        self.zline = QLineEdit(pbox)

        self.time_out_box = QDoubleSpinBox(pbox)
        self.time_out_box.setMinimum(1.)
        self.time_out_box.setMaximum(1000.)
        self.time_out_box.setSingleStep(1.)

        self.frame_box = QComboBox(pbox)
        for f in self.tf_listener.getFrameStrings():
            self.frame_box.addItem(f)

        self.pose_button = QPushButton(pbox)
        self.pose_button.setText('Get Point')

        self.wait_check = QCheckBox(pbox)
        self.wait_check.setTristate(False)

        pos_group = QGroupBox('Position', pbox)
        pos_layout = QFormLayout(pos_group)
        pos_group.setLayout(pos_layout)
        pos_layout.addRow("&X", self.xline)
        pos_layout.addRow("&Y", self.yline)
        pos_layout.addRow("&Z", self.zline)
        formlayout.addRow(pos_group)


        formlayout.addRow('&Frame', self.frame_box)
        formlayout.addRow('Wait For Point', self.wait_check)
        formlayout.addRow('Wait Time Out', self.time_out_box)
        formlayout.addRow(self.pose_button)

        self.rcommander.connect(self.pose_button, SIGNAL('clicked()'), self.get_current_pose)
        self.rcommander.connect(self.wait_check, SIGNAL('stateChanged(int)'), self.box_checked)
        self.reset()
        pbox.update()

    def new_node(self, name=None):
        point = [float(str(self.xline.text())), float(str(self.yline.text())), float(str(self.zline.text()))]
        frame = str(self.frame_box.currentText())
        if name == None:
            nname = self.name + str(self.counter)
        else:
            nname = name

        wait_for_msg = self.wait_check.isChecked()
        time_out = self.time_out_box.value()
        return Point3DState(nname, point, #angle, 
                            frame, wait_for_msg=wait_for_msg, 
                            time_out=time_out)

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


    def set_node_properties(self, node):
        self.xline.setText(str(node.point[0]))
        self.yline.setText(str(node.point[1]))
        self.zline.setText(str(node.point[2]))
        self.time_out_box.setValue(node.time_out)
        if node.wait_for_msg:
            self.wait_check.setCheckState(2)
            
        #self.frame_box.setText(node.frame)
        self.frame_box.setCurrentIndex(self.frame_box.findText(str(node.frame)))

    def reset(self):
        self.xline.setText(str(0.))
        self.yline.setText(str(0.))
        self.zline.setText(str(0.))

        self.time_out_box.setValue(60.)
        self.wait_check.setCheckState(False)
        self.frame_box.setCurrentIndex(self.frame_box.findText(self.default_frame))

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
        self.frame_box.setCurrentIndex(self.frame_box.findText(pose_stamped.header.frame_id))

class WaitForMessage:

    def __init__(self, topic, message_type):
        self.subscriber = rospy.Subscriber(topic, message_type, self.message_cb)
        self.msg = None

    def message_cb(self, message):
        self.msg = message

    def get_message(self):
        return self.msg

    def __del__(self):
        self.subscriber.unregister()


class Point3DStateSmach(smach.State):

    ##
    #@param message if self.message is None we will wait for a message, else use provided message
    #@param time_out if we have to wait for a message specify how long to wait before timing out
    def __init__(self, output_variable_name, message = None, time_out = None):
        smach.State.__init__(self, 
                outcomes = ['succeeded', 'preempted', 'timed_out'], 
                input_keys = [], output_keys = [output_variable_name])
        self.time_out = time_out
        self.message = message
        self.output_variable_name = output_variable_name

    def execute(self, userdata):
        point_stamped = self.message

        t_start = rospy.get_time()
        r = rospy.Rate(10)
        waitobj = WaitForMessage('/cloud_click_point', geo.PoseStamped)
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

        exec("userdata.%s = point_stamped" % self.output_variable_name)
        return 'succeeded'


class Point3DState(tu.StateBase):

    def __init__(self, name, point, #angle, 
                frame, wait_for_msg, time_out):
        tu.StateBase.__init__(self, name, outputs={name: geo.PointStamped})
        self.point = point
        self.frame = frame
        self.time_out = time_out
        self.wait_for_msg = wait_for_msg

    def get_smach_state(self):
        if self.wait_for_msg:
            return Point3DStateSmach(self.get_name(), message = None, time_out = self.time_out)
        else:
            ps = geo.PointStamped()
            ps.header.frame_id = self.frame
            ps.header.stamp = rospy.Time.now()
            ps.point.x = self.point[0]
            ps.point.y = self.point[1]
            ps.point.z = self.point[2]

            return Point3DStateSmach(self.get_name(), message = ps, time_out = None)



