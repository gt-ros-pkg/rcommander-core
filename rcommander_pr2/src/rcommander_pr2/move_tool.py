import roslib; roslib.load_manifest('rcommander_pr2')
import rospy
from PyQt4.QtGui import *
from PyQt4.QtCore import *
import rcommander_core.tool_utils as tu
import numpy as np
import actionlib_msgs.msg as am
import smach
from pycontroller_manager.pycontroller_manager import ControllerManager


class JointSequenceTool(tu.ToolBase):

    def __init__(self, rcommander):
        tu.ToolBase.__init__(self, rcommander, 'joint_sequence', 'Joint Sequence', JointSequenceState)
        self.joint_name_fields = ["shoulder_pan_joint", "shoulder_lift_joint", "upper_arm_roll_joint", 
                                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        self.joint_angs_list = None

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

        self.time_box = QDoubleSpinBox(pbox)
        self.time_box.setMinimum(0)
        self.time_box.setMaximum(1000.)
        self.time_box.setSingleStep(.2)
        self.time_box.setValue(1.)
        formlayout.addRow('&Time', self.time_box)
   
        #Controls for getting the current joint states
        self.joint_angs_list = []

        self.list_box = QWidget(pbox)
        self.list_box_layout = QHBoxLayout(self.list_box)
        self.list_box_layout.setMargin(0)

        self.list_widget = QListWidget(self.list_box)
        self.rcommander.connect(self.list_widget, SIGNAL('itemSelectionChanged()'), self.item_selection_changed_cb)

        self.movement_buttons_widget = QWidget(self.list_box)
        self.movement_buttons_widgetl = QVBoxLayout(self.movement_buttons_widget)
        self.movement_buttons_widgetl.setMargin(0)

        self.move_up_button = QPushButton(self.movement_buttons_widget)
        self.move_up_button.setText('Up')
        self.move_down_button = QPushButton(self.movement_buttons_widget)
        self.move_down_button.setText('Down')

        self.rcommander.connect(self.move_up_button, SIGNAL('clicked()'), self.move_up_cb)
        self.rcommander.connect(self.move_down_button, SIGNAL('clicked()'), self.move_down_cb)
        self.movement_buttons_widgetl.addWidget(self.move_up_button)
        self.movement_buttons_widgetl.addWidget(self.move_down_button)

        self.list_widget_buttons = QWidget(pbox)
        self.lbb_hlayout = QHBoxLayout(self.list_widget_buttons)

        self.add_joint_set_button = QPushButton(self.list_widget_buttons)
        self.add_joint_set_button.setText('Add')
        self.rcommander.connect(self.add_joint_set_button, SIGNAL('clicked()'), self.add_joint_set_cb)

        self.remove_joint_set_button = QPushButton(self.list_widget_buttons)
        self.remove_joint_set_button.setText('Remove')
        self.rcommander.connect(self.remove_joint_set_button, SIGNAL('clicked()'), self.remove_pose_cb)

        self.pose_button = QPushButton(self.list_widget_buttons)
        self.pose_button.setText('Pose')
        self.rcommander.connect(self.pose_button, SIGNAL('clicked()'), self.get_current_joint_angles)

        self.lbb_hlayout.addWidget(self.add_joint_set_button)
        self.lbb_hlayout.addWidget(self.remove_joint_set_button)
        self.lbb_hlayout.addWidget(self.pose_button)
        self.lbb_hlayout.setContentsMargins(2, 2, 2, 2)

        self.list_box_layout.addWidget(self.list_widget)
        self.list_box_layout.addWidget(self.movement_buttons_widget)

        #formlayout.addRow(self.list_widget)
        formlayout.addRow(self.list_box)
        formlayout.addRow(self.list_widget_buttons)
        self.reset()

    def _refill_list_widget(self, joints_list):
        self.list_widget.clear()
        for d in joints_list:
            self.list_widget.addItem(d['name'])

    def get_current_joint_angles(self):
        if ('left' == str(self.arm_box.currentText())):
            arm_obj = self.rcommander.robot.left
        else:
            arm_obj = self.rcommander.robot.right

        pose_mat = arm_obj.pose()
        for idx, name in enumerate(self.joint_name_fields):
            deg = np.degrees(pose_mat[idx, 0])
            exec('line_edit = self.%s' % name)
            line_edit.setText('%.2f' % deg)

    def _has_name(self, test_name):
        for rec in self.joint_angs_list:
            if rec['name'] == test_name:
                return True
            else:
                return False

    def _create_name(self):
        idx = len(self.joint_angs_list)
        tentative_name = 'point%d' % idx 

        while self._has_name(tentative_name):
            idx = idx + 1
            tentative_name = 'point%d' % idx 

        return tentative_name

    def item_selection_changed_cb(self):
        selected = self.list_widget.selectedItems()
        if len(selected) == 0:
            return
        idx = self._find_index_of(str(selected[0].text()))
        joint_angs = self.joint_angs_list[idx]['angs']
        self._set_joints_to_fields(joint_angs)
        self.time_box.setValue(self.joint_angs_list[idx]['time'])

    def add_joint_set_cb(self):
        #Create a new string, check to see whether it's in the current list
        name = self._create_name()
        #self.list_widget.addItem(name)
        self.joint_angs_list.append({'name':name, 'time': self.time_box.value(), 'angs': self._read_joints_from_fields()})
        self._refill_list_widget(self.joint_angs_list)

    def _find_index_of(self, name):
        for idx, tup in enumerate(self.joint_angs_list):
            if tup['name'] == name:
                return idx
        return None

    def move_up_cb(self):
        #get the current index
        idx = self._selected_idx()
        if idx == None:
            return

        #pop & insert it
        item = self.joint_angs_list.pop(idx)
        self.joint_angs_list.insert(idx-1, item)

        #refresh
        self._refill_list_widget(self.joint_angs_list)
        self.list_widget.setCurrentItem(self.list_widget.item(idx-1))

    def move_down_cb(self):
        #get the current index
        idx = self._selected_idx()
        if idx == None:
            return

        #pop & insert it
        item = self.joint_angs_list.pop(idx)
        self.joint_angs_list.insert(idx+1, item)

        #refresh
        self._refill_list_widget(self.joint_angs_list)
        self.list_widget.setCurrentItem(self.list_widget.item(idx+1))

    def _selected_idx(self):
        #Get currently selected
        selected = self.list_widget.selectedItems()
        if len(selected) == 0:
            return None
        sname = str(selected[0].text())

        #Remove it from list_widget and joint_angs_list
        idx = self._find_index_of(sname)
        return idx


    def remove_pose_cb(self):
        idx = self._selected_idx()
        if idx == None:
            return
        #self.list_widget.takeItem(idx)
        #self.list_widget.removeItemWidget(selected[0])
        if idx == None:
            raise RuntimeError('Inconsistency detected in list')
        else:
            self.joint_angs_list.pop(idx)
        self._refill_list_widget(self.joint_angs_list)

    def _read_joints_from_fields(self):
        joints = []
        for name in self.joint_name_fields:
            exec('rad = np.radians(float(str(self.%s.text())))' % name)
            joints.append(rad)
        return joints

    def _set_joints_to_fields(self, joints):
        for idx, name in enumerate(self.joint_name_fields):
            deg = np.degrees(joints[idx])
            exec('line_edit = self.%s' % name)
            line_edit.setText(str(deg))

    def new_node(self, name=None):
        if name == None:
            nname = self.name + str(self.counter)
        else:
            nname = name
    
        #sstate = JointSequenceState(nname, str(self.arm_box.currentText()), self._read_joints_from_fields())
        sstate = JointSequenceState(nname, str(self.arm_box.currentText()), self.joint_angs_list)
        sstate.set_robot(self.rcommander.robot)
        return sstate

    def set_node_properties(self, my_node):
        self.joint_angs_list = my_node.joint_waypoints
        #for d in self.joint_angs_list:
        #    self.list_widget.addItem(d['name'])
        self._refill_list_widget(self.joint_angs_list)
        self.arm_box.setCurrentIndex(self.arm_box.findText(my_node.arm))
        self.list_widget.setCurrentItem(self.list_widget.item(0))

    def reset(self):
        self.arm_box.setCurrentIndex(self.arm_box.findText('left'))
        for name in self.joint_name_fields:
            exec('self.%s.setText(str(0.))' % name)


class JointSequenceState(tu.StateBase): 

    def __init__(self, name, arm, joint_waypoints):
        tu.StateBase.__init__(self, name)
        self.arm = arm
        self.joint_waypoints = joint_waypoints
        self.arm_obj = None #FIX ME

    def set_robot(self, pr2):
        if self.arm == 'left':
            self.arm_obj = pr2.left

        if self.arm == 'right':
            self.arm_obj = pr2.right

    def get_smach_state(self):
        return JointSequenceStateSmach(self.arm, self.joint_waypoints, self.arm_obj)


class JointSequenceStateSmach(smach.State): 

    TIME_OUT_FACTOR = 3.

    def __init__(self, arm, joint_waypoints, arm_obj):
        smach.State.__init__(self, outcomes = ['succeeded', 'preempted', 'failed'], 
                             input_keys = [], output_keys = [])

        self.arm = arm
        self.controller_manager = ControllerManager()
        self.joint_waypoints = joint_waypoints
        self.arm_obj = arm_obj

    def execute(self, userdata):
        self.controller_manager.joint_mode(self.arm)

        #Construct trajectory command
        times = []
        wps = []
        for d in self.joint_waypoints:
            wps.append(np.matrix(d['angs']).T)
            times.append(d['time'])

        self.arm_obj.set_poses(np.column_stack(wps), np.cumsum(np.array(times)), block=False)
        client = self.arm_obj.client
        state = client.get_state()

        #Monitor execution
        trajectory_time_out = JointSequenceStateSmach.TIME_OUT_FACTOR * np.sum(times)
        succeeded = False
        preempted = False
        r = rospy.Rate(30)
        start_time = rospy.get_time()
        while True:
            #we have been preempted
            if self.preempt_requested():
                rospy.loginfo('JointSequenceState: preempt requested')
                client.cancel_goal()
                self.service_preempt()
                preempted = True
                break

            if (rospy.get_time() - start_time) > trajectory_time_out:
                client.cancel_goal()
                rospy.loginfo('JointSequenceState: timed out!')
                succeeded = False
                break

            #print tu.goal_status_to_string(state)
            if (state not in [am.GoalStatus.ACTIVE, am.GoalStatus.PENDING]):
                if state == am.GoalStatus.SUCCEEDED:
                    rospy.loginfo('JointSequenceState: Succeeded!')
                    succeeded = True
                break

            state = client.get_state()

            r.sleep()

        if preempted:
            return 'preempted'

        if succeeded:
            return 'succeeded'

        return 'failed'

#    def __init_unpicklables__(self):
#
#class JointSequenceState(smach.State, tu.StateBase): 
#
#    TIME_OUT_FACTOR = 3.
#
#    def __init__(self, name, arm, joint_waypoints):
#        self.name = name
#        tu.StateBase.__init__(self, self.name)
#        self.arm = arm
#        self.joint_waypoints = joint_waypoints
#        self.arm_obj = None
#        self.__init_unpicklables__()
#
#    def execute(self, userdata):
#        self.controller_manager.joint_mode(self.arm)
#
#        #Construct trajectory command
#        times = []
#        wps = []
#        for d in self.joint_waypoints:
#            wps.append(np.matrix(d['angs']).T)
#            times.append(d['time'])
#
#        self.arm_obj.set_poses(np.column_stack(wps), np.cumsum(np.array(times)), block=False)
#        client = self.arm_obj.client
#        state = client.get_state()
#
#        #Monitor execution
#        trajectory_time_out = JointSequenceState.TIME_OUT_FACTOR * np.sum(times)
#        succeeded = False
#        preempted = False
#        r = rospy.Rate(30)
#        start_time = rospy.get_time()
#        while True:
#            #we have been preempted
#            if self.preempt_requested():
#                rospy.loginfo('JointSequenceState: preempt requested')
#                client.cancel_goal()
#                self.service_preempt()
#                preempted = True
#                break
#
#            if (rospy.get_time() - start_time) > trajectory_time_out:
#                client.cancel_goal()
#                rospy.loginfo('JointSequenceState: timed out!')
#                succeeded = False
#                break
#
#            #print tu.goal_status_to_string(state)
#            if (state not in [am.GoalStatus.ACTIVE, am.GoalStatus.PENDING]):
#                if state == am.GoalStatus.SUCCEEDED:
#                    rospy.loginfo('JointSequenceState: Succeeded!')
#                    succeeded = True
#                break
#
#            state = client.get_state()
#
#            r.sleep()
#
#        if preempted:
#            return 'preempted'
#
#        if succeeded:
#            return 'succeeded'
#
#        return 'failed'
#
#    def set_robot(self, pr2):
#        if self.arm == 'left':
#            self.arm_obj = pr2.left
#
#        if self.arm == 'right':
#            self.arm_obj = pr2.right
#
#    def __init_unpicklables__(self):
#        smach.State.__init__(self, outcomes = ['succeeded', 'preempted', 'failed'], input_keys = [], output_keys = [])
#        self.controller_manager = ControllerManager()
#
#    def __getstate__(self):
#        state = tu.StateBase.__getstate__(self)
#        my_state = [self.name, self.arm, self.joint_waypoints] 
#        return {'state_base': state, 'self': my_state}
#
#    def __setstate__(self, state):
#        tu.StateBase.__setstate__(self, state['state_base'])
#        self.name, self.arm, self.joint_waypoints = state['self']
#        self.__init_unpicklables__()


