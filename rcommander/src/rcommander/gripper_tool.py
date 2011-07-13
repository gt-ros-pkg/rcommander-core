import tool_utils as tu
from PyQt4.QtGui import *
from PyQt4.QtCore import *
import time
import pr2_common_action_msgs.msg as ca 
import functools as ft
import pr2_controllers_msgs.msg as pm

#class SliderBox:
#
#    def __init__(self, parent, initial_value, max_value, min_value, ticks, name_preffix, units=''):
#        self.min_value = min_value
#        self.max_value = max_value
#
#        container_name = name_preffix + '_box'
#        disp_name = name_preffix + '_disp'
#        slider_name = name_preffix + '_slider'
#
#        nticks = float(max_value - min_value) / float(ticks)
#        tick_size = int(round(100. / nticks))
#        tick_pos = self._units_to_slider(initial_value)
#
#        container = QWidget(parent)
#        container.setObjectName(container_name)
#        
#        disp = QLabel(container)
#        disp.setObjectName(disp_name)
#        disp.setText('%.2f %s' % (initial_value, units))
#
#        slider = QSlider(container)
#        slider.setSingleStep(1)
#        slider.setOrientation(Qt.Horizontal)
#        slider.setTickPosition(QSlider.TicksBelow)
#        slider.setTickInterval(tick_size)
#        slider.setObjectName(slider_name)
#        slider.setSliderPosition(tick_size)
#        slider.setValue(tick_pos)
#
#        hlayout = QHBoxLayout(container)
#        hlayout.addWidget(disp)
#        hlayout.addWidget(slider)
#
#        def slider_moved_cb(disp, value):
#            cv = self._slider_to_units(value)
#            disp.setText('%3.2f %s' % (cv, units))
#        parent.connect(slider, SIGNAL('sliderMoved(int)'), ft.partial(slider_moved_cb, disp))
#
#        self.container = container
#        self.slider = slider
#
#    def _slider_to_units(self, value):
#        return ((value / 100.) * (self.max_value - self.min_value)) + self.min_value
#        
#    def _units_to_slider(self, value):
#        return int(round(((value - self.min_value) / (self.max_value - self.min_value)) * 100.0))
#
#    def value(self):
#        return self._slider_to_units(self.slider.value())
#
#    def set_value(self, value):
#        self.slider.setValue(self._units_to_slider(value))


#def make_radio_box(parent, options, name_preffix):
#    container_name = name_preffix + '_radio_box'
#
#    container = QWidget(parent)
#    container.setObjectName(container_name)
#    hlayout = QHBoxLayout(container)
#    radio_buttons = []
#
#    for option in options:
#        r = QRadioButton(container)
#        r.setObjectName(name_preffix + '_' + option)
#        r.setText(option)
#        hlayout.addWidget(r)
#        radio_buttons.append(r)
#    radio_buttons[0].setChecked(True)
#
#    return container, radio_buttons


class GripperTool(tu.ToolBase):

    def __init__(self, rcommander):
        tu.ToolBase.__init__(self, rcommander, 'gripper', 'Gripper')

    def fill_property_box(self, pbox):
        formlayout = pbox.layout()
        #Left or right
        self.radio_boxes, self.radio_buttons = tu.make_radio_box(pbox, ['Left', 'Right'], 'gripper_arm')
        #Opening distance
        self.gripper_box = tu.SliderBox(pbox, 0.01, 8.4, 0.02, .01, 'gripper', units='cm')
        #Effort
        self.effort_box = tu.SliderBox(pbox, 50., 200., 0., 40., 'effort', units='')

        formlayout.addRow('&Side', self.radio_boxes)
        formlayout.addRow('&Gripper Opening', self.gripper_box.container)
        formlayout.addRow('&Effort', self.effort_box.container)
        pbox.update()

    def _create_node(self, name=None):
        selected_arm = None
        for r in self.radio_buttons:
            if r.isChecked():
                selected_arm = str(r.text()).lower()
        if selected_arm == None:
            raise RuntimeError('No arm selected!')

        gsize = self.gripper_box.value()
        effort = self.effort_box.value()

        if name == None:
            nname = self.name + str(self.counter)
        else:
            nname = name
        return GripperState(nname, selected_arm, gsize, effort)
    
    def _node_selected(self, gripper_state):
        if gripper_state.arm == 'left':
            self.radio_buttons[0].setChecked(True)
        if gripper_state.arm == 'right':
            self.radio_buttons[1].setChecked(True)

        self.gripper_box.set_value(gripper_state.gripper_size)
        self.effort_box.set_value(gripper_state.effort)

    def reset(self):
        self.gripper_box.set_value(0.0)
        self.effort_box.set_value(50.)
        self.radio_buttons[0].setChecked(True)


class GripperState(tu.SimpleStateBase): # smach_ros.SimpleActionState):

    def __init__(self, name, arm, gripper_size, effort):
        if arm == 'left':
            action = 'l_gripper_controller/gripper_action'
        elif arm == 'right':
            action = 'r_gripper_controller/gripper_action'

        tu.SimpleStateBase.__init__(self, name, \
                action, pm.Pr2GripperCommandAction,
                goal_cb_str = 'ros_goal')

        self.gripper_size = gripper_size
        self.effort = effort
        self.arm = arm

    def ros_goal(self, userdata, default_goal):
        #import time
        #while True:
        #    time.sleep(.01)
        return pm.Pr2GripperCommandGoal(pm.Pr2GripperCommand(position=self.gripper_size/100., max_effort=self.effort))

    def __getstate__(self):
        state = tu.SimpleStateBase.__getstate__(self)
        my_state = [self.gripper_size, self.effort, self.arm]
        return {'simple_state': state, 'self': my_state}

    def __setstate__(self, state):
        tu.SimpleStateBase.__setstate__(self, state['simple_state'])
        self.gripper_size, self.effort, self.arm = state['self']

