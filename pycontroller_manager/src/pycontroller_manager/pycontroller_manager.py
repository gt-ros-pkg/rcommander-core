import roslib; roslib.load_manifest('pycontroller_manager')
import rospy
import pr2_mechanism_msgs.srv as pmm

class ControllerManager:

    def __init__(self):
        # LoadController        
        self.load = rospy.ServiceProxy('pr2_controller_manager/load_controller', pmm.LoadController)

        # UnloadController        
        self.unload = rospy.ServiceProxy('pr2_controller_manager/unload_controller', pmm.UnloadController)

        # SwitchController
        self._switch_controller = rospy.ServiceProxy('pr2_controller_manager/switch_controller', pmm.SwitchController)

        self.joint_controllers = {}
        self.cart_controllers = {}
        for arm in ['l', 'r']:
            self.joint_controllers[arm] = arm + '_arm_controller'
            self.cart_controllers[arm] = cart_controller_name = arm + '_cart'

    def switch(self, start_con, stop_con):
        print 'switching to', start_con, 'from', stop_con
        for n in start_con:
            self.load(n)
        resp = self._switch_controller(start_con, stop_con, pmm.SwitchControllerRequest.STRICT)
        #for n in stop_con:
        #    self.unload(n)
        return resp.ok

    def joint_mode(self, arm):
        #get current state
        if arm == 'left' or arm == 'both':
            self.switch([self.joint_controllers['l']], [self.cart_controllers['l']])
        if arm == 'right' or arm == 'both':
            self.switch([self.joint_controllers['r']], [self.cart_controllers['r']])

    def cart_mode(self, arm):
        if arm == 'left' or arm == 'both':
            #print 'switchleft'
            self.switch([self.cart_controllers['l']], [self.joint_controllers['l']])
        if arm == 'right' or arm == 'both':
            #print 'switchright'
            self.switch([self.cart_controllers['r']], [self.joint_controllers['r']])
