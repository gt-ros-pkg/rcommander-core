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

        self.list_controllers = rospy.ServiceProxy('pr2_controller_manager/list_controllers', pmm.ListControllers)

        self.joint_controllers = {}
        self.cart_controllers = {}
        for arm in ['l', 'r']:
            self.joint_controllers[arm] = arm + '_arm_controller'
            self.cart_controllers[arm] = cart_controller_name = arm + '_cart'

    def switch(self, start_con, stop_con):
        con = self.list_controllers()
        valid_start = []
        valid_stop = []

        #Add the ones that are loaded but not running.
        for idx, controller in enumerate(con.controllers):
            if controller in start_con:
                #print 'found', controller, 'it\'s state is', con.state[idx], len(con.state[idx]), con.state[idx].__class__
                if con.state[idx] != 'running':
                    #print 'ControllerManager: adding controller', controller
                    valid_start.append(controller)
                #else:
                #    print 'ControllerManager: not adding', controller

            if controller in stop_con:
                if con.state[idx] != 'stopped':
                    valid_stop.append(controller)

            #print controller, con.state[idx]

        #Add all controllers not loaded! But load them first
        for n in start_con:
            if not n in con.controllers:
                #print 'loading controller', n
                self.load(n)
                valid_start.append(n)

        if len(start_con) > 0:
            #print 'ControllerManager: starting', valid_start, 'stopping', valid_stop
            resp = self._switch_controller(valid_start, valid_stop, pmm.SwitchControllerRequest.STRICT)
            return resp.ok
        else:
            #print 'ControllerManager: not starting or stopping any controllsers'
            return None

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
