import roslib; roslib.load_manifest('rcommander')
import threading
import rospy
import smach
import smach_ros

class UserStoppedException:
    def __init__(self):
        self.msg = "Stopped"

    def __str__(self):
        return repr(self.msg)

class ThreadRunSM(threading.Thread):

    def __init__(self, sm_name, sm):
        threading.Thread.__init__(self)    
        self.sm = sm
        self.sm_name = sm_name
        self.outcome = None
        self.intro_server = None
        self.exception = None

    def run(self):
        rospy.loginfo('ThreadRunSM started with %s' % self.sm_name)
        try:
            self.intro_server = smach_ros.IntrospectionServer(self.sm_name, self.sm, '/' + self.sm_name)
            self.intro_server.start()
            self.outcome = self.sm.execute()
        except smach.InvalidTransitionError, e:
            self.exception = e
        except UserStoppedException, e:
            self.exception = e
            rospy.loginfo('ThreadRunSM: execution stopped')
        rospy.loginfo('ThreadRunSM finished')

    def except_stop(self):
        while self.isAlive():
            self._raise_exception()
            time.sleep(.1)

    def _raise_exception(self):
        res = ctypes.pythonapi.PyThreadState_SetAsyncExc(ctypes.c_long(self.ident), ctypes.py_object(UserStoppedException))
        if res == 0:
            raise ValueError("Invalid thread ID")
        elif res != 1:
            # "if it returns a number greater than one, you're in trouble,
            # and you should call it again with exc=NULL to revert the effect"
            ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, 0)
            raise SystemError("PyThreadState_SetAsyncExc failed")
