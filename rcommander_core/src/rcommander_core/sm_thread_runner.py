import roslib; roslib.load_manifest('rcommander_core')
import threading
import rospy
import smach
import smach_ros
import ctypes
import time

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
        #self.intro_server = None
        self.exception = None
        self.termination_func = None

    def register_termination_cb(self, func):
        self.termination_func = func

    def run(self):
        rospy.loginfo('ThreadRunSM started %s' % self.sm_name)
        try:
            #self.intro_server = smach_ros.IntrospectionServer(self.sm_name, self.sm, '/' + self.sm_name)
            #self.intro_server.start()
            self.outcome = self.sm.execute()
            rospy.loginfo('ThreadRunSM.run: execution finished outcome %s' % self.outcome)

        except smach.InvalidTransitionError, e:
            rospy.loginfo('ThreadRunSM: got InvalidTransitionError %s' % str(e))
            self.exception = e

        except UserStoppedException, e:
            #self.intro_server.stop()
            self.exception = e
            rospy.loginfo('ThreadRunSM: execution stopped by user.')

        if self.termination_func != None:
            self.termination_func(self.exception)

        #self.intro_server.stop()
        rospy.loginfo('ThreadRunSM.run: exiting')

    def preempt(self):
        if self.isAlive():
            self.sm.request_preempt()

    def except_preempt(self):
        while self.isAlive():
            self._raise_exception()
            time.sleep(2.)
            if self.isAlive():
                threading.Thread._Thread__stop(self)

    def _raise_exception(self):
        #res = ctypes.pythonapi.PyThreadState_SetAsyncExc(ctypes.c_long(self.ident), ctypes.py_object(UserStoppedException))
        res = ctypes.pythonapi.PyThreadState_SetAsyncExc(ctypes.c_long(self.ident), ctypes.py_object(SystemExit))
        print 'raised exception returned', res
        if res == 0:
            raise ValueError("Invalid thread ID")
        elif res != 1:
            # "if it returns a number greater than one, you're in trouble,
            # and you should call it again with exc=NULL to revert the effect"
            ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, 0)
            raise SystemError("PyThreadState_SetAsyncExc failed")
