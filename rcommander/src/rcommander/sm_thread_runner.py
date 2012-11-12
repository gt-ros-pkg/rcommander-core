import threading
import rospy
import smach
import smach_ros
import ctypes
import time
import sys
import tool_utils as tu


## Runs given state machine in a separate thread.  Clients register a callback
# to be notified when execution completes.  The outcome and exceptions can be
# checked in self.outcome and self.exception.
class ThreadRunSM(threading.Thread):

    ## Constructor
    # @param sm_name Name of state machine being run.
    # @param sm SMACH state machine to run.
    def __init__(self, sm_name, sm):
        threading.Thread.__init__(self)    
        self.sm = sm
        self.sm_name = sm_name
        self.outcome = None
        self.exception = None
        self.termination_func = None

    ## Registers a callback to call once execution finishes.
    def register_termination_cb(self, func):
        self.termination_func = func

    ## Inherited from Thread
    def run(self):
        rospy.loginfo('ThreadRunSM started %s' % self.sm_name)
        try:
            self.outcome = self.sm.execute()
            rospy.loginfo('ThreadRunSM.run: execution ' + \
                    'finished outcome %s' % self.outcome)

        except smach.InvalidTransitionError, e:
            rospy.loginfo('ThreadRunSM: got InvalidTransitionError %s' % str(e))
            self.exception = e

        except smach.InvalidUserCodeError, e:
            self.exception = e
            rospy.loginfo('ThreadRunSM: invalid user code error.')

        except:
            self.exception = Exception(str(sys.exc_info()[0]))
            rospy.loginfo('ThreadRunSM: unknown ' + \
                    'exception %s sdf.' % (str(self.exception)))

        if self.termination_func != None:
            if self.outcome == None:
                try:
                    self.sm.check_consistency()
                except smach.InvalidStateError, e:
                    self.exception = e
                except smach.InvalidTransitionError, e:
                    self.exception = e
            self.termination_func(self.exception)

        #self.intro_server.stop()
        rospy.loginfo('ThreadRunSM.run: exiting')

    ## Preempt execution of state machine to stop it.
    def preempt(self):
        if self.isAlive():
            self.sm.request_preempt()

    ## Last resort way to get state machine to stop.  
    # Raises an exception in the executing thread to stop it.
    def except_preempt(self):
        while self.isAlive():
            self._raise_exception()
            time.sleep(2.)
            if self.isAlive():
                threading.Thread._Thread__stop(self)

    ## Raises an exception in executing thread
    def _raise_exception(self):
        res = ctypes.pythonapi.PyThreadState_SetAsyncExc(
                ctypes.c_long(self.ident), ctypes.py_object(SystemExit))
        print 'raised exception returned', res
        if res == 0:
            raise ValueError("Invalid thread ID")
        elif res != 1:
            # "if it returns a number greater than one, you're in trouble,
            # and you should call it again with exc=NULL to revert the effect"
            ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, 0)
            raise SystemError("PyThreadState_SetAsyncExc failed")
