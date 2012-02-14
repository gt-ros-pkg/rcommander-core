import roslib; roslib.load_manifest('rcommander')
from pr2_interactive_manipulation.srv import *
import rospy
from PyQt4 import QtCore, QtGui
import graph_model as gm
import sys
import os
import os.path as pt

class WatchDirectory(QtCore.QObject):
    def __init__(self, path, dir_changed_func=None, file_changed_func=None):
        self.path = path
        self.fs_watcher = QtCore.QFileSystemWatcher([path])
        self.fs_watcher.connect(self.fs_watcher, QtCore.SIGNAL('directoryChanged(QString)'), self.directory_changed)
        self.fs_watcher.connect(self.fs_watcher, QtCore.SIGNAL('fileChanged(QString)'), self.file_changed)
        self.file_changed_func = file_changed_func
        self.dir_changed_func = dir_changed_func

    def __del__(self):
        rospy.loginfo('Deleting watcher for ' + self.path)
        self.fs_watcher.disconnect(self.fs_watcher, 
                QtCore.SIGNAL('directoryChanged(QString)'), self.directory_changed)
        self.fs_watcher.disconnect(self.fs_watcher, 
                QtCore.SIGNAL('fileChanged(QString)'), self.file_changed)

    @QtCore.pyqtSlot(str)
    def directory_changed(self, path):
        if self.dir_changed_func != None:
            self.dir_changed_func(path)
    
    @QtCore.pyqtSlot(str)
    def file_changed(self, path):
        if self.file_changed_func != None:
            self.file_changed_func(path)

class RCommanderAutoServer:

    def __init__(self, robot, path_to_rcommander_files):
        self.path_to_rcommander_files = path_to_rcommander_files
        self.robot = robot
        self.action_dict = {}
        self.main_dir_watcher = WatchDirectory(self.path_to_rcommander_files, self.main_directory_changed)
        self.main_directory_changed(self.path_to_rcommander_files)
        #rospy.Service('list_rcommander_actions', ActionInfo, self.list_action_cb)
        #self.main_directory_changed(self.path_to_rcommander_files)
        #self.actserv = actionlib.SimpleActionServer('run_rcommander_action', pim.RunScriptAction, execute_cb=self.execute_cb, auto_start=False)
        #self.actserv.start()

    def _find_all_actions(self):
        dirs = [] #[d for d in os.listdir(path_name) if os.path.isdir(d)]
        for d in os.listdir(self.path_to_rcommander_files):
            if os.path.isdir(os.path.join(self.path_to_rcommander_files, d)):
                dirs.append(d)
        return dirs

    def _load(self, action):
        rospy.loginfo('Loaded ' + action)
        action_path = os.path.join(self.path_to_rcommander_files, action)
        return  {#'server':  ScriptedActionServer(action, action_path, self.robot),
                 'watcher': WatchDirectory(action_path, self.action_directory_changed)}

    def action_directory_changed(self, action_path_name):
        action_path_name = str(action_path_name)
        action_name = pt.split(action_path_name)[1]
        rospy.loginfo('action_name ' + action_name)
        rospy.loginfo('action_directory_changed: ' + action_name)
        self.action_dict[action_name] = self._load(action_name)

    def main_directory_changed(self, main_path_name):
        rospy.loginfo('main_directory_changed: rescanning ' + main_path_name)
        ndict = {}
        actions = self._find_all_actions()
        for action in actions:
            if self.action_dict.has_key(action):
                ndict[action] = self.action_dict[action]
            else:
                ndict[action] = self._load(action)
        self.action_dict = ndict

    def list_action_cb(self, req):
        actions = self.action_dict.keys()
        actions.sort()
        return ActionInfoResponse(actions)

class ScriptedActionServer:

    def __init__(self, action_name, path_to_action, robot):
        self.action_name = action_name
        self.path_to_action = path_to_action
        self.robot = robot

    def execute(self, pose_stamped, actserver):
        self.graph_model = gm.GraphModel.load(self.path_to_action, self.robot)
        r = rospy.Rate(30)
        state_machine = self.graph_model.create_state_machine()
        rthread = smtr.ThreadRunSM(self.action_name, state_machine)
        rthread.start()

        #Do something
        while True:
            if actserver.is_preempt_requested():
                actserver.set_preempted()
                success = False
                break

            if rthread.exception:
                raise rthread.exception

            if rthread.outcome != None:
                success = True
                break

            if not rthread.isAlive():
                raise RuntimeError("Thread died unexpectedly.")

            r.sleep()

        if success:
            state_machine_output = rthread.outcome
            #result = pim.PoseStampedScriptedResult(state_machine_output)
            result = pim.RunScriptResult(state_machine_output)
            rospy.loginfo("%s: succeeded with %s" % (self.action_name, state_machine_output))
            actserver.set_succeeded(result)
        else:
            actserver.set_aborted()

def run(robot, path):
#    from optparse import OptionParser
    #p = OptionParser()
    #options, args = p.parse_args()
    #print options.name, options.dir
    app = QtGui.QApplication(sys.argv)
    server = RCommanderAutoServer(robot, path)
    app.exec_()

