import roslib; roslib.load_manifest('rcommander_core')
import rospy
import actionlib
import trajectory_msgs.msg as tm
import numpy as np
import functools as ft
import sensor_msgs.msg as sm
import std_msgs.msg as stdm
import pr2_controllers_msgs.msg as pm
import geometry_msgs.msg as gm
import time

#Test this
def unwrap2(cpos, npos):
    two_pi = 2*np.pi
    nin = npos % two_pi
    n_multiples_2pi = np.floor(cpos/two_pi)
    return nin + n_multiples_2pi*two_pi


##
# Takes a normal ROS callback channel and gives it an on demand query style
# interface.
class GenericListener:
    ##
    # Message has to have a header
    # @param node_name name of node (if haven't been inited)
    # @param message_type type of message to listen for
    # @param listen_channel ROS channel to listen
    # @param frequency the frequency to expect messages (used to print warning statements to console)
    # @param message_extractor function to preprocess the message into a desired format
    # @param queue_size ROS subscriber queue (None = infinite)
    def __init__(self, node_name, message_type, listen_channel,
                 frequency, message_extractor=None, queue_size=None):
        try:
            print node_name, ': inited node.'
            rospy.init_node(node_name, anonymous=True)
        except rospy.ROSException, e:
            pass
        self.last_msg_returned   = None   #Last message returned to callers from this class
        self.last_call_back      = None   #Local time of last received message
        self.delay_tolerance     = 1/frequency #in seconds
        self.reading             = {'message':None, 'msg_id':-1}
        self.curid               = 0
        self.message_extractor = message_extractor

        def callback(*msg):
            #If this is a tuple (using message filter)
            if 'header' in dir(msg):
                if msg.__class__ == ().__class__:
                    msg_number = msg[0].header.seq
                else:
                    msg_number = msg.header.seq
            else:
                msg_number = self.curid
                self.curid += 1

            #*msg makes everything a tuple.  If length is one, msg = (msg, )
            if len(msg) == 1:
                msg = msg[0]
            
            self.reading  = {'message':msg, 'msg_id':msg_number}

            #Check for delayed messages
            self.last_call_back = time.time() #record when we have been called back last

        if message_type.__class__ == [].__class__:
            import message_filters
            subscribers = [message_filters.Subscriber(channel, mtype) for channel, mtype in zip(listen_channel, message_type)]
            queue_size = 10
            ts = message_filters.TimeSynchronizer(subscribers, queue_size)
            ts.registerCallback(callback)
        else:
            rospy.Subscriber(listen_channel, message_type, callback,
                             queue_size = queue_size)

        self.node_name = node_name
        #print node_name,': subscribed to', listen_channel
        rospy.loginfo('%s: subscribed to %s' % (node_name, listen_channel))

    def _check_for_delivery_hiccups(self):
        #If have received a message in the past
        if self.last_call_back != None:
            #Calculate how it has been
            time_diff = time.time() - self.last_call_back
            #If it has been longer than expected hz, complain
            if time_diff > self.delay_tolerance:
                print self.node_name, ': have not heard back from publisher in', time_diff, 's'

    def _wait_for_first_read(self, quiet=False):
        if not quiet:
            rospy.loginfo('%s: waiting for reading ...' % self.node_name)
        while self.reading['message'] == None and not rospy.is_shutdown():
            time.sleep(0.1)
            #if not quiet:
            #    print self.node_name, ': waiting for reading ...'

    ## 
    # Supported use cases
    # rfid   - want to get a reading, can be stale, no duplication allowed (allow None),        query speed important
    # hokuyo - want to get a reading, can be stale, no duplication allowed (don't want a None), willing to wait for new data (default)
    # ft     - want to get a reading, can be stale, duplication allowed    (don't want a None), query speed important
    # NOT ALLOWED                                   duplication allowed,                        willing to wait for new data
    def read(self, allow_duplication=False, willing_to_wait=True, warn=False, quiet=True):
        if allow_duplication:
            if willing_to_wait:
                raise RuntimeError('Invalid settings for read.')
            else: 
                # ft - want to get a reading, can be stale, duplication allowed (but don't want a None), query speed important
                #self._wait_for_first_read(quiet)
                reading                = self.reading
                self.last_msg_returned = reading['msg_id']
                if self.message_extractor is not None:
                    return self.message_extractor(reading['message'])
                else:
                    return reading['message']
        else:
            if willing_to_wait:
                # hokuyo - want to get a reading, can be stale, no duplication allowed (don't want a None), willing to wait for new data (default)
                self._wait_for_first_read(quiet)
                while self.reading['msg_id'] == self.last_msg_returned and not rospy.is_shutdown():
                    if warn:
                        self._check_for_delivery_hiccups()
                    time.sleep(1/1000.0)
                reading = self.reading
                self.last_msg_returned = reading['msg_id']
                if self.message_extractor is not None:
                    return self.message_extractor(reading['message'])
                else:
                    return reading['message']
            else:
                # rfid   - want to get a reading, can be stale, no duplication allowed (allow None),        query speed important
                if self.last_msg_returned == self.reading['msg_id']:
                    return None
                else:
                    reading = self.reading
                    self.last_msg_returned = reading['msg_id']
                    if self.message_extractor is not None:
                        return self.message_extractor(reading['message'])
                    else:
                        return reading['message']


class Joint:

    def __init__(self, name, joint_provider):
        self.joint_provider = joint_provider
        self.joint_names = rospy.get_param('/%s/joints' % name)
        self.pub = rospy.Publisher('%s/command' % name, tm.JointTrajectory)
        self.names_index = None
        self.zeros = [0 for j in range(len(self.joint_names))]

    def pose(self, joint_states=None):
        if joint_states == None:
            joint_states = self.joint_provider()

        if self.names_index == None:
            self.names_index = {}
            for i, n in enumerate(joint_states.name):
                self.names_index[n] = i
            self.joint_idx = [self.names_index[n] for n in self.joint_names]

        return (np.matrix(joint_states.position).T)[self.joint_idx, 0]

    def _create_trajectory(self, pos_mat, times, vel_mat=None):
        #Make JointTrajectoryPoints
        points = [tm.JointTrajectoryPoint() for i in range(pos_mat.shape[1])]
        for i in range(pos_mat.shape[1]):
            points[i].positions = pos_mat[:,i].A1.tolist()
            points[i].accelerations = self.zeros
            if vel_mat == None:
                points[i].velocities = self.zeros
            else:
                points[i].velocities = vel_mat[:,i].A1.tolist()

        for i in range(pos_mat.shape[1]):
            points[i].time_from_start = rospy.Duration(times[i])

        #Create JointTrajectory
        jt = tm.JointTrajectory()
        jt.joint_names = self.joint_names
        jt.points = points
        jt.header.stamp = rospy.get_rostime() #+ rospy.Duration(1.)
        return jt

    def set_poses(self, pos_mat, times):
        #pos_mat = np.column_stack([self.pose(), pos_mat])
        #times = [0] + times
        #times = times + .1
        joint_trajectory = self._create_trajectory(pos_mat, times)
        self.pub.publish(joint_trajectory)

    def get_joint_names(self):
        return self.joint_names


class PR2Arm(Joint):

    def __init__(self, joint_provider, tf_listener, arm, use_kinematics=True):
        joint_controller_name = arm + '_arm_controller'
        cart_controller_name = arm + '_cart'
        Joint.__init__(self, joint_controller_name, joint_provider)
        self.arm = arm
        self.tf_listener = tf_listener
        self.client = actionlib.SimpleActionClient('/%s/joint_trajectory_action' % joint_controller_name, pm.JointTrajectoryAction)
        self.joint_controller_name = joint_controller_name

        self.cart_posure_pub = rospy.Publisher("/%s/command_posture" % cart_controller_name, stdm.Float64MultiArray).publish
        self.cart_pose_pub = rospy.Publisher("/%s/command_pose" % cart_controller_name, gm.PoseStamped).publish
        if arm == 'l':
            self.full_arm_name = 'left'
        else:
            self.full_arm_name = 'right'

        if use_kinematics:
            self.kinematics = pr2k.PR2ArmKinematics(self.full_arm_name, self.tf_listener)
        #self.ik_utilities = iku.IKUtilities(self.full_arm_name, self.tf_listener) 

        self.POSTURES = {
            'off':          np.matrix([]),
            'mantis':       np.matrix([0, 1, 0,  -1, 3.14, -1, 3.14]).T,
            'elbowupr':     np.matrix([-0.79,0,-1.6,  9999, 9999, 9999, 9999]).T,
            'elbowupl':     np.matrix([0.79,0,1.6 , 9999, 9999, 9999, 9999]).T,
            'old_elbowupr': np.matrix([-0.79,0,-1.6, -0.79,3.14, -0.79,5.49]).T,
            'old_elbowupl': np.matrix([0.79,0,1.6, -0.79,3.14, -0.79,5.49]).T,
            'elbowdownr':   np.matrix([-0.028262077316910873, 1.2946342642324222, -0.25785640577652386, -1.5498884526859626]).T, 
            'elbowdownl':   np.matrix([-0.0088195719039858515, 1.2834828245284853, 0.20338442004843196, -1.5565279256852611]).T
            }

    def set_posture(self, posture_mat):
        self.cart_posure_pub(stdm.Float64MultiArray(data=posture_mat.A1.tolist()))

    ##
    # Send a cartesian pose to *_cart controllers
    # @param trans len 3 list
    # @param rot len 3 list
    # @param frame string
    # @param msg_time float
    def set_cart_pose(self, trans, rot, frame, msg_time):
        ps = gm.PoseStamped()
        for i, field in enumerate(['x', 'y', 'z']):
            exec('ps.pose.position.%s = trans[%d]' % (field, i))
        for i, field in enumerate(['x', 'y', 'z', 'w']):
            exec('ps.pose.orientation.%s = rot[%d]' % (field, i))
        ps.header.frame_id = frame
        ps.header.stamp = rospy.Time(msg_time)
        self.cart_pose_pub(ps)

    ##
    # @param pos_mat column matrix of poses
    # @param times array of times
    def set_poses(self, pos_mat, times, vel_mat=None, block=True):
        #p = self.pose()
        #for i in range(pos_mat.shape[1]):
        #    pos_mat[4,i] = unwrap2(p[4,0], pos_mat[4,i])
        #    pos_mat[6,i] = unwrap2(p[6,0], pos_mat[6,i])
        #    p = pos_mat[:,i]

        pos_mat = np.column_stack([self.pose(), pos_mat])
        #print 'SETPOSES', times, times.__class__
        times   = np.concatenate(([0], times))
        times = times + .1
        #print "SET POSES", pos_mat.shape, len(times)
        joint_traj = Joint._create_trajectory(self, pos_mat, times, vel_mat)

        #Create goal msg
        #joint_traj.header.stamp = rospy.get_rostime() + rospy.Duration(5.)
        g = pm.JointTrajectoryGoal()
        g.trajectory = joint_traj
        g.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(1.)
        self.client.send_goal(g)
        if block:
            return self.client.wait_for_result()
        return self.client.get_state()

    def stop_trajectory_execution(self):
        self.client.cancel_all_goals()

    def has_active_goal(self):
        s = self.client.get_state()
        if s == amsg.GoalStatus.ACTIVE or s == amsg.GoalStatus.PENDING:
            return True
        else:
            return False

    def set_poses_monitored(self, pos_mat, times, vel_mat=None, block=True, time_look_ahead=.050):
        joint_traj = Joint._create_trajectory(self, pos_mat, times, vel_mat)

        #Create goal msg
        joint_traj.header.stamp = rospy.get_rostime() + rospy.Duration(1.)
        g = pm.JointTrajectoryGoal()
        g.trajectory = joint_traj
        self.client.send_goal(g)
        if block:
            return self.client.wait_for_result()
        return self.client.get_state()

    def set_pose(self, pos, nsecs=5., block=True):
        for i in range(2):
            cpos = self.pose()
        pos[4,0] = unwrap(cpos[4,0], pos[4,0])
        pos[6,0] = unwrap(cpos[6,0], pos[6,0])
        self.set_poses(np.column_stack([pos]), np.array([nsecs]), block=block)
        #self.set_poses(np.column_stack([cpos, pos]), np.array([min_time, min_time+nsecs]), block=block)

    def pose_cartesian(self, frame='base_link'):
        gripper_tool_frame = self.arm + '_gripper_tool_frame'
        return tfu.transform(frame, gripper_tool_frame, self.tf_listener)

    def pose_cartesian_tf(self, frame='base_link'):
        p, r = tfu.matrix_as_tf(self.pose_cartesian(frame))
        return np.matrix(p).T, np.matrix(r).T


class PR2Torso(Joint):

    def __init__(self, joint_provider):
        Joint.__init__(self, 'torso_controller', joint_provider)
        self.torso = actionlib.SimpleActionClient('torso_controller/position_joint_action', pm.SingleJointPositionAction)
        rospy.loginfo('waiting for torso_controller')
        self.torso.wait_for_server()

    def set_pose(self, p, block=True):
        self.torso.send_goal(pm.SingleJointPositionGoal(position = p))
        if block:
            self.torso.wait_for_result()
        return self.torso.get_state()


class PR2Head(Joint):

    def __init__(self, joint_provider):
        Joint.__init__(self, 'head_traj_controller', joint_provider)

    def set_pose(self, pos, nsecs=5.):
        for i in range(2):
            cpos = self.pose()
        min_time = .1
        self.set_poses(np.column_stack([cpos, pos]), np.array([min_time, min_time+nsecs]))


class PR2:

    def __init__(self, tf_listener):
        self.tf_listener = tf_listener
        jl = GenericListener('joint_state_listener', sm.JointState, 'joint_states', 100)
        joint_provider = ft.partial(jl.read, allow_duplication=False, willing_to_wait=True, warn=False, quiet=True)

        self.left = PR2Arm(joint_provider, tf_listener, 'l', use_kinematics=False)
        self.right = PR2Arm(joint_provider, tf_listener, 'r', use_kinematics=False)
        self.torso = PR2Torso(joint_provider)
        self.head = PR2Head(joint_provider)

