import numpy as np
import tf.transformations as tr
import geometry_msgs.msg as geo
import rospy
import tf

#Copied (gasp!) from object_manipulator.convert_functions
#(first premultiply by transform if given)
def mat_to_pose(mat, transform = None):
    if transform != None:
        mat = transform * mat
    pose = geo.Pose()
    pose.position.x = mat[0,3]
    pose.position.y = mat[1,3]
    pose.position.z = mat[2,3]
    quat = tf.transformations.quaternion_from_matrix(mat)
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]
    return pose

##make a PoseStamped out of a Pose
def stamp_pose(pose, frame_id):
    pose_stamped = geo.PoseStamped()
    stamp_msg(pose_stamped, frame_id)
    pose_stamped.pose = pose
    return pose_stamped

##change the frame of a PoseStamped
def change_pose_stamped_frame(tf_listener, pose, frame):

    #convert the PoseStamped to the desired frame, if necessary
    if pose.header.frame_id != frame:
        pose.header.stamp = rospy.Time(0)
        tf_listener.waitForTransform(frame, pose.header.frame_id, pose.header.stamp, rospy.Duration(5))
        try:
            trans_pose = tf_listener.transformPose(frame, pose)
        except rospy.ServiceException, e:
            print "pose:\n", pose
            print "frame:", frame
            rospy.logerr("change_pose_stamped_frame: error in transforming pose from " + pose.header.frame_id + " to " + frame + "error msg: %s"%e)
            return None
    else:
        trans_pose = pose

    return trans_pose


def tf_as_matrix(tup):
    return np.matrix(tr.translation_matrix(tup[0])) * np.matrix(tr.quaternion_matrix(tup[1])) 

def transform(to_frame, from_frame, tflistener, t=0):
    return tf_as_matrix(tflistener.lookupTransform(to_frame, from_frame, rospy.Time(t)))

def tf_as_matrix(tup):
    return np.matrix(tr.translation_matrix(tup[0])) * np.matrix(tr.quaternion_matrix(tup[1])) 

def matrix_as_tf(mat):
    return (tr.translation_from_matrix(mat), tr.quaternion_from_matrix(mat))

def origin_to_frame(origin, supplement_frame, tf_listener, command_frame):
    m = origin
    if isinstance(m, geo.PointStamped):
        #point in some frame, needs orientation...
        #print 'command_frame', command_frame, 'supplement_frame', supplement_frame, 'header_frame', m.header.frame_id
        command_frame_T_supplement_frame = tf_as_matrix(tf_listener.lookupTransform(command_frame, supplement_frame, rospy.Time(0)))
        #print 'command_frame_T_supplement_frame\n', command_frame_T_supplement_frame
        command_frame_T_header_frame = tf_as_matrix(tf_listener.lookupTransform(command_frame, m.header.frame_id, rospy.Time(0)))
        #print 'command_frame_T_header_frame\n', command_frame_T_header_frame
        point_header = np.matrix([m.point.x, m.point.y, m.point.z, 1.]).T
        point_command_frame = command_frame_T_header_frame * point_header
        #print 'point header\n', point_header.T
        #print 'point_command_frame\n', point_command_frame.T

        command_frame_T_point_frame = command_frame_T_supplement_frame.copy()
        command_frame_T_point_frame[0:3,3] = point_command_frame[0:3,0]
        CMD_T_frame = command_frame_T_point_frame
        #print 'command_frame_T_point_frame\n', command_frame_T_point_frame

    #If it's a pose stamped then we turn the pose stamped into a frame?
    elif isinstance(m, geo.PoseStamped):
        fid_T_p = pose_to_mat(m.pose)
        print 'fid_T_p\n', fid_T_p
        print fid_T_p[0:3,3].T
        CMD_T_fid = tf_as_matrix(tf_listener.lookupTransform(command_frame, m.header.frame_id, rospy.Time(0)))
        print 'FRAMES', command_frame, m.header.frame_id
        print 'CMD_T_fid\n', CMD_T_fid
        print CMD_T_fid[0:3,3].T
        CMD_T_frame = CMD_T_fid * fid_T_p
        print 'CMD_T_frame\n', CMD_T_frame
        print CMD_T_frame[0:3,3].T
    else:
        raise RuntimeError('Got origin that is an instance of ' + str(m.__class__))

    return CMD_T_frame
