#!/usr/bin/python
import roslib
roslib.load_manifest('tf_broadcast_server')
import rospy
import tf
from tf_broadcast_server.srv import *

##
# contains a list of default 'good' frames, loaded in from parameter file
# (has just python code) 
#
# base_link
# torso_lift_link
# r_gripper_tool_frame
# l_gripper_tool_frame
# base_laser_link
# laser_tilt_link

class TFBroadcastServer:

    def __init__(self, user_frames):
        self.broadcaster = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()
        self.tfdict = {}
        self.should_run = True
        self.user_frames = user_frames
        
        self.broadcast_transform_srv = rospy.Service('broadcast_transform', 
                BroadcastTransform, self.broadcast_transform_cb)
        self.remove_transform_srv = rospy.Service('stop_broadcast_transform',
                RemoveTransform, self.remove_transform_cb)
        self.clear_all_transforms_srv = rospy.Service('clear_all_transforms', 
                ClearTransforms, self.clear_all_transforms_cb)
        self.get_transforms_srv = rospy.Service('get_transforms',
                GetTransforms, self.get_transforms_cb)

    def run(self):
        r = rospy.Rate(30)
        rospy.loginfo('TF broadcast server UP!')
        while not rospy.is_shutdown():
        #while self.should_run:
            for k in self.tfdict.keys():
                posestamped = self.tfdict[k]['pose']
                parent       = self.tfdict[k]['parent']
                name         = k
                p = posestamped.pose.position
                o = posestamped.pose.orientation
                self.broadcaster.sendTransform((p.x, p.y, p.z), 
                    (o.x, o.y, o.z, o.w), 
                    rospy.Time.now(), name, parent)
            #for posestamped, name, parent in self.tfdict:
            r.sleep()

    def broadcast_transform_cb(self, req):
        rospy.loginfo('Broadcasting frame %s with parent %s and pose %s' % (req.frame_name, req.pose.header.frame_id, str(req.pose)))
        self.broadcast_transform(req.pose, req.pose.header.frame_id, req.frame_name)
        return BroadcastTransformResponse()

    def remove_transform_cb(self, req):
        self.remove_transform(req.frame_name)
        return RemoveTransformResponse()

    def clear_all_transforms_cb(self, req):
        self.clear_all_transforms()
        return ClearTransformsResponse()

    def get_transforms_cb(self, req):
        return GetTransformsResponse(self.get_transforms())

    def broadcast_transform(self, pose_stamped, parent, frame_name):
        #print pose_stamped, parent, frame_name
        self.tfdict[frame_name] = {'parent': parent, 'pose': pose_stamped}

    def remove_transform(self, frame_name):
        self.tfdict.pop(frame_name)

    def clear_all_transforms(self):
        self.tfdict = {}

    ##
    # valid transforms are thse that are declared explicitly and in tfdict
    def get_transforms(self):
        all_frames = self.tf_listener.getFrameStrings()
        valid_transforms = []
        tfkeys = ['/' + n for n in self.tfdict.keys()]
        frames = set(self.user_frames + tfkeys)
        #print all_frames
        #print '\n'
        #print tfkeys
        for T in frames:
            if T in all_frames:
                valid_transforms.append(T)
        return valid_transforms


if __name__ == '__main__':
    rospy.init_node('tf_broadcast_server')
    user_frames = ['/base_link', '/torso_lift_link', '/r_gripper_tool_frame', 
                   '/l_gripper_tool_frame', '/base_laser_link', '/laser_tilt_link', '/map']
    server = TFBroadcastServer(user_frames)
    server.run()


