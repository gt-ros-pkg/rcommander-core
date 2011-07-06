; Auto-generated. Do not edit!


(cl:in-package ptp_arm_action-msg)


;//! \htmlinclude LinearMovementGoal.msg.html

(cl:defclass <LinearMovementGoal> (roslisp-msg-protocol:ros-message)
  ((relative
    :reader relative
    :initarg :relative
    :type cl:boolean
    :initform cl:nil)
   (trans_vel
    :reader trans_vel
    :initarg :trans_vel
    :type cl:float
    :initform 0.0)
   (rot_vel
    :reader rot_vel
    :initarg :rot_vel
    :type cl:float
    :initform 0.0)
   (goal
    :reader goal
    :initarg :goal
    :type geometry_msgs-msg:PoseStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PoseStamped)))
)

(cl:defclass LinearMovementGoal (<LinearMovementGoal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LinearMovementGoal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LinearMovementGoal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ptp_arm_action-msg:<LinearMovementGoal> is deprecated: use ptp_arm_action-msg:LinearMovementGoal instead.")))

(cl:ensure-generic-function 'relative-val :lambda-list '(m))
(cl:defmethod relative-val ((m <LinearMovementGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ptp_arm_action-msg:relative-val is deprecated.  Use ptp_arm_action-msg:relative instead.")
  (relative m))

(cl:ensure-generic-function 'trans_vel-val :lambda-list '(m))
(cl:defmethod trans_vel-val ((m <LinearMovementGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ptp_arm_action-msg:trans_vel-val is deprecated.  Use ptp_arm_action-msg:trans_vel instead.")
  (trans_vel m))

(cl:ensure-generic-function 'rot_vel-val :lambda-list '(m))
(cl:defmethod rot_vel-val ((m <LinearMovementGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ptp_arm_action-msg:rot_vel-val is deprecated.  Use ptp_arm_action-msg:rot_vel instead.")
  (rot_vel m))

(cl:ensure-generic-function 'goal-val :lambda-list '(m))
(cl:defmethod goal-val ((m <LinearMovementGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ptp_arm_action-msg:goal-val is deprecated.  Use ptp_arm_action-msg:goal instead.")
  (goal m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LinearMovementGoal>) ostream)
  "Serializes a message object of type '<LinearMovementGoal>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'relative) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'trans_vel))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'rot_vel))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'goal) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LinearMovementGoal>) istream)
  "Deserializes a message object of type '<LinearMovementGoal>"
    (cl:setf (cl:slot-value msg 'relative) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'trans_vel) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rot_vel) (roslisp-utils:decode-double-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'goal) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LinearMovementGoal>)))
  "Returns string type for a message object of type '<LinearMovementGoal>"
  "ptp_arm_action/LinearMovementGoal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LinearMovementGoal)))
  "Returns string type for a message object of type 'LinearMovementGoal"
  "ptp_arm_action/LinearMovementGoal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LinearMovementGoal>)))
  "Returns md5sum for a message object of type '<LinearMovementGoal>"
  "3eceff484681a8278d5e078b51a65c24")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LinearMovementGoal)))
  "Returns md5sum for a message object of type 'LinearMovementGoal"
  "3eceff484681a8278d5e078b51a65c24")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LinearMovementGoal>)))
  "Returns full string definition for message of type '<LinearMovementGoal>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%bool relative~%float64 trans_vel~%float64 rot_vel~%geometry_msgs/PoseStamped goal~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LinearMovementGoal)))
  "Returns full string definition for message of type 'LinearMovementGoal"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%bool relative~%float64 trans_vel~%float64 rot_vel~%geometry_msgs/PoseStamped goal~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LinearMovementGoal>))
  (cl:+ 0
     1
     8
     8
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'goal))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LinearMovementGoal>))
  "Converts a ROS message object to a list"
  (cl:list 'LinearMovementGoal
    (cl:cons ':relative (relative msg))
    (cl:cons ':trans_vel (trans_vel msg))
    (cl:cons ':rot_vel (rot_vel msg))
    (cl:cons ':goal (goal msg))
))
