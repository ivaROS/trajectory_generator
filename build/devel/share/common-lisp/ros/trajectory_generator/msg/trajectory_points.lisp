; Auto-generated. Do not edit!


(cl:in-package trajectory_generator-msg)


;//! \htmlinclude trajectory_points.msg.html

(cl:defclass <trajectory_points> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (points
    :reader points
    :initarg :points
    :type (cl:vector trajectory_generator-msg:trajectory_point)
   :initform (cl:make-array 0 :element-type 'trajectory_generator-msg:trajectory_point :initial-element (cl:make-instance 'trajectory_generator-msg:trajectory_point))))
)

(cl:defclass trajectory_points (<trajectory_points>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <trajectory_points>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'trajectory_points)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name trajectory_generator-msg:<trajectory_points> is deprecated: use trajectory_generator-msg:trajectory_points instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <trajectory_points>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader trajectory_generator-msg:header-val is deprecated.  Use trajectory_generator-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'points-val :lambda-list '(m))
(cl:defmethod points-val ((m <trajectory_points>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader trajectory_generator-msg:points-val is deprecated.  Use trajectory_generator-msg:points instead.")
  (points m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <trajectory_points>) ostream)
  "Serializes a message object of type '<trajectory_points>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'points))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'points))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <trajectory_points>) istream)
  "Deserializes a message object of type '<trajectory_points>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'points) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'points)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'trajectory_generator-msg:trajectory_point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<trajectory_points>)))
  "Returns string type for a message object of type '<trajectory_points>"
  "trajectory_generator/trajectory_points")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'trajectory_points)))
  "Returns string type for a message object of type 'trajectory_points"
  "trajectory_generator/trajectory_points")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<trajectory_points>)))
  "Returns md5sum for a message object of type '<trajectory_points>"
  "0e147c86b6a00a2dea6684eeee7cb4dd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'trajectory_points)))
  "Returns md5sum for a message object of type 'trajectory_points"
  "0e147c86b6a00a2dea6684eeee7cb4dd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<trajectory_points>)))
  "Returns full string definition for message of type '<trajectory_points>"
  (cl:format cl:nil "Header header~%trajectory_point[] points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: trajectory_generator/trajectory_point~%duration time~%float32 x~%float32 y~%float32 theta~%float32 v~%float32 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'trajectory_points)))
  "Returns full string definition for message of type 'trajectory_points"
  (cl:format cl:nil "Header header~%trajectory_point[] points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: trajectory_generator/trajectory_point~%duration time~%float32 x~%float32 y~%float32 theta~%float32 v~%float32 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <trajectory_points>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'points) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <trajectory_points>))
  "Converts a ROS message object to a list"
  (cl:list 'trajectory_points
    (cl:cons ':header (header msg))
    (cl:cons ':points (points msg))
))
