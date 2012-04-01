; Auto-generated. Do not edit!


(cl:in-package iri_wam_arm_navigation-srv)


;//! \htmlinclude PosePath-request.msg.html

(cl:defclass <PosePath-request> (roslisp-msg-protocol:ros-message)
  ((pose_stamped
    :reader pose_stamped
    :initarg :pose_stamped
    :type geometry_msgs-msg:PoseStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PoseStamped)))
)

(cl:defclass PosePath-request (<PosePath-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PosePath-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PosePath-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name iri_wam_arm_navigation-srv:<PosePath-request> is deprecated: use iri_wam_arm_navigation-srv:PosePath-request instead.")))

(cl:ensure-generic-function 'pose_stamped-val :lambda-list '(m))
(cl:defmethod pose_stamped-val ((m <PosePath-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader iri_wam_arm_navigation-srv:pose_stamped-val is deprecated.  Use iri_wam_arm_navigation-srv:pose_stamped instead.")
  (pose_stamped m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PosePath-request>) ostream)
  "Serializes a message object of type '<PosePath-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose_stamped) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PosePath-request>) istream)
  "Deserializes a message object of type '<PosePath-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose_stamped) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PosePath-request>)))
  "Returns string type for a service object of type '<PosePath-request>"
  "iri_wam_arm_navigation/PosePathRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PosePath-request)))
  "Returns string type for a service object of type 'PosePath-request"
  "iri_wam_arm_navigation/PosePathRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PosePath-request>)))
  "Returns md5sum for a message object of type '<PosePath-request>"
  "25e4237af36cc7f99ce17ab2eca624a4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PosePath-request)))
  "Returns md5sum for a message object of type 'PosePath-request"
  "25e4237af36cc7f99ce17ab2eca624a4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PosePath-request>)))
  "Returns full string definition for message of type '<PosePath-request>"
  (cl:format cl:nil "geometry_msgs/PoseStamped pose_stamped~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PosePath-request)))
  "Returns full string definition for message of type 'PosePath-request"
  (cl:format cl:nil "geometry_msgs/PoseStamped pose_stamped~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PosePath-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose_stamped))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PosePath-request>))
  "Converts a ROS message object to a list"
  (cl:list 'PosePath-request
    (cl:cons ':pose_stamped (pose_stamped msg))
))
;//! \htmlinclude PosePath-response.msg.html

(cl:defclass <PosePath-response> (roslisp-msg-protocol:ros-message)
  ((pose_stamped
    :reader pose_stamped
    :initarg :pose_stamped
    :type (cl:vector geometry_msgs-msg:PoseStamped)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:PoseStamped :initial-element (cl:make-instance 'geometry_msgs-msg:PoseStamped))))
)

(cl:defclass PosePath-response (<PosePath-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PosePath-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PosePath-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name iri_wam_arm_navigation-srv:<PosePath-response> is deprecated: use iri_wam_arm_navigation-srv:PosePath-response instead.")))

(cl:ensure-generic-function 'pose_stamped-val :lambda-list '(m))
(cl:defmethod pose_stamped-val ((m <PosePath-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader iri_wam_arm_navigation-srv:pose_stamped-val is deprecated.  Use iri_wam_arm_navigation-srv:pose_stamped instead.")
  (pose_stamped m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PosePath-response>) ostream)
  "Serializes a message object of type '<PosePath-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'pose_stamped))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'pose_stamped))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PosePath-response>) istream)
  "Deserializes a message object of type '<PosePath-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'pose_stamped) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'pose_stamped)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:PoseStamped))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PosePath-response>)))
  "Returns string type for a service object of type '<PosePath-response>"
  "iri_wam_arm_navigation/PosePathResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PosePath-response)))
  "Returns string type for a service object of type 'PosePath-response"
  "iri_wam_arm_navigation/PosePathResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PosePath-response>)))
  "Returns md5sum for a message object of type '<PosePath-response>"
  "25e4237af36cc7f99ce17ab2eca624a4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PosePath-response)))
  "Returns md5sum for a message object of type 'PosePath-response"
  "25e4237af36cc7f99ce17ab2eca624a4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PosePath-response>)))
  "Returns full string definition for message of type '<PosePath-response>"
  (cl:format cl:nil "geometry_msgs/PoseStamped[] pose_stamped~%~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PosePath-response)))
  "Returns full string definition for message of type 'PosePath-response"
  (cl:format cl:nil "geometry_msgs/PoseStamped[] pose_stamped~%~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PosePath-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'pose_stamped) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PosePath-response>))
  "Converts a ROS message object to a list"
  (cl:list 'PosePath-response
    (cl:cons ':pose_stamped (pose_stamped msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'PosePath)))
  'PosePath-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'PosePath)))
  'PosePath-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PosePath)))
  "Returns string type for a service object of type '<PosePath>"
  "iri_wam_arm_navigation/PosePath")