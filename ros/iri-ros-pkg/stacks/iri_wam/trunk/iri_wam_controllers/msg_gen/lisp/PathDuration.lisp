; Auto-generated. Do not edit!


(cl:in-package iri_wam_controllers-msg)


;//! \htmlinclude PathDuration.msg.html

(cl:defclass <PathDuration> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (duration_path_expected
    :reader duration_path_expected
    :initarg :duration_path_expected
    :type cl:real
    :initform 0))
)

(cl:defclass PathDuration (<PathDuration>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PathDuration>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PathDuration)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name iri_wam_controllers-msg:<PathDuration> is deprecated: use iri_wam_controllers-msg:PathDuration instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <PathDuration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader iri_wam_controllers-msg:header-val is deprecated.  Use iri_wam_controllers-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'duration_path_expected-val :lambda-list '(m))
(cl:defmethod duration_path_expected-val ((m <PathDuration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader iri_wam_controllers-msg:duration_path_expected-val is deprecated.  Use iri_wam_controllers-msg:duration_path_expected instead.")
  (duration_path_expected m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PathDuration>) ostream)
  "Serializes a message object of type '<PathDuration>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'duration_path_expected)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'duration_path_expected) (cl:floor (cl:slot-value msg 'duration_path_expected)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PathDuration>) istream)
  "Deserializes a message object of type '<PathDuration>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'duration_path_expected) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PathDuration>)))
  "Returns string type for a message object of type '<PathDuration>"
  "iri_wam_controllers/PathDuration")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PathDuration)))
  "Returns string type for a message object of type 'PathDuration"
  "iri_wam_controllers/PathDuration")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PathDuration>)))
  "Returns md5sum for a message object of type '<PathDuration>"
  "fd865e85cc346994ac32a0d0a54b9778")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PathDuration)))
  "Returns md5sum for a message object of type 'PathDuration"
  "fd865e85cc346994ac32a0d0a54b9778")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PathDuration>)))
  "Returns full string definition for message of type '<PathDuration>"
  (cl:format cl:nil "Header header~%duration duration_path_expected~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PathDuration)))
  "Returns full string definition for message of type 'PathDuration"
  (cl:format cl:nil "Header header~%duration duration_path_expected~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PathDuration>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PathDuration>))
  "Converts a ROS message object to a list"
  (cl:list 'PathDuration
    (cl:cons ':header (header msg))
    (cl:cons ':duration_path_expected (duration_path_expected msg))
))
