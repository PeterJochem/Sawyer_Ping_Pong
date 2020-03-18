; Auto-generated. Do not edit!


(cl:in-package nodelet_pcl_demo-msg)


;//! \htmlinclude velocity.msg.html

(cl:defclass <velocity> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (myPoint
    :reader myPoint
    :initarg :myPoint
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point)))
)

(cl:defclass velocity (<velocity>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <velocity>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'velocity)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name nodelet_pcl_demo-msg:<velocity> is deprecated: use nodelet_pcl_demo-msg:velocity instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <velocity>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nodelet_pcl_demo-msg:header-val is deprecated.  Use nodelet_pcl_demo-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'myPoint-val :lambda-list '(m))
(cl:defmethod myPoint-val ((m <velocity>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nodelet_pcl_demo-msg:myPoint-val is deprecated.  Use nodelet_pcl_demo-msg:myPoint instead.")
  (myPoint m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <velocity>) ostream)
  "Serializes a message object of type '<velocity>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'myPoint) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <velocity>) istream)
  "Deserializes a message object of type '<velocity>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'myPoint) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<velocity>)))
  "Returns string type for a message object of type '<velocity>"
  "nodelet_pcl_demo/velocity")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'velocity)))
  "Returns string type for a message object of type 'velocity"
  "nodelet_pcl_demo/velocity")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<velocity>)))
  "Returns md5sum for a message object of type '<velocity>"
  "d5b28115b7d6a030008e1aa363ff62e3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'velocity)))
  "Returns md5sum for a message object of type 'velocity"
  "d5b28115b7d6a030008e1aa363ff62e3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<velocity>)))
  "Returns full string definition for message of type '<velocity>"
  (cl:format cl:nil "Header header~%geometry_msgs/Point myPoint~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'velocity)))
  "Returns full string definition for message of type 'velocity"
  (cl:format cl:nil "Header header~%geometry_msgs/Point myPoint~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <velocity>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'myPoint))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <velocity>))
  "Converts a ROS message object to a list"
  (cl:list 'velocity
    (cl:cons ':header (header msg))
    (cl:cons ':myPoint (myPoint msg))
))
