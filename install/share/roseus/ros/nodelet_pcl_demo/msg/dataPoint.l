;; Auto-generated. Do not edit!


(when (boundp 'nodelet_pcl_demo::dataPoint)
  (if (not (find-package "NODELET_PCL_DEMO"))
    (make-package "NODELET_PCL_DEMO"))
  (shadow 'dataPoint (find-package "NODELET_PCL_DEMO")))
(unless (find-package "NODELET_PCL_DEMO::DATAPOINT")
  (make-package "NODELET_PCL_DEMO::DATAPOINT"))

(in-package "ROS")
;;//! \htmlinclude dataPoint.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass nodelet_pcl_demo::dataPoint
  :super ros::object
  :slots (_header _myPoint ))

(defmethod nodelet_pcl_demo::dataPoint
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:myPoint __myPoint) (instance geometry_msgs::Point :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _myPoint __myPoint)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:myPoint
   (&rest __myPoint)
   (if (keywordp (car __myPoint))
       (send* _myPoint __myPoint)
     (progn
       (if __myPoint (setq _myPoint (car __myPoint)))
       _myPoint)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Point _myPoint
    (send _myPoint :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Point _myPoint
       (send _myPoint :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Point _myPoint
     (send _myPoint :deserialize buf ptr-) (incf ptr- (send _myPoint :serialization-length))
   ;;
   self)
  )

(setf (get nodelet_pcl_demo::dataPoint :md5sum-) "d5b28115b7d6a030008e1aa363ff62e3")
(setf (get nodelet_pcl_demo::dataPoint :datatype-) "nodelet_pcl_demo/dataPoint")
(setf (get nodelet_pcl_demo::dataPoint :definition-)
      "Header header
geometry_msgs/Point myPoint

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :nodelet_pcl_demo/dataPoint "d5b28115b7d6a030008e1aa363ff62e3")


