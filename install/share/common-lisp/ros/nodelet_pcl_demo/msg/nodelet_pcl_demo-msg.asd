
(cl:in-package :asdf)

(defsystem "nodelet_pcl_demo-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "PointArray" :depends-on ("_package_PointArray"))
    (:file "_package_PointArray" :depends-on ("_package"))
    (:file "dataPoint" :depends-on ("_package_dataPoint"))
    (:file "_package_dataPoint" :depends-on ("_package"))
  ))