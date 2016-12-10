
(cl:in-package :asdf)

(defsystem "trajectory_generator-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "trajectory_point" :depends-on ("_package_trajectory_point"))
    (:file "_package_trajectory_point" :depends-on ("_package"))
    (:file "trajectory_points" :depends-on ("_package_trajectory_points"))
    (:file "_package_trajectory_points" :depends-on ("_package"))
  ))