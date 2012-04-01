
(cl:in-package :asdf)

(defsystem "iri_wam_arm_navigation-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "PosePath" :depends-on ("_package_PosePath"))
    (:file "_package_PosePath" :depends-on ("_package"))
  ))