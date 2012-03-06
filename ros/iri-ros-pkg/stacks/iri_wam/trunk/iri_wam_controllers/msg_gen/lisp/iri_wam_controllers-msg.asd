
(cl:in-package :asdf)

(defsystem "iri_wam_controllers-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "PathDuration" :depends-on ("_package_PathDuration"))
    (:file "_package_PathDuration" :depends-on ("_package"))
  ))