
(cl:in-package :asdf)

(defsystem "iri_wam_arm_navigation-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :actionlib_msgs-msg
               :arm_navigation_msgs-msg
               :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "SimplePoseGoal" :depends-on ("_package_SimplePoseGoal"))
    (:file "_package_SimplePoseGoal" :depends-on ("_package"))
    (:file "SimplePoseActionResult" :depends-on ("_package_SimplePoseActionResult"))
    (:file "_package_SimplePoseActionResult" :depends-on ("_package"))
    (:file "SimplePoseActionGoal" :depends-on ("_package_SimplePoseActionGoal"))
    (:file "_package_SimplePoseActionGoal" :depends-on ("_package"))
    (:file "SimplePoseFeedback" :depends-on ("_package_SimplePoseFeedback"))
    (:file "_package_SimplePoseFeedback" :depends-on ("_package"))
    (:file "SimplePoseResult" :depends-on ("_package_SimplePoseResult"))
    (:file "_package_SimplePoseResult" :depends-on ("_package"))
    (:file "SimplePoseAction" :depends-on ("_package_SimplePoseAction"))
    (:file "_package_SimplePoseAction" :depends-on ("_package"))
    (:file "SimplePoseActionFeedback" :depends-on ("_package_SimplePoseActionFeedback"))
    (:file "_package_SimplePoseActionFeedback" :depends-on ("_package"))
  ))