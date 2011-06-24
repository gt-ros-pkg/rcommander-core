
(cl:in-package :asdf)

(defsystem "ptp_arm_action-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :actionlib_msgs-msg
               :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "LinearMovementAction" :depends-on ("_package_LinearMovementAction"))
    (:file "_package_LinearMovementAction" :depends-on ("_package"))
    (:file "LinearMovementGoal" :depends-on ("_package_LinearMovementGoal"))
    (:file "_package_LinearMovementGoal" :depends-on ("_package"))
    (:file "LinearMovementActionGoal" :depends-on ("_package_LinearMovementActionGoal"))
    (:file "_package_LinearMovementActionGoal" :depends-on ("_package"))
    (:file "LinearMovementResult" :depends-on ("_package_LinearMovementResult"))
    (:file "_package_LinearMovementResult" :depends-on ("_package"))
    (:file "LinearMovementActionResult" :depends-on ("_package_LinearMovementActionResult"))
    (:file "_package_LinearMovementActionResult" :depends-on ("_package"))
    (:file "LinearMovementFeedback" :depends-on ("_package_LinearMovementFeedback"))
    (:file "_package_LinearMovementFeedback" :depends-on ("_package"))
    (:file "LinearMovementActionFeedback" :depends-on ("_package_LinearMovementActionFeedback"))
    (:file "_package_LinearMovementActionFeedback" :depends-on ("_package"))
  ))