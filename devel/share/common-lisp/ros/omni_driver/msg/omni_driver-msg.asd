
(cl:in-package :asdf)

(defsystem "omni_driver-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "OmniButtonEvent" :depends-on ("_package_OmniButtonEvent"))
    (:file "_package_OmniButtonEvent" :depends-on ("_package"))
    (:file "OmniFeedback" :depends-on ("_package_OmniFeedback"))
    (:file "_package_OmniFeedback" :depends-on ("_package"))
    (:file "OmniState" :depends-on ("_package_OmniState"))
    (:file "_package_OmniState" :depends-on ("_package"))
    (:file "TeleopControl" :depends-on ("_package_TeleopControl"))
    (:file "_package_TeleopControl" :depends-on ("_package"))
  ))