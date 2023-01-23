
(cl:in-package :asdf)

(defsystem "geomagic_control-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "DeviceButtonEvent" :depends-on ("_package_DeviceButtonEvent"))
    (:file "_package_DeviceButtonEvent" :depends-on ("_package"))
    (:file "DeviceFeedback" :depends-on ("_package_DeviceFeedback"))
    (:file "_package_DeviceFeedback" :depends-on ("_package"))
  ))