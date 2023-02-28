;; Auto-generated. Do not edit!


(when (boundp 'omni_driver::TeleopControl)
  (if (not (find-package "OMNI_DRIVER"))
    (make-package "OMNI_DRIVER"))
  (shadow 'TeleopControl (find-package "OMNI_DRIVER")))
(unless (find-package "OMNI_DRIVER::TELEOPCONTROL")
  (make-package "OMNI_DRIVER::TELEOPCONTROL"))

(in-package "ROS")
;;//! \htmlinclude TeleopControl.msg.html


(defclass omni_driver::TeleopControl
  :super ros::object
  :slots (_vel_joint _vel_effector _mode ))

(defmethod omni_driver::TeleopControl
  (:init
   (&key
    ((:vel_joint __vel_joint) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:vel_effector __vel_effector) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:mode __mode) 0)
    )
   (send-super :init)
   (setq _vel_joint __vel_joint)
   (setq _vel_effector __vel_effector)
   (setq _mode (round __mode))
   self)
  (:vel_joint
   (&optional __vel_joint)
   (if __vel_joint (setq _vel_joint __vel_joint)) _vel_joint)
  (:vel_effector
   (&optional __vel_effector)
   (if __vel_effector (setq _vel_effector __vel_effector)) _vel_effector)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:serialization-length
   ()
   (+
    ;; float64[] _vel_joint
    (* 8    (length _vel_joint)) 4
    ;; float64[] _vel_effector
    (* 8    (length _vel_effector)) 4
    ;; uint8 _mode
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _vel_joint
     (write-long (length _vel_joint) s)
     (dotimes (i (length _vel_joint))
       (sys::poke (elt _vel_joint i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _vel_effector
     (write-long (length _vel_effector) s)
     (dotimes (i (length _vel_effector))
       (sys::poke (elt _vel_effector i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; uint8 _mode
       (write-byte _mode s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _vel_joint
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _vel_joint (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _vel_joint i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _vel_effector
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _vel_effector (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _vel_effector i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; uint8 _mode
     (setq _mode (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get omni_driver::TeleopControl :md5sum-) "acc00fe3a847581a668e7968e27d235a")
(setf (get omni_driver::TeleopControl :datatype-) "omni_driver/TeleopControl")
(setf (get omni_driver::TeleopControl :definition-)
      "float64[] vel_joint
float64[] vel_effector
uint8 mode

")



(provide :omni_driver/TeleopControl "acc00fe3a847581a668e7968e27d235a")


