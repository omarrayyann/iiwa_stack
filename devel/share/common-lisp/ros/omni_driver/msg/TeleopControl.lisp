; Auto-generated. Do not edit!


(cl:in-package omni_driver-msg)


;//! \htmlinclude TeleopControl.msg.html

(cl:defclass <TeleopControl> (roslisp-msg-protocol:ros-message)
  ((vel_joint
    :reader vel_joint
    :initarg :vel_joint
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (vel_effector
    :reader vel_effector
    :initarg :vel_effector
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (mode
    :reader mode
    :initarg :mode
    :type cl:fixnum
    :initform 0))
)

(cl:defclass TeleopControl (<TeleopControl>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TeleopControl>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TeleopControl)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name omni_driver-msg:<TeleopControl> is deprecated: use omni_driver-msg:TeleopControl instead.")))

(cl:ensure-generic-function 'vel_joint-val :lambda-list '(m))
(cl:defmethod vel_joint-val ((m <TeleopControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader omni_driver-msg:vel_joint-val is deprecated.  Use omni_driver-msg:vel_joint instead.")
  (vel_joint m))

(cl:ensure-generic-function 'vel_effector-val :lambda-list '(m))
(cl:defmethod vel_effector-val ((m <TeleopControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader omni_driver-msg:vel_effector-val is deprecated.  Use omni_driver-msg:vel_effector instead.")
  (vel_effector m))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <TeleopControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader omni_driver-msg:mode-val is deprecated.  Use omni_driver-msg:mode instead.")
  (mode m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TeleopControl>) ostream)
  "Serializes a message object of type '<TeleopControl>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'vel_joint))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'vel_joint))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'vel_effector))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'vel_effector))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TeleopControl>) istream)
  "Deserializes a message object of type '<TeleopControl>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'vel_joint) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'vel_joint)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'vel_effector) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'vel_effector)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TeleopControl>)))
  "Returns string type for a message object of type '<TeleopControl>"
  "omni_driver/TeleopControl")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TeleopControl)))
  "Returns string type for a message object of type 'TeleopControl"
  "omni_driver/TeleopControl")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TeleopControl>)))
  "Returns md5sum for a message object of type '<TeleopControl>"
  "acc00fe3a847581a668e7968e27d235a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TeleopControl)))
  "Returns md5sum for a message object of type 'TeleopControl"
  "acc00fe3a847581a668e7968e27d235a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TeleopControl>)))
  "Returns full string definition for message of type '<TeleopControl>"
  (cl:format cl:nil "float64[] vel_joint~%float64[] vel_effector~%uint8 mode~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TeleopControl)))
  "Returns full string definition for message of type 'TeleopControl"
  (cl:format cl:nil "float64[] vel_joint~%float64[] vel_effector~%uint8 mode~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TeleopControl>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'vel_joint) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'vel_effector) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TeleopControl>))
  "Converts a ROS message object to a list"
  (cl:list 'TeleopControl
    (cl:cons ':vel_joint (vel_joint msg))
    (cl:cons ':vel_effector (vel_effector msg))
    (cl:cons ':mode (mode msg))
))
