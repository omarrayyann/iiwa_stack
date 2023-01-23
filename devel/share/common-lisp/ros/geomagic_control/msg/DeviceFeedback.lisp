; Auto-generated. Do not edit!


(cl:in-package geomagic_control-msg)


;//! \htmlinclude DeviceFeedback.msg.html

(cl:defclass <DeviceFeedback> (roslisp-msg-protocol:ros-message)
  ((force
    :reader force
    :initarg :force
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (position
    :reader position
    :initarg :position
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (lock
    :reader lock
    :initarg :lock
    :type (cl:vector cl:boolean)
   :initform (cl:make-array 0 :element-type 'cl:boolean :initial-element cl:nil)))
)

(cl:defclass DeviceFeedback (<DeviceFeedback>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DeviceFeedback>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DeviceFeedback)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name geomagic_control-msg:<DeviceFeedback> is deprecated: use geomagic_control-msg:DeviceFeedback instead.")))

(cl:ensure-generic-function 'force-val :lambda-list '(m))
(cl:defmethod force-val ((m <DeviceFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader geomagic_control-msg:force-val is deprecated.  Use geomagic_control-msg:force instead.")
  (force m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <DeviceFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader geomagic_control-msg:position-val is deprecated.  Use geomagic_control-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'lock-val :lambda-list '(m))
(cl:defmethod lock-val ((m <DeviceFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader geomagic_control-msg:lock-val is deprecated.  Use geomagic_control-msg:lock instead.")
  (lock m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DeviceFeedback>) ostream)
  "Serializes a message object of type '<DeviceFeedback>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'force) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'lock))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if ele 1 0)) ostream))
   (cl:slot-value msg 'lock))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DeviceFeedback>) istream)
  "Deserializes a message object of type '<DeviceFeedback>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'force) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'lock) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'lock)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:not (cl:zerop (cl:read-byte istream)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DeviceFeedback>)))
  "Returns string type for a message object of type '<DeviceFeedback>"
  "geomagic_control/DeviceFeedback")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DeviceFeedback)))
  "Returns string type for a message object of type 'DeviceFeedback"
  "geomagic_control/DeviceFeedback")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DeviceFeedback>)))
  "Returns md5sum for a message object of type '<DeviceFeedback>"
  "94755b031db27fe99c2c235c0da9b072")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DeviceFeedback)))
  "Returns md5sum for a message object of type 'DeviceFeedback"
  "94755b031db27fe99c2c235c0da9b072")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DeviceFeedback>)))
  "Returns full string definition for message of type '<DeviceFeedback>"
  (cl:format cl:nil "# This is the force as estimated from the applied torques as well as the current~%# end effector position of the robot arm                           ~%geometry_msgs/Vector3  force                                                                  ~%geometry_msgs/Vector3  position ~%bool[] lock ~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DeviceFeedback)))
  "Returns full string definition for message of type 'DeviceFeedback"
  (cl:format cl:nil "# This is the force as estimated from the applied torques as well as the current~%# end effector position of the robot arm                           ~%geometry_msgs/Vector3  force                                                                  ~%geometry_msgs/Vector3  position ~%bool[] lock ~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DeviceFeedback>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'force))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'lock) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DeviceFeedback>))
  "Converts a ROS message object to a list"
  (cl:list 'DeviceFeedback
    (cl:cons ':force (force msg))
    (cl:cons ':position (position msg))
    (cl:cons ':lock (lock msg))
))
