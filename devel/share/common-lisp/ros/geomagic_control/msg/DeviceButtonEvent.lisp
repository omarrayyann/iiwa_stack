; Auto-generated. Do not edit!


(cl:in-package geomagic_control-msg)


;//! \htmlinclude DeviceButtonEvent.msg.html

(cl:defclass <DeviceButtonEvent> (roslisp-msg-protocol:ros-message)
  ((grey_button
    :reader grey_button
    :initarg :grey_button
    :type cl:integer
    :initform 0)
   (white_button
    :reader white_button
    :initarg :white_button
    :type cl:integer
    :initform 0))
)

(cl:defclass DeviceButtonEvent (<DeviceButtonEvent>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DeviceButtonEvent>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DeviceButtonEvent)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name geomagic_control-msg:<DeviceButtonEvent> is deprecated: use geomagic_control-msg:DeviceButtonEvent instead.")))

(cl:ensure-generic-function 'grey_button-val :lambda-list '(m))
(cl:defmethod grey_button-val ((m <DeviceButtonEvent>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader geomagic_control-msg:grey_button-val is deprecated.  Use geomagic_control-msg:grey_button instead.")
  (grey_button m))

(cl:ensure-generic-function 'white_button-val :lambda-list '(m))
(cl:defmethod white_button-val ((m <DeviceButtonEvent>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader geomagic_control-msg:white_button-val is deprecated.  Use geomagic_control-msg:white_button instead.")
  (white_button m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DeviceButtonEvent>) ostream)
  "Serializes a message object of type '<DeviceButtonEvent>"
  (cl:let* ((signed (cl:slot-value msg 'grey_button)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'white_button)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DeviceButtonEvent>) istream)
  "Deserializes a message object of type '<DeviceButtonEvent>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'grey_button) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'white_button) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DeviceButtonEvent>)))
  "Returns string type for a message object of type '<DeviceButtonEvent>"
  "geomagic_control/DeviceButtonEvent")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DeviceButtonEvent)))
  "Returns string type for a message object of type 'DeviceButtonEvent"
  "geomagic_control/DeviceButtonEvent")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DeviceButtonEvent>)))
  "Returns md5sum for a message object of type '<DeviceButtonEvent>"
  "fb77877e6b639935c3360838062f05f0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DeviceButtonEvent)))
  "Returns md5sum for a message object of type 'DeviceButtonEvent"
  "fb77877e6b639935c3360838062f05f0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DeviceButtonEvent>)))
  "Returns full string definition for message of type '<DeviceButtonEvent>"
  (cl:format cl:nil "int32 grey_button~%int32 white_button~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DeviceButtonEvent)))
  "Returns full string definition for message of type 'DeviceButtonEvent"
  (cl:format cl:nil "int32 grey_button~%int32 white_button~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DeviceButtonEvent>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DeviceButtonEvent>))
  "Converts a ROS message object to a list"
  (cl:list 'DeviceButtonEvent
    (cl:cons ':grey_button (grey_button msg))
    (cl:cons ':white_button (white_button msg))
))
