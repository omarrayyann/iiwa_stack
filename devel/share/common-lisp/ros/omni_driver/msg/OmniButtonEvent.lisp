; Auto-generated. Do not edit!


(cl:in-package omni_driver-msg)


;//! \htmlinclude OmniButtonEvent.msg.html

(cl:defclass <OmniButtonEvent> (roslisp-msg-protocol:ros-message)
  ((grey_button
    :reader grey_button
    :initarg :grey_button
    :type cl:boolean
    :initform cl:nil)
   (white_button
    :reader white_button
    :initarg :white_button
    :type cl:boolean
    :initform cl:nil)
   (grey_button_clicked
    :reader grey_button_clicked
    :initarg :grey_button_clicked
    :type cl:boolean
    :initform cl:nil)
   (white_button_clicked
    :reader white_button_clicked
    :initarg :white_button_clicked
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass OmniButtonEvent (<OmniButtonEvent>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OmniButtonEvent>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OmniButtonEvent)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name omni_driver-msg:<OmniButtonEvent> is deprecated: use omni_driver-msg:OmniButtonEvent instead.")))

(cl:ensure-generic-function 'grey_button-val :lambda-list '(m))
(cl:defmethod grey_button-val ((m <OmniButtonEvent>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader omni_driver-msg:grey_button-val is deprecated.  Use omni_driver-msg:grey_button instead.")
  (grey_button m))

(cl:ensure-generic-function 'white_button-val :lambda-list '(m))
(cl:defmethod white_button-val ((m <OmniButtonEvent>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader omni_driver-msg:white_button-val is deprecated.  Use omni_driver-msg:white_button instead.")
  (white_button m))

(cl:ensure-generic-function 'grey_button_clicked-val :lambda-list '(m))
(cl:defmethod grey_button_clicked-val ((m <OmniButtonEvent>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader omni_driver-msg:grey_button_clicked-val is deprecated.  Use omni_driver-msg:grey_button_clicked instead.")
  (grey_button_clicked m))

(cl:ensure-generic-function 'white_button_clicked-val :lambda-list '(m))
(cl:defmethod white_button_clicked-val ((m <OmniButtonEvent>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader omni_driver-msg:white_button_clicked-val is deprecated.  Use omni_driver-msg:white_button_clicked instead.")
  (white_button_clicked m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OmniButtonEvent>) ostream)
  "Serializes a message object of type '<OmniButtonEvent>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'grey_button) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'white_button) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'grey_button_clicked) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'white_button_clicked) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OmniButtonEvent>) istream)
  "Deserializes a message object of type '<OmniButtonEvent>"
    (cl:setf (cl:slot-value msg 'grey_button) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'white_button) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'grey_button_clicked) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'white_button_clicked) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OmniButtonEvent>)))
  "Returns string type for a message object of type '<OmniButtonEvent>"
  "omni_driver/OmniButtonEvent")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OmniButtonEvent)))
  "Returns string type for a message object of type 'OmniButtonEvent"
  "omni_driver/OmniButtonEvent")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OmniButtonEvent>)))
  "Returns md5sum for a message object of type '<OmniButtonEvent>"
  "67ad876972cf46727cadcf266dde0b5c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OmniButtonEvent)))
  "Returns md5sum for a message object of type 'OmniButtonEvent"
  "67ad876972cf46727cadcf266dde0b5c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OmniButtonEvent>)))
  "Returns full string definition for message of type '<OmniButtonEvent>"
  (cl:format cl:nil "bool grey_button~%bool white_button~%bool grey_button_clicked~%bool white_button_clicked~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OmniButtonEvent)))
  "Returns full string definition for message of type 'OmniButtonEvent"
  (cl:format cl:nil "bool grey_button~%bool white_button~%bool grey_button_clicked~%bool white_button_clicked~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OmniButtonEvent>))
  (cl:+ 0
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OmniButtonEvent>))
  "Converts a ROS message object to a list"
  (cl:list 'OmniButtonEvent
    (cl:cons ':grey_button (grey_button msg))
    (cl:cons ':white_button (white_button msg))
    (cl:cons ':grey_button_clicked (grey_button_clicked msg))
    (cl:cons ':white_button_clicked (white_button_clicked msg))
))
