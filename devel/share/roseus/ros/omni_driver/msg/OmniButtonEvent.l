;; Auto-generated. Do not edit!


(when (boundp 'omni_driver::OmniButtonEvent)
  (if (not (find-package "OMNI_DRIVER"))
    (make-package "OMNI_DRIVER"))
  (shadow 'OmniButtonEvent (find-package "OMNI_DRIVER")))
(unless (find-package "OMNI_DRIVER::OMNIBUTTONEVENT")
  (make-package "OMNI_DRIVER::OMNIBUTTONEVENT"))

(in-package "ROS")
;;//! \htmlinclude OmniButtonEvent.msg.html


(defclass omni_driver::OmniButtonEvent
  :super ros::object
  :slots (_grey_button _white_button _grey_button_clicked _white_button_clicked ))

(defmethod omni_driver::OmniButtonEvent
  (:init
   (&key
    ((:grey_button __grey_button) nil)
    ((:white_button __white_button) nil)
    ((:grey_button_clicked __grey_button_clicked) nil)
    ((:white_button_clicked __white_button_clicked) nil)
    )
   (send-super :init)
   (setq _grey_button __grey_button)
   (setq _white_button __white_button)
   (setq _grey_button_clicked __grey_button_clicked)
   (setq _white_button_clicked __white_button_clicked)
   self)
  (:grey_button
   (&optional (__grey_button :null))
   (if (not (eq __grey_button :null)) (setq _grey_button __grey_button)) _grey_button)
  (:white_button
   (&optional (__white_button :null))
   (if (not (eq __white_button :null)) (setq _white_button __white_button)) _white_button)
  (:grey_button_clicked
   (&optional (__grey_button_clicked :null))
   (if (not (eq __grey_button_clicked :null)) (setq _grey_button_clicked __grey_button_clicked)) _grey_button_clicked)
  (:white_button_clicked
   (&optional (__white_button_clicked :null))
   (if (not (eq __white_button_clicked :null)) (setq _white_button_clicked __white_button_clicked)) _white_button_clicked)
  (:serialization-length
   ()
   (+
    ;; bool _grey_button
    1
    ;; bool _white_button
    1
    ;; bool _grey_button_clicked
    1
    ;; bool _white_button_clicked
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _grey_button
       (if _grey_button (write-byte -1 s) (write-byte 0 s))
     ;; bool _white_button
       (if _white_button (write-byte -1 s) (write-byte 0 s))
     ;; bool _grey_button_clicked
       (if _grey_button_clicked (write-byte -1 s) (write-byte 0 s))
     ;; bool _white_button_clicked
       (if _white_button_clicked (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _grey_button
     (setq _grey_button (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _white_button
     (setq _white_button (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _grey_button_clicked
     (setq _grey_button_clicked (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _white_button_clicked
     (setq _white_button_clicked (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get omni_driver::OmniButtonEvent :md5sum-) "67ad876972cf46727cadcf266dde0b5c")
(setf (get omni_driver::OmniButtonEvent :datatype-) "omni_driver/OmniButtonEvent")
(setf (get omni_driver::OmniButtonEvent :definition-)
      "bool grey_button
bool white_button
bool grey_button_clicked
bool white_button_clicked

")



(provide :omni_driver/OmniButtonEvent "67ad876972cf46727cadcf266dde0b5c")


