;; Auto-generated. Do not edit!


(when (boundp 'ucar_commander::nav_start)
  (if (not (find-package "UCAR_COMMANDER"))
    (make-package "UCAR_COMMANDER"))
  (shadow 'nav_start (find-package "UCAR_COMMANDER")))
(unless (find-package "UCAR_COMMANDER::NAV_START")
  (make-package "UCAR_COMMANDER::NAV_START"))
(unless (find-package "UCAR_COMMANDER::NAV_STARTREQUEST")
  (make-package "UCAR_COMMANDER::NAV_STARTREQUEST"))
(unless (find-package "UCAR_COMMANDER::NAV_STARTRESPONSE")
  (make-package "UCAR_COMMANDER::NAV_STARTRESPONSE"))

(in-package "ROS")





(defclass ucar_commander::nav_startRequest
  :super ros::object
  :slots (_data ))

(defmethod ucar_commander::nav_startRequest
  (:init
   (&key
    ((:data __data) 0)
    )
   (send-super :init)
   (setq _data (round __data))
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; int8 _data
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _data
       (write-byte _data s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _data
     (setq _data (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _data 127) (setq _data (- _data 256)))
   ;;
   self)
  )

(defclass ucar_commander::nav_startResponse
  :super ros::object
  :slots (_success _message ))

(defmethod ucar_commander::nav_startResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:message __message) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _message (string __message))
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass ucar_commander::nav_start
  :super ros::object
  :slots ())

(setf (get ucar_commander::nav_start :md5sum-) "208d4ab53cf12eb941a7d93b34b7532b")
(setf (get ucar_commander::nav_start :datatype-) "ucar_commander/nav_start")
(setf (get ucar_commander::nav_start :request) ucar_commander::nav_startRequest)
(setf (get ucar_commander::nav_start :response) ucar_commander::nav_startResponse)

(defmethod ucar_commander::nav_startRequest
  (:response () (instance ucar_commander::nav_startResponse :init)))

(setf (get ucar_commander::nav_startRequest :md5sum-) "208d4ab53cf12eb941a7d93b34b7532b")
(setf (get ucar_commander::nav_startRequest :datatype-) "ucar_commander/nav_startRequest")
(setf (get ucar_commander::nav_startRequest :definition-)
      "# request
int8 data
---
# response
bool success
string message
")

(setf (get ucar_commander::nav_startResponse :md5sum-) "208d4ab53cf12eb941a7d93b34b7532b")
(setf (get ucar_commander::nav_startResponse :datatype-) "ucar_commander/nav_startResponse")
(setf (get ucar_commander::nav_startResponse :definition-)
      "# request
int8 data
---
# response
bool success
string message
")



(provide :ucar_commander/nav_start "208d4ab53cf12eb941a7d93b34b7532b")


