; Auto-generated. Do not edit!


(cl:in-package ucar_commander-srv)


;//! \htmlinclude nav_start-request.msg.html

(cl:defclass <nav_start-request> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type cl:fixnum
    :initform 0))
)

(cl:defclass nav_start-request (<nav_start-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <nav_start-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'nav_start-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ucar_commander-srv:<nav_start-request> is deprecated: use ucar_commander-srv:nav_start-request instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <nav_start-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucar_commander-srv:data-val is deprecated.  Use ucar_commander-srv:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <nav_start-request>) ostream)
  "Serializes a message object of type '<nav_start-request>"
  (cl:let* ((signed (cl:slot-value msg 'data)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <nav_start-request>) istream)
  "Deserializes a message object of type '<nav_start-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'data) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<nav_start-request>)))
  "Returns string type for a service object of type '<nav_start-request>"
  "ucar_commander/nav_startRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'nav_start-request)))
  "Returns string type for a service object of type 'nav_start-request"
  "ucar_commander/nav_startRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<nav_start-request>)))
  "Returns md5sum for a message object of type '<nav_start-request>"
  "208d4ab53cf12eb941a7d93b34b7532b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'nav_start-request)))
  "Returns md5sum for a message object of type 'nav_start-request"
  "208d4ab53cf12eb941a7d93b34b7532b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<nav_start-request>)))
  "Returns full string definition for message of type '<nav_start-request>"
  (cl:format cl:nil "# request~%int8 data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'nav_start-request)))
  "Returns full string definition for message of type 'nav_start-request"
  (cl:format cl:nil "# request~%int8 data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <nav_start-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <nav_start-request>))
  "Converts a ROS message object to a list"
  (cl:list 'nav_start-request
    (cl:cons ':data (data msg))
))
;//! \htmlinclude nav_start-response.msg.html

(cl:defclass <nav_start-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (message
    :reader message
    :initarg :message
    :type cl:string
    :initform ""))
)

(cl:defclass nav_start-response (<nav_start-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <nav_start-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'nav_start-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ucar_commander-srv:<nav_start-response> is deprecated: use ucar_commander-srv:nav_start-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <nav_start-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucar_commander-srv:success-val is deprecated.  Use ucar_commander-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <nav_start-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucar_commander-srv:message-val is deprecated.  Use ucar_commander-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <nav_start-response>) ostream)
  "Serializes a message object of type '<nav_start-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <nav_start-response>) istream)
  "Deserializes a message object of type '<nav_start-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<nav_start-response>)))
  "Returns string type for a service object of type '<nav_start-response>"
  "ucar_commander/nav_startResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'nav_start-response)))
  "Returns string type for a service object of type 'nav_start-response"
  "ucar_commander/nav_startResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<nav_start-response>)))
  "Returns md5sum for a message object of type '<nav_start-response>"
  "208d4ab53cf12eb941a7d93b34b7532b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'nav_start-response)))
  "Returns md5sum for a message object of type 'nav_start-response"
  "208d4ab53cf12eb941a7d93b34b7532b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<nav_start-response>)))
  "Returns full string definition for message of type '<nav_start-response>"
  (cl:format cl:nil "# response~%bool success~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'nav_start-response)))
  "Returns full string definition for message of type 'nav_start-response"
  (cl:format cl:nil "# response~%bool success~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <nav_start-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <nav_start-response>))
  "Converts a ROS message object to a list"
  (cl:list 'nav_start-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'nav_start)))
  'nav_start-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'nav_start)))
  'nav_start-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'nav_start)))
  "Returns string type for a service object of type '<nav_start>"
  "ucar_commander/nav_start")