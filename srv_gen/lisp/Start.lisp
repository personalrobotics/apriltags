; Auto-generated. Do not edit!


(cl:in-package apriltags-srv)


;//! \htmlinclude Start-request.msg.html

(cl:defclass <Start-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Start-request (<Start-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Start-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Start-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name apriltags-srv:<Start-request> is deprecated: use apriltags-srv:Start-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Start-request>) ostream)
  "Serializes a message object of type '<Start-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Start-request>) istream)
  "Deserializes a message object of type '<Start-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Start-request>)))
  "Returns string type for a service object of type '<Start-request>"
  "apriltags/StartRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Start-request)))
  "Returns string type for a service object of type 'Start-request"
  "apriltags/StartRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Start-request>)))
  "Returns md5sum for a message object of type '<Start-request>"
  "c5e4a7d59c68f74eabcec876a00216aa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Start-request)))
  "Returns md5sum for a message object of type 'Start-request"
  "c5e4a7d59c68f74eabcec876a00216aa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Start-request>)))
  "Returns full string definition for message of type '<Start-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Start-request)))
  "Returns full string definition for message of type 'Start-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Start-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Start-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Start-request
))
;//! \htmlinclude Start-response.msg.html

(cl:defclass <Start-response> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0))
)

(cl:defclass Start-response (<Start-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Start-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Start-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name apriltags-srv:<Start-response> is deprecated: use apriltags-srv:Start-response instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <Start-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader apriltags-srv:id-val is deprecated.  Use apriltags-srv:id instead.")
  (id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Start-response>) ostream)
  "Serializes a message object of type '<Start-response>"
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Start-response>) istream)
  "Deserializes a message object of type '<Start-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Start-response>)))
  "Returns string type for a service object of type '<Start-response>"
  "apriltags/StartResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Start-response)))
  "Returns string type for a service object of type 'Start-response"
  "apriltags/StartResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Start-response>)))
  "Returns md5sum for a message object of type '<Start-response>"
  "c5e4a7d59c68f74eabcec876a00216aa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Start-response)))
  "Returns md5sum for a message object of type 'Start-response"
  "c5e4a7d59c68f74eabcec876a00216aa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Start-response>)))
  "Returns full string definition for message of type '<Start-response>"
  (cl:format cl:nil "int32 id~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Start-response)))
  "Returns full string definition for message of type 'Start-response"
  (cl:format cl:nil "int32 id~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Start-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Start-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Start-response
    (cl:cons ':id (id msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Start)))
  'Start-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Start)))
  'Start-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Start)))
  "Returns string type for a service object of type '<Start>"
  "apriltags/Start")