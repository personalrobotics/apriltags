; Auto-generated. Do not edit!


(cl:in-package apriltags-srv)


;//! \htmlinclude IsIdOpen-request.msg.html

(cl:defclass <IsIdOpen-request> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0))
)

(cl:defclass IsIdOpen-request (<IsIdOpen-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <IsIdOpen-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'IsIdOpen-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name apriltags-srv:<IsIdOpen-request> is deprecated: use apriltags-srv:IsIdOpen-request instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <IsIdOpen-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader apriltags-srv:id-val is deprecated.  Use apriltags-srv:id instead.")
  (id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <IsIdOpen-request>) ostream)
  "Serializes a message object of type '<IsIdOpen-request>"
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <IsIdOpen-request>) istream)
  "Deserializes a message object of type '<IsIdOpen-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<IsIdOpen-request>)))
  "Returns string type for a service object of type '<IsIdOpen-request>"
  "apriltags/IsIdOpenRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'IsIdOpen-request)))
  "Returns string type for a service object of type 'IsIdOpen-request"
  "apriltags/IsIdOpenRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<IsIdOpen-request>)))
  "Returns md5sum for a message object of type '<IsIdOpen-request>"
  "dde39d9247fd3f471837f724f55c0805")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'IsIdOpen-request)))
  "Returns md5sum for a message object of type 'IsIdOpen-request"
  "dde39d9247fd3f471837f724f55c0805")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<IsIdOpen-request>)))
  "Returns full string definition for message of type '<IsIdOpen-request>"
  (cl:format cl:nil "int32 id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'IsIdOpen-request)))
  "Returns full string definition for message of type 'IsIdOpen-request"
  (cl:format cl:nil "int32 id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <IsIdOpen-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <IsIdOpen-request>))
  "Converts a ROS message object to a list"
  (cl:list 'IsIdOpen-request
    (cl:cons ':id (id msg))
))
;//! \htmlinclude IsIdOpen-response.msg.html

(cl:defclass <IsIdOpen-response> (roslisp-msg-protocol:ros-message)
  ((open
    :reader open
    :initarg :open
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass IsIdOpen-response (<IsIdOpen-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <IsIdOpen-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'IsIdOpen-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name apriltags-srv:<IsIdOpen-response> is deprecated: use apriltags-srv:IsIdOpen-response instead.")))

(cl:ensure-generic-function 'open-val :lambda-list '(m))
(cl:defmethod open-val ((m <IsIdOpen-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader apriltags-srv:open-val is deprecated.  Use apriltags-srv:open instead.")
  (open m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <IsIdOpen-response>) ostream)
  "Serializes a message object of type '<IsIdOpen-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'open) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <IsIdOpen-response>) istream)
  "Deserializes a message object of type '<IsIdOpen-response>"
    (cl:setf (cl:slot-value msg 'open) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<IsIdOpen-response>)))
  "Returns string type for a service object of type '<IsIdOpen-response>"
  "apriltags/IsIdOpenResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'IsIdOpen-response)))
  "Returns string type for a service object of type 'IsIdOpen-response"
  "apriltags/IsIdOpenResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<IsIdOpen-response>)))
  "Returns md5sum for a message object of type '<IsIdOpen-response>"
  "dde39d9247fd3f471837f724f55c0805")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'IsIdOpen-response)))
  "Returns md5sum for a message object of type 'IsIdOpen-response"
  "dde39d9247fd3f471837f724f55c0805")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<IsIdOpen-response>)))
  "Returns full string definition for message of type '<IsIdOpen-response>"
  (cl:format cl:nil "bool open~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'IsIdOpen-response)))
  "Returns full string definition for message of type 'IsIdOpen-response"
  (cl:format cl:nil "bool open~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <IsIdOpen-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <IsIdOpen-response>))
  "Converts a ROS message object to a list"
  (cl:list 'IsIdOpen-response
    (cl:cons ':open (open msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'IsIdOpen)))
  'IsIdOpen-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'IsIdOpen)))
  'IsIdOpen-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'IsIdOpen)))
  "Returns string type for a service object of type '<IsIdOpen>"
  "apriltags/IsIdOpen")