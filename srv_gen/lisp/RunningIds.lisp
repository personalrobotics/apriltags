; Auto-generated. Do not edit!


(cl:in-package apriltags-srv)


;//! \htmlinclude RunningIds-request.msg.html

(cl:defclass <RunningIds-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass RunningIds-request (<RunningIds-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RunningIds-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RunningIds-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name apriltags-srv:<RunningIds-request> is deprecated: use apriltags-srv:RunningIds-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RunningIds-request>) ostream)
  "Serializes a message object of type '<RunningIds-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RunningIds-request>) istream)
  "Deserializes a message object of type '<RunningIds-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RunningIds-request>)))
  "Returns string type for a service object of type '<RunningIds-request>"
  "apriltags/RunningIdsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RunningIds-request)))
  "Returns string type for a service object of type 'RunningIds-request"
  "apriltags/RunningIdsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RunningIds-request>)))
  "Returns md5sum for a message object of type '<RunningIds-request>"
  "4f22efebf407aadba2ecc69df353d113")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RunningIds-request)))
  "Returns md5sum for a message object of type 'RunningIds-request"
  "4f22efebf407aadba2ecc69df353d113")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RunningIds-request>)))
  "Returns full string definition for message of type '<RunningIds-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RunningIds-request)))
  "Returns full string definition for message of type 'RunningIds-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RunningIds-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RunningIds-request>))
  "Converts a ROS message object to a list"
  (cl:list 'RunningIds-request
))
;//! \htmlinclude RunningIds-response.msg.html

(cl:defclass <RunningIds-response> (roslisp-msg-protocol:ros-message)
  ((ids
    :reader ids
    :initarg :ids
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass RunningIds-response (<RunningIds-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RunningIds-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RunningIds-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name apriltags-srv:<RunningIds-response> is deprecated: use apriltags-srv:RunningIds-response instead.")))

(cl:ensure-generic-function 'ids-val :lambda-list '(m))
(cl:defmethod ids-val ((m <RunningIds-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader apriltags-srv:ids-val is deprecated.  Use apriltags-srv:ids instead.")
  (ids m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RunningIds-response>) ostream)
  "Serializes a message object of type '<RunningIds-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'ids))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'ids))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RunningIds-response>) istream)
  "Deserializes a message object of type '<RunningIds-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'ids) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'ids)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RunningIds-response>)))
  "Returns string type for a service object of type '<RunningIds-response>"
  "apriltags/RunningIdsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RunningIds-response)))
  "Returns string type for a service object of type 'RunningIds-response"
  "apriltags/RunningIdsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RunningIds-response>)))
  "Returns md5sum for a message object of type '<RunningIds-response>"
  "4f22efebf407aadba2ecc69df353d113")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RunningIds-response)))
  "Returns md5sum for a message object of type 'RunningIds-response"
  "4f22efebf407aadba2ecc69df353d113")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RunningIds-response>)))
  "Returns full string definition for message of type '<RunningIds-response>"
  (cl:format cl:nil "int32[] ids~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RunningIds-response)))
  "Returns full string definition for message of type 'RunningIds-response"
  (cl:format cl:nil "int32[] ids~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RunningIds-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'ids) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RunningIds-response>))
  "Converts a ROS message object to a list"
  (cl:list 'RunningIds-response
    (cl:cons ':ids (ids msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'RunningIds)))
  'RunningIds-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'RunningIds)))
  'RunningIds-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RunningIds)))
  "Returns string type for a service object of type '<RunningIds>"
  "apriltags/RunningIds")