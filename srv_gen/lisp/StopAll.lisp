; Auto-generated. Do not edit!


(cl:in-package apriltags-srv)


;//! \htmlinclude StopAll-request.msg.html

(cl:defclass <StopAll-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass StopAll-request (<StopAll-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StopAll-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StopAll-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name apriltags-srv:<StopAll-request> is deprecated: use apriltags-srv:StopAll-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StopAll-request>) ostream)
  "Serializes a message object of type '<StopAll-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StopAll-request>) istream)
  "Deserializes a message object of type '<StopAll-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StopAll-request>)))
  "Returns string type for a service object of type '<StopAll-request>"
  "apriltags/StopAllRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StopAll-request)))
  "Returns string type for a service object of type 'StopAll-request"
  "apriltags/StopAllRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StopAll-request>)))
  "Returns md5sum for a message object of type '<StopAll-request>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StopAll-request)))
  "Returns md5sum for a message object of type 'StopAll-request"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StopAll-request>)))
  "Returns full string definition for message of type '<StopAll-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StopAll-request)))
  "Returns full string definition for message of type 'StopAll-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StopAll-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StopAll-request>))
  "Converts a ROS message object to a list"
  (cl:list 'StopAll-request
))
;//! \htmlinclude StopAll-response.msg.html

(cl:defclass <StopAll-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass StopAll-response (<StopAll-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StopAll-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StopAll-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name apriltags-srv:<StopAll-response> is deprecated: use apriltags-srv:StopAll-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StopAll-response>) ostream)
  "Serializes a message object of type '<StopAll-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StopAll-response>) istream)
  "Deserializes a message object of type '<StopAll-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StopAll-response>)))
  "Returns string type for a service object of type '<StopAll-response>"
  "apriltags/StopAllResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StopAll-response)))
  "Returns string type for a service object of type 'StopAll-response"
  "apriltags/StopAllResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StopAll-response>)))
  "Returns md5sum for a message object of type '<StopAll-response>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StopAll-response)))
  "Returns md5sum for a message object of type 'StopAll-response"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StopAll-response>)))
  "Returns full string definition for message of type '<StopAll-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StopAll-response)))
  "Returns full string definition for message of type 'StopAll-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StopAll-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StopAll-response>))
  "Converts a ROS message object to a list"
  (cl:list 'StopAll-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'StopAll)))
  'StopAll-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'StopAll)))
  'StopAll-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StopAll)))
  "Returns string type for a service object of type '<StopAll>"
  "apriltags/StopAll")