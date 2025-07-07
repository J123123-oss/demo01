; Auto-generated. Do not edit!


(cl:in-package serial_comms-msg)


;//! \htmlinclude Distances.msg.html

(cl:defclass <Distances> (roslisp-msg-protocol:ros-message)
  ((distance_a
    :reader distance_a
    :initarg :distance_a
    :type cl:integer
    :initform 0)
   (distance_b
    :reader distance_b
    :initarg :distance_b
    :type cl:integer
    :initform 0)
   (distance_c
    :reader distance_c
    :initarg :distance_c
    :type cl:integer
    :initform 0)
   (distance_d
    :reader distance_d
    :initarg :distance_d
    :type cl:integer
    :initform 0)
   (distance_e
    :reader distance_e
    :initarg :distance_e
    :type cl:integer
    :initform 0)
   (distance_f
    :reader distance_f
    :initarg :distance_f
    :type cl:integer
    :initform 0))
)

(cl:defclass Distances (<Distances>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Distances>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Distances)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name serial_comms-msg:<Distances> is deprecated: use serial_comms-msg:Distances instead.")))

(cl:ensure-generic-function 'distance_a-val :lambda-list '(m))
(cl:defmethod distance_a-val ((m <Distances>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader serial_comms-msg:distance_a-val is deprecated.  Use serial_comms-msg:distance_a instead.")
  (distance_a m))

(cl:ensure-generic-function 'distance_b-val :lambda-list '(m))
(cl:defmethod distance_b-val ((m <Distances>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader serial_comms-msg:distance_b-val is deprecated.  Use serial_comms-msg:distance_b instead.")
  (distance_b m))

(cl:ensure-generic-function 'distance_c-val :lambda-list '(m))
(cl:defmethod distance_c-val ((m <Distances>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader serial_comms-msg:distance_c-val is deprecated.  Use serial_comms-msg:distance_c instead.")
  (distance_c m))

(cl:ensure-generic-function 'distance_d-val :lambda-list '(m))
(cl:defmethod distance_d-val ((m <Distances>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader serial_comms-msg:distance_d-val is deprecated.  Use serial_comms-msg:distance_d instead.")
  (distance_d m))

(cl:ensure-generic-function 'distance_e-val :lambda-list '(m))
(cl:defmethod distance_e-val ((m <Distances>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader serial_comms-msg:distance_e-val is deprecated.  Use serial_comms-msg:distance_e instead.")
  (distance_e m))

(cl:ensure-generic-function 'distance_f-val :lambda-list '(m))
(cl:defmethod distance_f-val ((m <Distances>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader serial_comms-msg:distance_f-val is deprecated.  Use serial_comms-msg:distance_f instead.")
  (distance_f m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Distances>) ostream)
  "Serializes a message object of type '<Distances>"
  (cl:let* ((signed (cl:slot-value msg 'distance_a)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'distance_b)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'distance_c)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'distance_d)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'distance_e)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'distance_f)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Distances>) istream)
  "Deserializes a message object of type '<Distances>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'distance_a) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'distance_b) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'distance_c) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'distance_d) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'distance_e) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'distance_f) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Distances>)))
  "Returns string type for a message object of type '<Distances>"
  "serial_comms/Distances")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Distances)))
  "Returns string type for a message object of type 'Distances"
  "serial_comms/Distances")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Distances>)))
  "Returns md5sum for a message object of type '<Distances>"
  "88930f00743784851f3f6d92c51802bd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Distances)))
  "Returns md5sum for a message object of type 'Distances"
  "88930f00743784851f3f6d92c51802bd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Distances>)))
  "Returns full string definition for message of type '<Distances>"
  (cl:format cl:nil "# Defines a message for holding the six distance values.~%int32 distance_a~%int32 distance_b~%int32 distance_c~%int32 distance_d~%int32 distance_e~%int32 distance_f~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Distances)))
  "Returns full string definition for message of type 'Distances"
  (cl:format cl:nil "# Defines a message for holding the six distance values.~%int32 distance_a~%int32 distance_b~%int32 distance_c~%int32 distance_d~%int32 distance_e~%int32 distance_f~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Distances>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Distances>))
  "Converts a ROS message object to a list"
  (cl:list 'Distances
    (cl:cons ':distance_a (distance_a msg))
    (cl:cons ':distance_b (distance_b msg))
    (cl:cons ':distance_c (distance_c msg))
    (cl:cons ':distance_d (distance_d msg))
    (cl:cons ':distance_e (distance_e msg))
    (cl:cons ':distance_f (distance_f msg))
))
