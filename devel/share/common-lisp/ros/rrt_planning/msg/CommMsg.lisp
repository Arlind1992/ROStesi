; Auto-generated. Do not edit!


(cl:in-package rrt_planning-msg)


;//! \htmlinclude CommMsg.msg.html

(cl:defclass <CommMsg> (roslisp-msg-protocol:ros-message)
  ((cellX
    :reader cellX
    :initarg :cellX
    :type cl:integer
    :initform 0)
   (cellY
    :reader cellY
    :initarg :cellY
    :type cl:integer
    :initform 0)
   (uploadTime
    :reader uploadTime
    :initarg :uploadTime
    :type cl:integer
    :initform 0))
)

(cl:defclass CommMsg (<CommMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CommMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CommMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rrt_planning-msg:<CommMsg> is deprecated: use rrt_planning-msg:CommMsg instead.")))

(cl:ensure-generic-function 'cellX-val :lambda-list '(m))
(cl:defmethod cellX-val ((m <CommMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rrt_planning-msg:cellX-val is deprecated.  Use rrt_planning-msg:cellX instead.")
  (cellX m))

(cl:ensure-generic-function 'cellY-val :lambda-list '(m))
(cl:defmethod cellY-val ((m <CommMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rrt_planning-msg:cellY-val is deprecated.  Use rrt_planning-msg:cellY instead.")
  (cellY m))

(cl:ensure-generic-function 'uploadTime-val :lambda-list '(m))
(cl:defmethod uploadTime-val ((m <CommMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rrt_planning-msg:uploadTime-val is deprecated.  Use rrt_planning-msg:uploadTime instead.")
  (uploadTime m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CommMsg>) ostream)
  "Serializes a message object of type '<CommMsg>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cellX)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'cellX)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'cellX)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'cellX)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cellY)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'cellY)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'cellY)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'cellY)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'uploadTime)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'uploadTime)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'uploadTime)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'uploadTime)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CommMsg>) istream)
  "Deserializes a message object of type '<CommMsg>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cellX)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'cellX)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'cellX)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'cellX)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cellY)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'cellY)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'cellY)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'cellY)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'uploadTime)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'uploadTime)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'uploadTime)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'uploadTime)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CommMsg>)))
  "Returns string type for a message object of type '<CommMsg>"
  "rrt_planning/CommMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CommMsg)))
  "Returns string type for a message object of type 'CommMsg"
  "rrt_planning/CommMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CommMsg>)))
  "Returns md5sum for a message object of type '<CommMsg>"
  "2ae8e81eb815f3fe2d8493f75b85e1c9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CommMsg)))
  "Returns md5sum for a message object of type 'CommMsg"
  "2ae8e81eb815f3fe2d8493f75b85e1c9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CommMsg>)))
  "Returns full string definition for message of type '<CommMsg>"
  (cl:format cl:nil "~%uint32  cellX	~%uint32  cellY~%uint32 uploadTime~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CommMsg)))
  "Returns full string definition for message of type 'CommMsg"
  (cl:format cl:nil "~%uint32  cellX	~%uint32  cellY~%uint32 uploadTime~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CommMsg>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CommMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'CommMsg
    (cl:cons ':cellX (cellX msg))
    (cl:cons ':cellY (cellY msg))
    (cl:cons ':uploadTime (uploadTime msg))
))
