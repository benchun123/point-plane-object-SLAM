; Auto-generated. Do not edit!


(cl:in-package tictoc_profiler-msg)


;//! \htmlinclude ProfilerEntry.msg.html

(cl:defclass <ProfilerEntry> (roslisp-msg-protocol:ros-message)
  ((seq
    :reader seq
    :initarg :seq
    :type cl:integer
    :initform 0)
   (name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (start_time
    :reader start_time
    :initarg :start_time
    :type cl:integer
    :initform 0)
   (end_time
    :reader end_time
    :initarg :end_time
    :type cl:integer
    :initform 0)
   (delta_time_ms
    :reader delta_time_ms
    :initarg :delta_time_ms
    :type cl:float
    :initform 0.0))
)

(cl:defclass ProfilerEntry (<ProfilerEntry>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ProfilerEntry>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ProfilerEntry)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tictoc_profiler-msg:<ProfilerEntry> is deprecated: use tictoc_profiler-msg:ProfilerEntry instead.")))

(cl:ensure-generic-function 'seq-val :lambda-list '(m))
(cl:defmethod seq-val ((m <ProfilerEntry>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tictoc_profiler-msg:seq-val is deprecated.  Use tictoc_profiler-msg:seq instead.")
  (seq m))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <ProfilerEntry>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tictoc_profiler-msg:name-val is deprecated.  Use tictoc_profiler-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'start_time-val :lambda-list '(m))
(cl:defmethod start_time-val ((m <ProfilerEntry>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tictoc_profiler-msg:start_time-val is deprecated.  Use tictoc_profiler-msg:start_time instead.")
  (start_time m))

(cl:ensure-generic-function 'end_time-val :lambda-list '(m))
(cl:defmethod end_time-val ((m <ProfilerEntry>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tictoc_profiler-msg:end_time-val is deprecated.  Use tictoc_profiler-msg:end_time instead.")
  (end_time m))

(cl:ensure-generic-function 'delta_time_ms-val :lambda-list '(m))
(cl:defmethod delta_time_ms-val ((m <ProfilerEntry>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tictoc_profiler-msg:delta_time_ms-val is deprecated.  Use tictoc_profiler-msg:delta_time_ms instead.")
  (delta_time_ms m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ProfilerEntry>) ostream)
  "Serializes a message object of type '<ProfilerEntry>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'seq)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'seq)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'seq)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'seq)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:let* ((signed (cl:slot-value msg 'start_time)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'end_time)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'delta_time_ms))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ProfilerEntry>) istream)
  "Deserializes a message object of type '<ProfilerEntry>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'seq)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'seq)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'seq)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'seq)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'start_time) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'end_time) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'delta_time_ms) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ProfilerEntry>)))
  "Returns string type for a message object of type '<ProfilerEntry>"
  "tictoc_profiler/ProfilerEntry")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ProfilerEntry)))
  "Returns string type for a message object of type 'ProfilerEntry"
  "tictoc_profiler/ProfilerEntry")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ProfilerEntry>)))
  "Returns md5sum for a message object of type '<ProfilerEntry>"
  "bfc3f9f9968c9db2e3db18a9276d6e48")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ProfilerEntry)))
  "Returns md5sum for a message object of type 'ProfilerEntry"
  "bfc3f9f9968c9db2e3db18a9276d6e48")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ProfilerEntry>)))
  "Returns full string definition for message of type '<ProfilerEntry>"
  (cl:format cl:nil "uint32 seq~%string name~%int64 start_time~%int64 end_time~%float64 delta_time_ms~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ProfilerEntry)))
  "Returns full string definition for message of type 'ProfilerEntry"
  (cl:format cl:nil "uint32 seq~%string name~%int64 start_time~%int64 end_time~%float64 delta_time_ms~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ProfilerEntry>))
  (cl:+ 0
     4
     4 (cl:length (cl:slot-value msg 'name))
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ProfilerEntry>))
  "Converts a ROS message object to a list"
  (cl:list 'ProfilerEntry
    (cl:cons ':seq (seq msg))
    (cl:cons ':name (name msg))
    (cl:cons ':start_time (start_time msg))
    (cl:cons ':end_time (end_time msg))
    (cl:cons ':delta_time_ms (delta_time_ms msg))
))
