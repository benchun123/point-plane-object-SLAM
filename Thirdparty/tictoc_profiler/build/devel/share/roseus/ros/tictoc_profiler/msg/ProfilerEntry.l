;; Auto-generated. Do not edit!


(when (boundp 'tictoc_profiler::ProfilerEntry)
  (if (not (find-package "TICTOC_PROFILER"))
    (make-package "TICTOC_PROFILER"))
  (shadow 'ProfilerEntry (find-package "TICTOC_PROFILER")))
(unless (find-package "TICTOC_PROFILER::PROFILERENTRY")
  (make-package "TICTOC_PROFILER::PROFILERENTRY"))

(in-package "ROS")
;;//! \htmlinclude ProfilerEntry.msg.html


(defclass tictoc_profiler::ProfilerEntry
  :super ros::object
  :slots (_seq _name _start_time _end_time _delta_time_ms ))

(defmethod tictoc_profiler::ProfilerEntry
  (:init
   (&key
    ((:seq __seq) 0)
    ((:name __name) "")
    ((:start_time __start_time) 0)
    ((:end_time __end_time) 0)
    ((:delta_time_ms __delta_time_ms) 0.0)
    )
   (send-super :init)
   (setq _seq (round __seq))
   (setq _name (string __name))
   (setq _start_time (round __start_time))
   (setq _end_time (round __end_time))
   (setq _delta_time_ms (float __delta_time_ms))
   self)
  (:seq
   (&optional __seq)
   (if __seq (setq _seq __seq)) _seq)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:start_time
   (&optional __start_time)
   (if __start_time (setq _start_time __start_time)) _start_time)
  (:end_time
   (&optional __end_time)
   (if __end_time (setq _end_time __end_time)) _end_time)
  (:delta_time_ms
   (&optional __delta_time_ms)
   (if __delta_time_ms (setq _delta_time_ms __delta_time_ms)) _delta_time_ms)
  (:serialization-length
   ()
   (+
    ;; uint32 _seq
    4
    ;; string _name
    4 (length _name)
    ;; int64 _start_time
    8
    ;; int64 _end_time
    8
    ;; float64 _delta_time_ms
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _seq
       (write-long _seq s)
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; int64 _start_time
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _start_time (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _start_time) (= (length (_start_time . bv)) 2)) ;; bignum
              (write-long (ash (elt (_start_time . bv) 0) 0) s)
              (write-long (ash (elt (_start_time . bv) 1) -1) s))
             ((and (class _start_time) (= (length (_start_time . bv)) 1)) ;; big1
              (write-long (elt (_start_time . bv) 0) s)
              (write-long (if (>= _start_time 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _start_time s)(write-long (if (>= _start_time 0) 0 #xffffffff) s)))
     ;; int64 _end_time
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _end_time (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _end_time) (= (length (_end_time . bv)) 2)) ;; bignum
              (write-long (ash (elt (_end_time . bv) 0) 0) s)
              (write-long (ash (elt (_end_time . bv) 1) -1) s))
             ((and (class _end_time) (= (length (_end_time . bv)) 1)) ;; big1
              (write-long (elt (_end_time . bv) 0) s)
              (write-long (if (>= _end_time 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _end_time s)(write-long (if (>= _end_time 0) 0 #xffffffff) s)))
     ;; float64 _delta_time_ms
       (sys::poke _delta_time_ms (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _seq
     (setq _seq (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int64 _start_time
#+(or :alpha :irix6 :x86_64)
      (setf _start_time (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _start_time (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _end_time
#+(or :alpha :irix6 :x86_64)
      (setf _end_time (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _end_time (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; float64 _delta_time_ms
     (setq _delta_time_ms (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get tictoc_profiler::ProfilerEntry :md5sum-) "bfc3f9f9968c9db2e3db18a9276d6e48")
(setf (get tictoc_profiler::ProfilerEntry :datatype-) "tictoc_profiler/ProfilerEntry")
(setf (get tictoc_profiler::ProfilerEntry :definition-)
      "uint32 seq
string name
int64 start_time
int64 end_time
float64 delta_time_ms

")



(provide :tictoc_profiler/ProfilerEntry "bfc3f9f9968c9db2e3db18a9276d6e48")


