;; Auto-generated. Do not edit!


(when (boundp 'serial_comms::INSPVAE)
  (if (not (find-package "SERIAL_COMMS"))
    (make-package "SERIAL_COMMS"))
  (shadow 'INSPVAE (find-package "SERIAL_COMMS")))
(unless (find-package "SERIAL_COMMS::INSPVAE")
  (make-package "SERIAL_COMMS::INSPVAE"))

(in-package "ROS")
;;//! \htmlinclude INSPVAE.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass serial_comms::INSPVAE
  :super ros::object
  :slots (_header _week _seconds _latitude _longitude _altitude _undulation _std_lat _std_lon _std_alt _ve _vn _vu _std_ve _std_vn _std_vu _pitch _roll _yaw _std_pitch _std_roll _std_yaw _ns _gnss_st _nav_st _odo_st _nav_status ))

(defmethod serial_comms::INSPVAE
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:week __week) 0)
    ((:seconds __seconds) 0.0)
    ((:latitude __latitude) 0.0)
    ((:longitude __longitude) 0.0)
    ((:altitude __altitude) 0.0)
    ((:undulation __undulation) 0.0)
    ((:std_lat __std_lat) 0.0)
    ((:std_lon __std_lon) 0.0)
    ((:std_alt __std_alt) 0.0)
    ((:ve __ve) 0.0)
    ((:vn __vn) 0.0)
    ((:vu __vu) 0.0)
    ((:std_ve __std_ve) 0.0)
    ((:std_vn __std_vn) 0.0)
    ((:std_vu __std_vu) 0.0)
    ((:pitch __pitch) 0.0)
    ((:roll __roll) 0.0)
    ((:yaw __yaw) 0.0)
    ((:std_pitch __std_pitch) 0.0)
    ((:std_roll __std_roll) 0.0)
    ((:std_yaw __std_yaw) 0.0)
    ((:ns __ns) 0)
    ((:gnss_st __gnss_st) 0)
    ((:nav_st __nav_st) 0)
    ((:odo_st __odo_st) 0)
    ((:nav_status __nav_status) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _week (round __week))
   (setq _seconds (float __seconds))
   (setq _latitude (float __latitude))
   (setq _longitude (float __longitude))
   (setq _altitude (float __altitude))
   (setq _undulation (float __undulation))
   (setq _std_lat (float __std_lat))
   (setq _std_lon (float __std_lon))
   (setq _std_alt (float __std_alt))
   (setq _ve (float __ve))
   (setq _vn (float __vn))
   (setq _vu (float __vu))
   (setq _std_ve (float __std_ve))
   (setq _std_vn (float __std_vn))
   (setq _std_vu (float __std_vu))
   (setq _pitch (float __pitch))
   (setq _roll (float __roll))
   (setq _yaw (float __yaw))
   (setq _std_pitch (float __std_pitch))
   (setq _std_roll (float __std_roll))
   (setq _std_yaw (float __std_yaw))
   (setq _ns (round __ns))
   (setq _gnss_st (round __gnss_st))
   (setq _nav_st (round __nav_st))
   (setq _odo_st (round __odo_st))
   (setq _nav_status (string __nav_status))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:week
   (&optional __week)
   (if __week (setq _week __week)) _week)
  (:seconds
   (&optional __seconds)
   (if __seconds (setq _seconds __seconds)) _seconds)
  (:latitude
   (&optional __latitude)
   (if __latitude (setq _latitude __latitude)) _latitude)
  (:longitude
   (&optional __longitude)
   (if __longitude (setq _longitude __longitude)) _longitude)
  (:altitude
   (&optional __altitude)
   (if __altitude (setq _altitude __altitude)) _altitude)
  (:undulation
   (&optional __undulation)
   (if __undulation (setq _undulation __undulation)) _undulation)
  (:std_lat
   (&optional __std_lat)
   (if __std_lat (setq _std_lat __std_lat)) _std_lat)
  (:std_lon
   (&optional __std_lon)
   (if __std_lon (setq _std_lon __std_lon)) _std_lon)
  (:std_alt
   (&optional __std_alt)
   (if __std_alt (setq _std_alt __std_alt)) _std_alt)
  (:ve
   (&optional __ve)
   (if __ve (setq _ve __ve)) _ve)
  (:vn
   (&optional __vn)
   (if __vn (setq _vn __vn)) _vn)
  (:vu
   (&optional __vu)
   (if __vu (setq _vu __vu)) _vu)
  (:std_ve
   (&optional __std_ve)
   (if __std_ve (setq _std_ve __std_ve)) _std_ve)
  (:std_vn
   (&optional __std_vn)
   (if __std_vn (setq _std_vn __std_vn)) _std_vn)
  (:std_vu
   (&optional __std_vu)
   (if __std_vu (setq _std_vu __std_vu)) _std_vu)
  (:pitch
   (&optional __pitch)
   (if __pitch (setq _pitch __pitch)) _pitch)
  (:roll
   (&optional __roll)
   (if __roll (setq _roll __roll)) _roll)
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:std_pitch
   (&optional __std_pitch)
   (if __std_pitch (setq _std_pitch __std_pitch)) _std_pitch)
  (:std_roll
   (&optional __std_roll)
   (if __std_roll (setq _std_roll __std_roll)) _std_roll)
  (:std_yaw
   (&optional __std_yaw)
   (if __std_yaw (setq _std_yaw __std_yaw)) _std_yaw)
  (:ns
   (&optional __ns)
   (if __ns (setq _ns __ns)) _ns)
  (:gnss_st
   (&optional __gnss_st)
   (if __gnss_st (setq _gnss_st __gnss_st)) _gnss_st)
  (:nav_st
   (&optional __nav_st)
   (if __nav_st (setq _nav_st __nav_st)) _nav_st)
  (:odo_st
   (&optional __odo_st)
   (if __odo_st (setq _odo_st __odo_st)) _odo_st)
  (:nav_status
   (&optional __nav_status)
   (if __nav_status (setq _nav_status __nav_status)) _nav_status)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint32 _week
    4
    ;; float64 _seconds
    8
    ;; float64 _latitude
    8
    ;; float64 _longitude
    8
    ;; float64 _altitude
    8
    ;; float64 _undulation
    8
    ;; float64 _std_lat
    8
    ;; float64 _std_lon
    8
    ;; float64 _std_alt
    8
    ;; float64 _ve
    8
    ;; float64 _vn
    8
    ;; float64 _vu
    8
    ;; float64 _std_ve
    8
    ;; float64 _std_vn
    8
    ;; float64 _std_vu
    8
    ;; float64 _pitch
    8
    ;; float64 _roll
    8
    ;; float64 _yaw
    8
    ;; float64 _std_pitch
    8
    ;; float64 _std_roll
    8
    ;; float64 _std_yaw
    8
    ;; uint32 _ns
    4
    ;; uint32 _gnss_st
    4
    ;; uint32 _nav_st
    4
    ;; uint32 _odo_st
    4
    ;; string _nav_status
    4 (length _nav_status)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint32 _week
       (write-long _week s)
     ;; float64 _seconds
       (sys::poke _seconds (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _latitude
       (sys::poke _latitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _longitude
       (sys::poke _longitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _altitude
       (sys::poke _altitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _undulation
       (sys::poke _undulation (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _std_lat
       (sys::poke _std_lat (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _std_lon
       (sys::poke _std_lon (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _std_alt
       (sys::poke _std_alt (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _ve
       (sys::poke _ve (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _vn
       (sys::poke _vn (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _vu
       (sys::poke _vu (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _std_ve
       (sys::poke _std_ve (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _std_vn
       (sys::poke _std_vn (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _std_vu
       (sys::poke _std_vu (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _pitch
       (sys::poke _pitch (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _roll
       (sys::poke _roll (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _yaw
       (sys::poke _yaw (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _std_pitch
       (sys::poke _std_pitch (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _std_roll
       (sys::poke _std_roll (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _std_yaw
       (sys::poke _std_yaw (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; uint32 _ns
       (write-long _ns s)
     ;; uint32 _gnss_st
       (write-long _gnss_st s)
     ;; uint32 _nav_st
       (write-long _nav_st s)
     ;; uint32 _odo_st
       (write-long _odo_st s)
     ;; string _nav_status
       (write-long (length _nav_status) s) (princ _nav_status s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint32 _week
     (setq _week (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64 _seconds
     (setq _seconds (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _latitude
     (setq _latitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _longitude
     (setq _longitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _altitude
     (setq _altitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _undulation
     (setq _undulation (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _std_lat
     (setq _std_lat (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _std_lon
     (setq _std_lon (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _std_alt
     (setq _std_alt (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _ve
     (setq _ve (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _vn
     (setq _vn (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _vu
     (setq _vu (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _std_ve
     (setq _std_ve (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _std_vn
     (setq _std_vn (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _std_vu
     (setq _std_vu (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _pitch
     (setq _pitch (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _roll
     (setq _roll (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _yaw
     (setq _yaw (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _std_pitch
     (setq _std_pitch (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _std_roll
     (setq _std_roll (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _std_yaw
     (setq _std_yaw (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; uint32 _ns
     (setq _ns (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _gnss_st
     (setq _gnss_st (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _nav_st
     (setq _nav_st (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _odo_st
     (setq _odo_st (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _nav_status
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _nav_status (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get serial_comms::INSPVAE :md5sum-) "952992b8f2d9974237c40c1e9c3b19ce")
(setf (get serial_comms::INSPVAE :datatype-) "serial_comms/INSPVAE")
(setf (get serial_comms::INSPVAE :definition-)
      "Header header
uint32 week
float64 seconds
float64 latitude
float64 longitude
float64 altitude
float64 undulation
float64 std_lat
float64 std_lon
float64 std_alt
float64 ve
float64 vn
float64 vu
float64 std_ve
float64 std_vn
float64 std_vu
float64 pitch
float64 roll
float64 yaw
float64 std_pitch
float64 std_roll
float64 std_yaw
uint32 ns
uint32 gnss_st
uint32 nav_st
uint32 odo_st
string nav_status

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :serial_comms/INSPVAE "952992b8f2d9974237c40c1e9c3b19ce")


