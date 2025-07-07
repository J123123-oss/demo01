;; Auto-generated. Do not edit!


(when (boundp 'serial_comms::Distances)
  (if (not (find-package "SERIAL_COMMS"))
    (make-package "SERIAL_COMMS"))
  (shadow 'Distances (find-package "SERIAL_COMMS")))
(unless (find-package "SERIAL_COMMS::DISTANCES")
  (make-package "SERIAL_COMMS::DISTANCES"))

(in-package "ROS")
;;//! \htmlinclude Distances.msg.html


(defclass serial_comms::Distances
  :super ros::object
  :slots (_distance_a _distance_b _distance_c _distance_d _distance_e _distance_f ))

(defmethod serial_comms::Distances
  (:init
   (&key
    ((:distance_a __distance_a) 0)
    ((:distance_b __distance_b) 0)
    ((:distance_c __distance_c) 0)
    ((:distance_d __distance_d) 0)
    ((:distance_e __distance_e) 0)
    ((:distance_f __distance_f) 0)
    )
   (send-super :init)
   (setq _distance_a (round __distance_a))
   (setq _distance_b (round __distance_b))
   (setq _distance_c (round __distance_c))
   (setq _distance_d (round __distance_d))
   (setq _distance_e (round __distance_e))
   (setq _distance_f (round __distance_f))
   self)
  (:distance_a
   (&optional __distance_a)
   (if __distance_a (setq _distance_a __distance_a)) _distance_a)
  (:distance_b
   (&optional __distance_b)
   (if __distance_b (setq _distance_b __distance_b)) _distance_b)
  (:distance_c
   (&optional __distance_c)
   (if __distance_c (setq _distance_c __distance_c)) _distance_c)
  (:distance_d
   (&optional __distance_d)
   (if __distance_d (setq _distance_d __distance_d)) _distance_d)
  (:distance_e
   (&optional __distance_e)
   (if __distance_e (setq _distance_e __distance_e)) _distance_e)
  (:distance_f
   (&optional __distance_f)
   (if __distance_f (setq _distance_f __distance_f)) _distance_f)
  (:serialization-length
   ()
   (+
    ;; int32 _distance_a
    4
    ;; int32 _distance_b
    4
    ;; int32 _distance_c
    4
    ;; int32 _distance_d
    4
    ;; int32 _distance_e
    4
    ;; int32 _distance_f
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _distance_a
       (write-long _distance_a s)
     ;; int32 _distance_b
       (write-long _distance_b s)
     ;; int32 _distance_c
       (write-long _distance_c s)
     ;; int32 _distance_d
       (write-long _distance_d s)
     ;; int32 _distance_e
       (write-long _distance_e s)
     ;; int32 _distance_f
       (write-long _distance_f s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _distance_a
     (setq _distance_a (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _distance_b
     (setq _distance_b (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _distance_c
     (setq _distance_c (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _distance_d
     (setq _distance_d (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _distance_e
     (setq _distance_e (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _distance_f
     (setq _distance_f (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get serial_comms::Distances :md5sum-) "88930f00743784851f3f6d92c51802bd")
(setf (get serial_comms::Distances :datatype-) "serial_comms/Distances")
(setf (get serial_comms::Distances :definition-)
      "# Defines a message for holding the six distance values.
int32 distance_a
int32 distance_b
int32 distance_c
int32 distance_d
int32 distance_e
int32 distance_f

")



(provide :serial_comms/Distances "88930f00743784851f3f6d92c51802bd")


