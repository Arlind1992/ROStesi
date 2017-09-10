;; Auto-generated. Do not edit!


(when (boundp 'rrt_planning::CommMsg)
  (if (not (find-package "RRT_PLANNING"))
    (make-package "RRT_PLANNING"))
  (shadow 'CommMsg (find-package "RRT_PLANNING")))
(unless (find-package "RRT_PLANNING::COMMMSG")
  (make-package "RRT_PLANNING::COMMMSG"))

(in-package "ROS")
;;//! \htmlinclude CommMsg.msg.html


(defclass rrt_planning::CommMsg
  :super ros::object
  :slots (_cellX _cellY _uploadTime ))

(defmethod rrt_planning::CommMsg
  (:init
   (&key
    ((:cellX __cellX) 0)
    ((:cellY __cellY) 0)
    ((:uploadTime __uploadTime) 0)
    )
   (send-super :init)
   (setq _cellX (round __cellX))
   (setq _cellY (round __cellY))
   (setq _uploadTime (round __uploadTime))
   self)
  (:cellX
   (&optional __cellX)
   (if __cellX (setq _cellX __cellX)) _cellX)
  (:cellY
   (&optional __cellY)
   (if __cellY (setq _cellY __cellY)) _cellY)
  (:uploadTime
   (&optional __uploadTime)
   (if __uploadTime (setq _uploadTime __uploadTime)) _uploadTime)
  (:serialization-length
   ()
   (+
    ;; uint32 _cellX
    4
    ;; uint32 _cellY
    4
    ;; uint32 _uploadTime
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _cellX
       (write-long _cellX s)
     ;; uint32 _cellY
       (write-long _cellY s)
     ;; uint32 _uploadTime
       (write-long _uploadTime s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _cellX
     (setq _cellX (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _cellY
     (setq _cellY (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _uploadTime
     (setq _uploadTime (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get rrt_planning::CommMsg :md5sum-) "2ae8e81eb815f3fe2d8493f75b85e1c9")
(setf (get rrt_planning::CommMsg :datatype-) "rrt_planning/CommMsg")
(setf (get rrt_planning::CommMsg :definition-)
      "
uint32  cellX	
uint32  cellY
uint32 uploadTime

")



(provide :rrt_planning/CommMsg "2ae8e81eb815f3fe2d8493f75b85e1c9")


