;; Auto-generated. Do not edit!


(when (boundp 'sample::sample_message)
  (if (not (find-package "SAMPLE"))
    (make-package "SAMPLE"))
  (shadow 'sample_message (find-package "SAMPLE")))
(unless (find-package "SAMPLE::SAMPLE_MESSAGE")
  (make-package "SAMPLE::SAMPLE_MESSAGE"))

(in-package "ROS")
;;//! \htmlinclude sample_message.msg.html


(defclass sample::sample_message
  :super ros::object
  :slots (_message _count ))

(defmethod sample::sample_message
  (:init
   (&key
    ((:message __message) "")
    ((:count __count) 0)
    )
   (send-super :init)
   (setq _message (string __message))
   (setq _count (round __count))
   self)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:count
   (&optional __count)
   (if __count (setq _count __count)) _count)
  (:serialization-length
   ()
   (+
    ;; string _message
    4 (length _message)
    ;; uint32 _count
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;; uint32 _count
       (write-long _count s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint32 _count
     (setq _count (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get sample::sample_message :md5sum-) "89498e8c3e7e4e4d3ec5e32aa108f04d")
(setf (get sample::sample_message :datatype-) "sample/sample_message")
(setf (get sample::sample_message :definition-)
      "string message
uint32 count

")



(provide :sample/sample_message "89498e8c3e7e4e4d3ec5e32aa108f04d")


