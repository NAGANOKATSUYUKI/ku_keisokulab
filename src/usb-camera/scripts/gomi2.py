#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from darknet_ros_msgs.msg import BoundingBoxes
import cv2
from cv_bridge import CvBridge, CvBridgeError


class Detector(object):
    def __init__(self):
        self.yolo_x = 320
        self.yolo_y = 240

        #topic_subscriber
        rospy.init_node("detector_node")
        rospy.Subscriber("hsrb/head_rgbd_sensor/depth_registered/image_raw", Image, self.Camera_callback)
        #rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.Camera_callback)
        rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.Bbox_callback)

        rospy.loginfo("Detector --> OK")


    def Camera_callback(self, msg):
        try:
            bridge = CvBridge()
            self.image = bridge.imgmsg_to_cv2(msg, "passthrough") # passthrough , bgr8 , mono8

            self.depth_data = self.image[int(self.yolo_y)][int(self.yolo_x)]

            #topicとメッセージの作成
            pub = rospy.Publisher('point_topic', Point, queue_size=10)

            point_msg = Point()
            point_msg.x = self.yolo_x
            point_msg.y = self.yolo_y
            point_msg.z = self.depth_data

            pub.publish(point_msg)
            rospy.loginfo('Published point: x={}, y={}, z={}'.format(point_msg.x, point_msg.y, point_msg.z))

            # 画像を表示する
            #cv2.imshow("Test Image", self.image)
            #cv2.waitKey(1)
        
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: %s", e)


    def Bbox_callback(self, msg = BoundingBoxes):
        for box in msg.bounding_boxes:

            if box.Class == "bottle":
                self.yolo_x = (box.xmin+box.xmax)/2
                self.yolo_y = (box.ymin+box.ymax)/2

                print("x: ( %s + %s )/2 = %s "%(box.xmin,box.xmax,self.yolo_x))
                print("y: ( %s + %s )/2 = %s "%(box.ymin,box.ymax,self.yolo_y))


if __name__ == "__main__":
    Detector()
    rospy.spin()


# "/camera/depth/image_rect_raw"
# /camera/aligned_depth_to_color/image_raw
# /hsrb/head_rgbd_sensor/rgb/image_raw
# "hsrb/head_rgbd_sensor/depth_registered/image_raw"



