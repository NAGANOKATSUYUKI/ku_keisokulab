#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox
from cv_bridge import CvBridge, CvBridgeError

class Detector():
    def __init__(self):
        rospy.init_node("object_detector")
        self.cv_bridge = CvBridge()
        self.bbox = BoundingBox()
        self.m_pub_threshold = rospy.get_param("~pub_threshold", 0.40)

        sub_cam_depth    =rospy.Subscriber("/hsrb/head_rgbd_sensor/depth_registered/image_raw", Image, self.DepthCallback)
        sub_darknet_bbox =rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.DarknetBboxCallback)


    def DepthCallback(self, depth_image_data):
        try:
            #中心座標
            w = self.bbox.xmax - self.bbox.xmin
            h = self.bbox.ymax - self.bbox.ymin
            x = self.bbox.xmin + w/2
            y = self.bbox.ymin + h/2

            #depth距離測定
            self.depth_image = self.cv_bridge.imgmsg_to_cv2(depth_image_data, "passthrough")
            depth_x = int(x)
            depth_y = int(y)
            bbox_depth = self.depth_image[depth_y][depth_x]

            #topicとメッセージ値の作成
            pub = rospy.Publisher("point_topic", Point, queue_size=10)
            point_msg = Point()
            point_msg.x = x
            point_msg.y = y
            point_msg.z = bbox_depth

            pub.publish(point_msg)
            rospy.loginfo('Published point: x={}, y={}, z={}'.format(point_msg.x, point_msg.y, point_msg.z))#ピクセル単位の座標

        except CvBridgeError as e:
            rospy.logerr("Cvbridge error: %s", e)

    def DarknetBboxCallback(self, darknet_bboxs):
        bboxs = darknet_bboxs.bounding_boxes
        bbox = BoundingBox()
        if len(bboxs) != 0 :
            for i, bb in enumerate(bboxs) :
                if bboxs[i].Class == 'bottle' and bboxs[i].probability >= self.m_pub_threshold:
                    bbox = bboxs[i]        
        self.bbox = bbox

if __name__ == "__main__":
    try:
        Detector()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
    



