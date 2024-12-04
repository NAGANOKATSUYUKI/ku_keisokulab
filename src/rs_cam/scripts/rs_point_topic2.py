#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo


class Detector():
    def __init__(self):
        rospy.init_node("object_deterctor")
        self.cv_bridge = CvBridge()
        self.bbox = BoundingBox()
        self.m_pub_threshold = rospy.get_param("~pub_threshold", 0.40)

        sub_cam_depth     = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.DepthCallback)
        sub_cam_info      = rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.CameraInfo_callback)
        sub_darknet_bbox  = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.DarknetBboxCallback)

    def DepthCallback(self, depth_image_data):
        try:
            #中心座標
            w = self.bbox.xmax - self.bbox.xmin
            h = self.bbox.ymax - self.bbox.ymin
            self.cam_x = self.bbox.xmin + w/2
            self.cam_y = self.bbox.ymin + h/2

            #depth距離測定
            self.depth_image = self.cv_bridge.imgmsg_to_cv2(depth_image_data, "passthrough")
            depth_x = int(self.cam_x)
            depth_y = int(self.cam_y)
            self.bbox_depth = self.depth_image[depth_y][depth_x]

        except CvBridgeError as e:
            rospy.logerr("Cvbridge error: %s", e)

    #座標変換
    def CameraInfo_callback(self, info_msg):
        try:
            self.x = self.cam_x - info_msg.K[2]
            self.y = self.cam_y - info_msg.K[5]
            self.z = self.bbox_depth

            self.x1 = self.z * self.x / info_msg.K[0]
            self.y1 = self.z * self.y / info_msg.K[4]

            self.x1 = self.x1 * 0.001
            self.y1 = self.y1 * 0.001
            self.z = self.z * 0.001
            rospy.loginfo("x =%.2f, y=%.2f, z=%.2f ", self.x1, self.y1, self.z)

        except:
            rospy.logwarn("transform_ERROR")

        #topic_publish
        try:
            #topicとメッセージ値の作成
            pub = rospy.Publisher("point_topic", Point, queue_size=10)
            point_msg = Point()
            point_msg.x = self.x1
            point_msg.y = self.y1
            point_msg.z = self.z
            pub.publish(point_msg)

        except:
            rospy.logwarn("Publish_ERROR")

    def DarknetBboxCallback(self, darknet_bboxs):
        bboxs = darknet_bboxs.bounding_boxes
        bbox = BoundingBox()
        if len(bboxs) != 0 :
            for i, bb in enumerate(bboxs) :
                if bboxs[i].Class == 'bottle' and bboxs[i].probability >= self.m_pub_threshold:
                    bbox = bboxs[i]        
        self.bbox = bbox


if __name__=="__main__":
    rospy.loginfo("Unable to create tf")
    try:
        
        Detector()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
