#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo
import cv2

class Detector():
    def __init__(self):
        rospy.init_node("Point_head_topic")
        self.cv_bridge = CvBridge()
        self.bbox = BoundingBox()
        self.m_pub_threshold = rospy.get_param("~pub_threshold", 0.20)
        self.pub = rospy.Publisher("point_head_topic", Point, queue_size=10)

        self.cam_x = 0.0
        self.cam_y = 0.0

        sub_cam_depth     = rospy.Subscriber("/hsrb/head_rgbd_sensor/depth_registered/image_raw", Image, self.DepthCallback)
        # sub_cam_info      = rospy.Subscriber("/hsrb/head_rgbd_sensor/depth_registered/camera_info", CameraInfo, self.CameraInfo_callback)
        sub_darknet_bbox  = rospy.Subscriber("/darknet_ros0/bounding_boxes", BoundingBoxes, self.DarknetBboxCallback)

    #検出
    def DarknetBboxCallback(self, darknet_bboxs):
        bboxs = darknet_bboxs.bounding_boxes
        bbox = BoundingBox()
        if len(bboxs) != 0 :
            for i, bb in enumerate(bboxs) :
                if bboxs[i].Class == 'bottle' and bboxs[i].probability >= self.m_pub_threshold:
                    bbox = bboxs[i]   
                    self.bbox = bbox
                    #中心座標
                    self.cam_x = self.bbox.xmin + (self.bbox.xmax - self.bbox.xmin) / 2
                    self.cam_y = self.bbox.ymin + (self.bbox.ymax - self.bbox.ymin) / 2
                    # rospy.loginfo("cam= %.2f %.2f ", self.cam_x,self.cam_y)
                elif bboxs[i].Class == 'cup' and bboxs[i].probability >= self.m_pub_threshold:
                    bbox = bboxs[i]
                    self.bbox = bbox
                    #中心座標
                    self.cam_x = self.bbox.xmin + (self.bbox.xmax - self.bbox.xmin) / 2
                    self.cam_y = self.bbox.ymin + (self.bbox.ymax - self.bbox.ymin) / 2
                    # rospy.loginfo("cam= %.2f %.2f ", self.cam_x,self.cam_y)
                else :
                    pass       
                    
    def DepthCallback(self, depth_image_data):
        try:
            #depth距離測定
            self.depth_image = self.cv_bridge.imgmsg_to_cv2(depth_image_data, "passthrough")
            depth_x = int(self.cam_x)
            depth_y = int(self.cam_y)
            self.bbox_depth = self.depth_image[depth_y][depth_x]

            self.CameraInfo_callback()
            # cv2.imshow('depth',self.depth_image)
            # cv2.waitKey(1)

        except CvBridgeError as e:
            rospy.logerr("Cvbridge error: %s", e)

    #座標変換
    # def CameraInfo_callback(self, info_msg):
    def CameraInfo_callback(self):
        if self.cam_x != 0.0 and self.cam_y != 0.0 or self.cam_x > self.cam_y :
            if self.cam_y > 125.0:  #y座標の上の方は対象外
                try:
                    self.x = self.cam_x - 320.7234912485017  #depth_infoの値を入力
                    self.y = self.cam_y - 242.2827148742709
                    self.z = self.bbox_depth

                    self.x1 = self.z * self.x / 533.8084805746594
                    self.y1 = self.z * self.y / 534.057713759154

                    self.x1 = self.x1 * 0.001
                    if self.bbox.Class == "bottle" :
                        self.y1 = self.y1 * 0.001 + 0.2 * self.y1 *0.001
                    else :
                        self.y1 = self.y1 * 0.001
                        
                    self.z = self.z * 0.001
                except:
                    rospy.logwarn("transform_ERROR")

                #topic_publish
                try:
                    if 0.40 < self.z < 1.4 :
                        if -0.4 < self.x1 < 0.4 :
                            if self.bbox.Class == "bottle" :
                                point_msg = Point()
                                point_msg.x = self.x1
                                point_msg.y = self.y1
                                point_msg.z = self.z
                                self.pub.publish(point_msg)
                                rospy.loginfo("x =%.2f y=%.2f z=%.2f ", self.x1, self.y1, self.z)
                            else :
                                point_msg = Point()
                                point_msg.x = self.x1
                                point_msg.y = self.y1
                                point_msg.z = self.z 
                                self.pub.publish(point_msg)
                                rospy.loginfo("x =%.2f y=%.2f z=%.2f ", self.x1, self.y1, self.z)
                        else:
                            rospy.logwarn("Out of range_X")
                    else:
                        rospy.logwarn("Not Detection --> Distance over z=%.2f ", self.z)
                        # rospy.loginfo("x =%.2f y=%.2f z=%.2f ", self.x1, self.y1, self.z)
                        self.cam_x = 0.0
                        self.cam_y = 0.0
                        self.x1 = 0.0
                        self.y1 = 0.0
                        # rospy.logwarn("Distance over")    
                except:
                    rospy.logwarn("Publish_ERROR")
            else:
                rospy.logwarn("Out of range_Y")
        else:
            rospy.logwarn("Not Detection --> ")
            # rospy.loginfo("cam= %.2f %.2f x,y= %.2f %.2f", self.cam_x,self.cam_y, self.x1,self.y1)
            pass

if __name__=="__main__":
    rospy.loginfo("Unable to create tf")
    try:
        
        Detector()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

#1117
#1120