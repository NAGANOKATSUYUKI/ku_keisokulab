#!/usr/bin/env python
# -*- coding: utf-8 -*-

# This import is for general library
import os
import threading

# This import is for ROS integration
import rospy
from sensor_msgs.msg import Image,CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from darknet_ros_msgs.msg import BoundingBoxes,BoundingBox
import cv2
from geometry_msgs.msg import Twist

class PersonDetector():
    def __init__(self):

        self.cv_bridge = CvBridge()
        self.person_bbox = BoundingBox()
        self.vel = Twist()



        # ROS PARAM
        self.m_pub_threshold = rospy.get_param('~pub_threshold', 0.40)

        # Subscribe
        sub_camera_rgb     =  rospy.Subscriber('/camera/color/image_raw', Image, self.CamRgbImageCallback)
        sub_camera_depth   =  rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.CamDepthImageCallback)
        sub_darknet_bbox   =  rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.DarknetBboxCallback)
        #self.pub_vel = rospy.Publisher('/whill/controller/cmd_vel', Twist, queue_size=1)
        return





    def CamRgbImageCallback(self, rgb_image_data):

        try:
            self.rgb_image = self.cv_bridge.imgmsg_to_cv2(rgb_image_data, 'passthrough')
        except CvBridgeError as e:
            rospy.logerr(e)

        self.rgb_image = cv2.cvtColor(self.rgb_image, cv2.COLOR_BGR2RGB)
        self.color_height, self.color_width = self.rgb_image.shape[:2]

        self.left_width = self.color_width / 4
        self.right_width = self.color_width *3 / 4




        
        # 人がいる場合
        if self.person_bbox.probability > 0.0 :

           # 一旦、BoundingBoxの中心位置の深度を取得 
            self.m_person_depth = self.m_depth_image[(int)(self.person_bbox.ymax+self.person_bbox.ymin)/2][(int)(self.person_bbox.xmax+self.person_bbox.xmin)/2]
            self.target_center =(int)(self.person_bbox.xmax+self.person_bbox.xmin)/2
            cv2.rectangle(self.rgb_image, (self.person_bbox.xmin, self.person_bbox.ymin), (self.person_bbox.xmax, self.person_bbox.ymax),(0,255,0), 2)
            #rospy.loginfo('Class : person, Score: %.2f, Dist: %fm ' %(self.person_bbox.probability, self.m_person_depth))
            text = "person " +('%dmm' % self.m_person_depth)
            self.m_person_depth =self.m_person_depth/1000.0
            rospy.loginfo('Class : person, Score: %.2f, Dist: %fm ' %(self.person_bbox.probability, self.m_person_depth))
            if(self.target_center<self.left_width):
                self.vel.angular.z=0.3
            elif(self.target_center>self.right_width):
                self.vel.angular.z=-0.3
            if(self.m_person_depth<2.0):
                self.vel.linear.x=0.0
            elif (self.m_person_depth>=2.0):
                self.vel.linear.x=0.3
            self.pub_vel.publish(self.vel)
            text_top = (self.person_bbox.xmin, self.person_bbox.ymin - 10)
            text_bot = (self.person_bbox.xmin + 80, self.person_bbox.ymin + 5)
            text_position = (self.person_bbox.xmin + 5, self.person_bbox.ymin)
            cv2.rectangle(self.rgb_image, text_top, text_bot, (0,0,0),-1)
            cv2.putText(self.rgb_image, text, text_position, cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 0, 255), 1)
            



        cv2.namedWindow("self.rgb_image")
        cv2.imshow("self.rgb_image", self.rgb_image)
        cv2.waitKey(1)
        cv2.namedWindow("depth_image")
        cv2.imshow("depth_image",self.m_depth_image)  
        cv2.waitKey(1)

        return



    def CamDepthImageCallback(self, depth_image_data):
        try:
            self.m_depth_image = self.cv_bridge.imgmsg_to_cv2(depth_image_data, 'passthrough')
        except CvBridgeError as e:
            rospy.logerr(e)
        self.m_camdepth_height, self.m_camdepth_width = self.m_depth_image.shape[:2]
        return

    def DarknetBboxCallback(self, darknet_bboxs):
        bboxs = darknet_bboxs.bounding_boxes
        person_bbox = BoundingBox()
        if len(bboxs) != 0 :
            for i, bb in enumerate(bboxs) :
                if bboxs[i].Class == 'person' and bboxs[i].probability >= self.m_pub_threshold:
                    person_bbox = bboxs[i]        
        self.person_bbox = person_bbox



if __name__ == '__main__':
    try:
        rospy.init_node('person_detector', anonymous=True)
        idc = PersonDetector()
        rospy.loginfo('idc Initialized')
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
