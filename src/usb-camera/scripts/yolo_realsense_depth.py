#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox
import cv2
import csv

class Detector():
    def __init__(self) :

        self.cv_bridge = CvBridge()
        self.bbox = BoundingBox()
        self.m_pub_threshold = rospy.get_param("~pub_threshold", 0.40)
        self.count = 0
        self.depth_distance = []
        self.probability = []
        self.last_depth_distance = None
        self.average_probability = 0
        self.average_depth_distance = 0

        sub_cam_rgb      = rospy.Subscriber("/camera/color/image_raw", Image, self.ImageCallback)
        sub_darknet_bbox = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.DarknetBboxCallback)
        sub_cam_depth    = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.DepthCallback)

        # csv出力
        self.csv_file = open("yolo_distance_data.csv", "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([ "Depth", "Score"])

    def ImageCallback(self, cam_image_msg):
        try:
            # ROSのImageメッセージをOpenCVの画像形式に変換
            cam_image = self.cv_bridge.imgmsg_to_cv2(cam_image_msg, "bgr8")
            height, width = cam_image.shape[:2]

        except CvBridgeError as e:
            rospy.logerr(e)

        # スケーリングなし
        if self.bbox.probability > 0.0 :
            #中心座標
            w = self.bbox.xmax - self.bbox.xmin
            h = self.bbox.ymax - self.bbox.ymin
            x = self.bbox.xmin + w/2
            y = self.bbox.ymin + h/2
            class_name = str(self.bbox.Class)
                
            #深度距離
            depth_x = int((self.bbox.xmax + self.bbox.xmin) // 2)
            depth_y = int((self.bbox.ymax + self.bbox.ymin) // 2)
            bbox_depth = self.depth_image[depth_y][depth_x]

            # rospy.loginfo("Class: %s  Score: %.2f  center: %.1f,%.1f  Dist: %dmm  count: %d" % (class_name, self.bbox.probability, x, y, bbox_depth,self.count))
            rospy.loginfo("  Score: %.2f  Dist: %lfmm  count: %d" % (self.bbox.probability, bbox_depth,self.count))

            # rospy.loginfo("color:width= %d, height= %d, depth:width= %d, height= %d" %(width, height, self.depth_width, self.depth_height ))

            #検出したものを円で囲む
            cv2.circle(cam_image, (int(x), int(y)), 5, (0,0,255), -1)
            #検出したものを矩形で囲む
            cv2.rectangle(cam_image, (self.bbox.xmin, self.bbox.ymin), (self.bbox.xmax, self.bbox.ymax),(0,0,255), 2)
            cv2.putText(cam_image, class_name, (self.bbox.xmin, self.bbox.ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255),2 )
            
            cv2.imshow("cam_image", cam_image)
            cv2.waitKey(1)
            cv2.imshow("depth_image", self.depth_image)
            cv2.waitKey(1)

            # 平均深度距離
            if 0 < bbox_depth <= 2500:   #0より大きく1000以下 
                self.probability.append(self.bbox.probability)
                self.depth_distance.append(bbox_depth)
                # self.count += 1
                self.csv_writer.writerow([bbox_depth, self.bbox.probability]) #csv
            # if bbox_depth != 0:    
            #     if self.last_depth_distance is not None and abs(bbox_depth - self.last_depth_distance) < 50: #last_depth_distanceがNoneではなくかつ差が50mmない場合
            #         self.depth_distance.append(bbox_depth)
            #         self.count += 1
            #     else :
            #         self.last_depth_distance = bbox_depth #うまくコードがつくれない
            #         self.count += 1
                    
            # if self.count >= 30:
            #     self.average_probability = sum(self.probability) / len(self.probability)
            #     self.average_depth_distance = sum(self.depth_distance) / len(self.depth_distance)
            #     rospy.loginfo("信頼度： %.3f  平均深度距離： %dmm" % (self.average_probability, self.average_depth_distance))
                
            #     # csv
            #     self.csv_writer.writerow([])
            #     self.csv_writer.writerow([ self.average_depth_distance, self.average_probability])
            #     self.csv_writer.writerow([])

            #     self.depth_distance = []
            #     self.probability = []
            #     self.count = 0

        else :
            cv2.imshow("cam_image", cam_image)
            cv2.waitKey(1)
            cv2.imshow("depth_image", self.depth_image)
            cv2.waitKey(1)
    
    def DepthCallback(self, depth_image_data):
        try:
            self.depth_image = self.cv_bridge.imgmsg_to_cv2(depth_image_data, "passthrough")
            self.depth_height, self.depth_width = self.depth_image.shape[:2]

        except CvBridgeError as e:
            rospy.logerr(e)

    def DarknetBboxCallback(self, darknet_bboxs):
        bboxs = darknet_bboxs.bounding_boxes
        bbox = BoundingBox()
        if len(bboxs) != 0 :
            for i, bb in enumerate(bboxs) :
                if bboxs[i].Class == 'bottle' and bboxs[i].probability >= self.m_pub_threshold:
                    bbox = bboxs[i]        
        self.bbox = bbox

    def __del__(self):

        self.csv_file.close()
    
if __name__ == '__main__':
    try:
        rospy.init_node('yolo_object_detection', anonymous=True)

        data = Detector()
        rospy.loginfo("data Initialized")
        rospy.spin() 
        
    except rospy.ROSInterruptException:
        pass
