#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# YOLOの検出信頼度計測

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
import csv

class Detector():
    def __init__(self):
        rospy.init_node("Point_head_topic")
        self.cv_bridge = CvBridge()
        self.bbox = BoundingBox()
        self.m_pub_threshold = rospy.get_param("~pub_threshold", 0.1)
        self.pub = rospy.Publisher("point_head_topic", Point, queue_size=10)
        self.image_number = 0
        self.bbox_list = []

        sub_cam_depth = rospy.Subscriber("/z/generated_image", Image, self.ImageCallback)
        sub_darknet_bbox = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.DarknetBboxCallback)

        # CSVファイルの初期化
        self.csv_file1 = "/home/keisoku/20241220/detection_data.csv"
        self.csv_file2 = "/home/keisoku/20241220/detection_count.csv"
        # ディレクトリが存在しない場合は作成
        os.makedirs(os.path.dirname(self.csv_file1), exist_ok=True)
        os.makedirs(os.path.dirname(self.csv_file2), exist_ok=True)
        
        with open(self.csv_file1, 'w', newline='') as file1:
            writer = csv.writer(file1)
            writer.writerow(["Image Number", "Class", "Probability"])
        with open(self.csv_file2, 'w', newline='') as file2:
            writer = csv.writer(file2)
            writer.writerow(["Image Number", "Total Objects", "Bottle Count", "Cup Count", "Bowl Count"])
        
        
    # 検出
    def DarknetBboxCallback(self, darknet_bboxs):
        self.bbox_list = darknet_bboxs.bounding_boxes

    def ImageCallback(self, img_msg):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
            total_objects = 0
            object_counts = {'bottle': 0, 'cup': 0, 'bowl': 0}

            with open(self.csv_file1, 'a', newline='') as file1, open(self.csv_file2, 'a', newline='') as file2:
                writer1 = csv.writer(file1)
                writer2 = csv.writer(file2)

                for bbox in self.bbox_list:
                    if bbox.Class in object_counts and bbox.probability >= self.m_pub_threshold:
                        cv2.rectangle(cv_image, (bbox.xmin, bbox.ymin), (bbox.xmax, bbox.ymax), (0, 255, 0), 2)
                        label = f"{bbox.Class}: {bbox.probability:.2f}"
                        cv2.putText(cv_image, label, (bbox.xmin, bbox.ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        
                        object_counts[bbox.Class] += 1
                        total_objects += 1

                        # CSVに書き込み (cup, bowl, bottle のみ)
                        writer1.writerow([self.image_number, bbox.Class, bbox.probability])
                
                # 各画像のオブジェクト数をCSVに書き込み
                writer2.writerow([self.image_number, total_objects, object_counts['bottle'], object_counts['cup'], object_counts['bowl']])

            cv2.imshow("Detection Image", cv_image)
            cv2.waitKey(1)
            
            self.image_number += 1
            
            self.save_dir = "/home/keisoku/20241220"
            image_filename = f"{self.save_dir}/{self.image_number}.png"
            
            if not os.path.exists(self.save_dir):
                os.makedirs(self.save_dir)
            
            cv2.imwrite(image_filename, cv_image)
            rospy.loginfo(f"Image saved to {image_filename}")

            # 各信頼度を表示
            for bbox in self.bbox_list:
                if bbox.probability >= self.m_pub_threshold:
                    rospy.loginfo(f"Detected {bbox.Class} with probability: {bbox.probability:.2f}")

            # 画像全体のオブジェクト数を表示
            rospy.loginfo(f"Total number of objects detected: {total_objects}")

            # 物体ごとのオブジェクト数を表示
            for obj_class, count in object_counts.items():
                rospy.loginfo(f"Number of {obj_class}s detected: {count}")

        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
        except Exception as e:
            rospy.logerr("Unexpected error: {0}".format(e))

if __name__ == "__main__":
    rospy.loginfo("Starting Detector Node")
    try:
        Detector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
