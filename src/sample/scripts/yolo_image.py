#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""YOLO検出結果の信頼度を画像・CSVに保存するノード。"""

import csv
import os
import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from darknet_ros_msgs.msg import BoundingBoxes
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image

# ===== 設定（最初に編集する場所）=====
NODE_NAME = "Point_head_topic"
PUB_THRESHOLD_PARAM = "~pub_threshold"
DEFAULT_PUB_THRESHOLD = 0.1

IMAGE_TOPIC = "/z/image2movie_infra"  # 検出対象画像を受信するトピック
BBOX_TOPIC = "/darknet_ros/bounding_boxes"  # YOLOのバウンディングボックス結果を受信するトピック
POINT_TOPIC = "point_head_topic"  # 互換維持用のPoint publishトピック
QUEUE_SIZE = 10
IMAGE_ENCODING = "bgr8"

SAVE_DIR = "/home/keisoku/20250310"
CSV_FILE_DETECTION = os.path.join(SAVE_DIR, "detection_data.csv")
CSV_FILE_COUNT = os.path.join(SAVE_DIR, "detection_count.csv")
WINDOW_NAME = "Detection Image"
TARGET_CLASSES = ("bottle", "cup", "bowl")


class Detector:
    """画像受信時に最新bboxを重畳して保存する。"""

    def __init__(self):
        rospy.init_node(NODE_NAME)
        self.cv_bridge = CvBridge()
        self.m_pub_threshold = rospy.get_param(PUB_THRESHOLD_PARAM, DEFAULT_PUB_THRESHOLD)
        self.pub = rospy.Publisher(POINT_TOPIC, Point, queue_size=QUEUE_SIZE)

        self.image_number = 0
        self.bbox_list = []
        self.save_dir = SAVE_DIR

        os.makedirs(self.save_dir, exist_ok=True)
        self.initialize_csv_files()

        self.sub_cam_depth = rospy.Subscriber(IMAGE_TOPIC, Image, self.image_callback)
        self.sub_darknet_bbox = rospy.Subscriber(
            BBOX_TOPIC, BoundingBoxes, self.darknet_bbox_callback
        )

    def initialize_csv_files(self):
        """出力CSVをヘッダー付きで初期化する。"""
        with open(CSV_FILE_DETECTION, "w", newline="") as file1:
            writer = csv.writer(file1)
            writer.writerow(["Image Number", "Class", "Probability"])

        with open(CSV_FILE_COUNT, "w", newline="") as file2:
            writer = csv.writer(file2)
            writer.writerow(
                ["Image Number", "Total Objects", "Bottle Count", "Cup Count", "Bowl Count"]
            )

    def darknet_bbox_callback(self, darknet_bboxs):
        """YOLO検出結果を更新する。"""
        self.bbox_list = darknet_bboxs.bounding_boxes

    def image_callback(self, img_msg):
        """画像にbboxを重畳し、CSV・画像として保存する。"""
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(img_msg, desired_encoding=IMAGE_ENCODING)
            total_objects = 0
            object_counts = {name: 0 for name in TARGET_CLASSES}

            with open(CSV_FILE_DETECTION, "a", newline="") as file1, open(
                CSV_FILE_COUNT, "a", newline=""
            ) as file2:
                writer1 = csv.writer(file1)
                writer2 = csv.writer(file2)

                for bbox in self.bbox_list:
                    if bbox.Class in object_counts and bbox.probability >= self.m_pub_threshold:
                        cv2.rectangle(
                            cv_image, (bbox.xmin, bbox.ymin), (bbox.xmax, bbox.ymax), (0, 255, 0), 2
                        )
                        label = f"{bbox.Class}: {bbox.probability:.2f}"
                        cv2.putText(
                            cv_image,
                            label,
                            (bbox.xmin, bbox.ymin - 10),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (0, 255, 0),
                            2,
                        )

                        object_counts[bbox.Class] += 1
                        total_objects += 1
                        writer1.writerow([self.image_number, bbox.Class, bbox.probability])

                writer2.writerow(
                    [
                        self.image_number,
                        total_objects,
                        object_counts["bottle"],
                        object_counts["cup"],
                        object_counts["bowl"],
                    ]
                )

            cv2.imshow(WINDOW_NAME, cv_image)
            cv2.waitKey(1)

            self.image_number += 1
            image_filename = os.path.join(self.save_dir, f"{self.image_number}.png")
            cv2.imwrite(image_filename, cv_image)
            rospy.loginfo("Image saved to %s", image_filename)

            for bbox in self.bbox_list:
                if bbox.probability >= self.m_pub_threshold:
                    rospy.loginfo(
                        "Detected %s with probability: %.2f", bbox.Class, bbox.probability
                    )

            rospy.loginfo("Total number of objects detected: %d", total_objects)
            for obj_class, count in object_counts.items():
                rospy.loginfo("Number of %ss detected: %d", obj_class, count)
        except CvBridgeError as error:
            rospy.logerr("CvBridge Error: %s", error)
        except Exception as error:
            rospy.logerr("Unexpected error: %s", error)


def main():
    rospy.loginfo("Starting Detector Node")
    Detector()
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
