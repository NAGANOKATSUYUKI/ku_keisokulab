#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""2系統画像を配信し、YOLO検出結果を重畳してCSVと画像を保存する。"""

import csv
import os
import signal
import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from darknet_ros_msgs.msg import BoundingBoxes
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image

# ===== 設定（最初に編集する場所）=====
NODE_NAME = "image_publisher"

INFRA_DIR = "/home/keisoku/ROBOT_data/Robot_data0718/fake/infra/edge"
COLOR_DIR = "/home/keisoku/pytorch-CycleGAN-and-pix2pix/datasets/Data3/Data/color"
IMAGE_EXTENSION = ".png"
PUBLISH_RATE_HZ = 1

INFRA_TOPIC = "/z/image2movie_infra"  # 推論用の赤外画像を配信する出力トピック
COLOR_TOPIC = "/z/image2movie_color"  # 推論用のカラー画像を配信する出力トピック
BBOX_TOPIC = "/darknet_ros/bounding_boxes"  # YOLO検出結果（bbox）を受信する入力トピック
POINT_TOPIC = "point_head_topic"  # Pointメッセージを配信する互換用トピック
QUEUE_SIZE = 10
PUB_THRESHOLD_PARAM = "~pub_threshold"
DEFAULT_PUB_THRESHOLD = 0.1

LOG_DIR = "/home/keisoku/gomi"
CSV_DETECTION = os.path.join(LOG_DIR, "detection_data.csv")
CSV_COUNT = os.path.join(LOG_DIR, "detection_count.csv")
OUTPUT_ENCODING = "bgr8"
OBJECT_CLASSES = ("bottle", "cup", "bowl")

class ImagePublisher:
    """画像配信とYOLO結果の記録を行うクラス。"""
    def __init__(self):
        rospy.init_node(NODE_NAME, anonymous=True)
        self.bridge = CvBridge()
        self.running = True

        self.image_pub1 = rospy.Publisher(INFRA_TOPIC, Image, queue_size=QUEUE_SIZE)
        self.image_pub2 = rospy.Publisher(COLOR_TOPIC, Image, queue_size=QUEUE_SIZE)
        self.point_pub = rospy.Publisher(POINT_TOPIC, Point, queue_size=QUEUE_SIZE)

        self.image_files1 = self.list_image_files(INFRA_DIR)
        self.image_files2 = self.list_image_files(COLOR_DIR)
        self.image_number = 0
        self.bbox_list = []
        self.current_color_image = None
        self.m_pub_threshold = rospy.get_param(PUB_THRESHOLD_PARAM, DEFAULT_PUB_THRESHOLD)

        os.makedirs(LOG_DIR, exist_ok=True)
        self.initialize_csv_files()

        signal.signal(signal.SIGINT, self.signal_handler)
        self.darknet_sub = rospy.Subscriber(
            BBOX_TOPIC, BoundingBoxes, self.darknet_bbox_callback, queue_size=1
        )

    @staticmethod
    def list_image_files(folder_path):
        """ディレクトリ内の対象画像ファイルをソートして返す。"""
        return sorted(
            os.path.join(folder_path, filename)
            for filename in os.listdir(folder_path)
            if filename.lower().endswith(IMAGE_EXTENSION)
        )

    def initialize_csv_files(self):
        """出力CSVファイルをヘッダー付きで初期化する。"""
        with open(CSV_DETECTION, "w", newline="") as file1:
            writer = csv.writer(file1)
            writer.writerow(["Image Number", "Class", "Probability"])

        with open(CSV_COUNT, "w", newline="") as file2:
            writer = csv.writer(file2)
            writer.writerow(["Image Number", "Total Objects", "Bottle Count", "Cup Count", "Bowl Count"])

    def signal_handler(self, sig, frame):
        del sig, frame
        self.running = False
        rospy.logwarn("Shutting down gracefully...")
        rospy.signal_shutdown("Ctrl+C pressed")

    def publish_images(self):
        """2つのディレクトリの画像を順番に配信する。"""
        rate = rospy.Rate(PUBLISH_RATE_HZ)

        if len(self.image_files1) != len(self.image_files2):
            rospy.logwarn("Image counts differ: infra=%d, color=%d", len(self.image_files1), len(self.image_files2))

        for img_file1, img_file2 in zip(self.image_files1, self.image_files2):
            if not self.running or rospy.is_shutdown():
                break

            image1 = cv2.imread(img_file1)
            image2 = cv2.imread(img_file2)

            if image1 is None:
                rospy.logwarn("Failed to read image file: %s", img_file1)
                continue
            if image2 is None:
                rospy.logwarn("Failed to read image file: %s", img_file2)
                continue

            self.current_color_image = image2.copy()
            ros_image1 = self.bridge.cv2_to_imgmsg(image1, OUTPUT_ENCODING)
            ros_image2 = self.bridge.cv2_to_imgmsg(image2, OUTPUT_ENCODING)

            self.image_pub1.publish(ros_image1)
            self.image_pub2.publish(ros_image2)
            rospy.loginfo("Published images: %s / %s", img_file1, img_file2)
            rate.sleep()

    def darknet_bbox_callback(self, darknet_bboxs):
        """YOLOのbbox受信時に現在画像へ重畳し、CSVと画像を保存する。"""
        self.bbox_list = darknet_bboxs.bounding_boxes
        if self.current_color_image is None:
            return
        self.process_detection_image(self.current_color_image.copy())

    def process_detection_image(self, cv_image):
        """bboxを描画し、検出結果をCSVと画像として保存する。"""
        total_objects = 0
        object_counts = {name: 0 for name in OBJECT_CLASSES}

        try:
            with open(CSV_DETECTION, "a", newline="") as file1, open(CSV_COUNT, "a", newline="") as file2:
                writer1 = csv.writer(file1)
                writer2 = csv.writer(file2)

                for bbox in self.bbox_list:
                    if bbox.Class in object_counts and bbox.probability >= self.m_pub_threshold:
                        cv2.rectangle(cv_image, (bbox.xmin, bbox.ymin), (bbox.xmax, bbox.ymax), (0, 255, 0), 2)
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

            cv2.imshow("Detection Image", cv_image)
            cv2.waitKey(1)

            self.image_number += 1
            image_filename = os.path.join(LOG_DIR, f"{self.image_number}.png")
            cv2.imwrite(image_filename, cv_image)
            rospy.loginfo("Image saved to %s", image_filename)

            for bbox in self.bbox_list:
                if bbox.probability >= self.m_pub_threshold:
                    rospy.loginfo("Detected %s with probability: %.2f", bbox.Class, bbox.probability)

            rospy.loginfo("Total number of objects detected: %d", total_objects)
            for obj_class, count in object_counts.items():
                rospy.loginfo("Number of %ss detected: %d", obj_class, count)
        except CvBridgeError as error:
            rospy.logerr("CvBridge Error: %s", error)
        except Exception as error:
            rospy.logerr("Unexpected error: %s", error)

def main():
    try:
        image_publisher = ImagePublisher()
        image_publisher.publish_images()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
        rospy.loginfo("Node terminated.")

if __name__ == "__main__":
    main()
