#!/usr/bin/env python3

"""2つの画像ディレクトリを同期的に ROS Image トピックへ配信する。"""

import os
import signal

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

# ===== 設定（最初に編集する場所）=====
NODE_NAME = "image_publisher"
INFRA_DIR = "/home/keisoku/Depth-Anything-V2/output"
COLOR_DIR = "/home/keisoku/Depth-Anything-V2/output"
IMAGE_EXTENSION = ".png"
FPS = 0.1
INFRA_TOPIC = "/z/image2movie_infra"  # 赤外線画像を配信する出力トピック
COLOR_TOPIC = "/z/image2movie_color"  # カラー画像を配信する出力トピック
QUEUE_SIZE = 5
OUTPUT_ENCODING = "bgr8"
 

class ImagePublisher:
    def __init__(self):
        rospy.init_node(NODE_NAME, anonymous=True)
        self.bridge = CvBridge()
        self.running = True

        self.image_pub1 = rospy.Publisher(INFRA_TOPIC, Image, queue_size=QUEUE_SIZE)
        self.image_pub2 = rospy.Publisher(COLOR_TOPIC, Image, queue_size=QUEUE_SIZE)

        self.image_files1 = self.list_image_files(INFRA_DIR)
        self.image_files2 = self.list_image_files(COLOR_DIR)
        signal.signal(signal.SIGINT, self.signal_handler)

    @staticmethod
    def list_image_files(image_folder):
        """対象ディレクトリ内の画像ファイルパスをソートして返す。"""
        return sorted(
            os.path.join(image_folder, filename)
            for filename in os.listdir(image_folder)
            if filename.lower().endswith(IMAGE_EXTENSION)
        )

    def signal_handler(self, sig, frame):
        del sig, frame
        self.running = False
        rospy.logwarn("Shutting down gracefully...")
        rospy.signal_shutdown("Ctrl+C pressed")

    def publish_images(self):
        rate = rospy.Rate(FPS)

        for img_file1, img_file2 in zip(self.image_files1, self.image_files2):
            if not self.running:
                break

            image1 = cv2.imread(img_file1)
            image2 = cv2.imread(img_file2)

            if image1 is None:
                rospy.logwarn("Failed to read image file: %s", img_file1)
                continue
            if image2 is None:
                rospy.logwarn("Failed to read image file: %s", img_file2)
                continue

            ros_image1 = self.bridge.cv2_to_imgmsg(image1, OUTPUT_ENCODING)
            ros_image2 = self.bridge.cv2_to_imgmsg(image2, OUTPUT_ENCODING)

            self.image_pub1.publish(ros_image1)
            self.image_pub2.publish(ros_image2)

            rospy.loginfo("Published images: %s / %s", img_file1, img_file2)
            rate.sleep()


def main():
    try:
        image_publisher = ImagePublisher()
        image_publisher.publish_images()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("Node terminated.")


if __name__ == "__main__":
    main()
