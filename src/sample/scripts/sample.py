#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""darknet_ros の検出画像を受信し、指定パスに保存する。"""

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

# ===== 設定（最初に編集する場所）=====
NODE_NAME = "image_saver"
SUBSCRIBE_TOPIC = "/darknet_ros/detection_image"
SAVE_PATH = "/home/keisoku/20250310/00000000.png"
INPUT_ENCODING = "bgr8"

class DetectionImageSaver:
    """検出画像トピックを監視して画像保存するクラス。"""

    def __init__(self):
        self.bridge = CvBridge()
        rospy.Subscriber(SUBSCRIBE_TOPIC, Image, self.callback)

    def callback(self, msg):
        """受信画像を OpenCV 形式に変換して保存する。"""
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding=INPUT_ENCODING)
        cv2.imwrite(SAVE_PATH, img)
        rospy.loginfo("Saved detected image: %s", SAVE_PATH)

def main():
    rospy.init_node(NODE_NAME)
    _saver = DetectionImageSaver()
    rospy.spin()

if __name__ == "__main__":
    main()
