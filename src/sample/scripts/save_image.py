#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""複数トピックの画像を一定間隔で自動保存する。"""

import os
import time
from threading import Lock

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

# ===== 設定（最初に編集する場所）=====
NODE_NAME = "multi_image_saver"
SAVE_INTERVAL_SEC = 5
LOOP_RATE_HZ = 10

TOPIC_COLOR = "/camera/color/image_raw"  # 生カラー画像の入力トピック
TOPIC_COLOR_FILTERING = "/z/generated_image"  # フィルタ後カラー画像の入力トピック
TOPIC_IR_FILTERING = "/z/infra_image_masked"  # フィルタ後赤外画像の入力トピック
TOPIC_IR = "/camera/infra/image_raw"  # 生赤外画像の入力トピック

SAVE_DIR_COLOR = "/home/keisoku/save_image/color"
SAVE_DIR_COLOR_FILTERING = "/home/keisoku/save_image/color_filtering"
SAVE_DIR_IR_FILTERING = "/home/keisoku/save_image/ir_filtering"
SAVE_DIR_IR = "/home/keisoku/save_image/ir"

class MultiImageSaver:
    """4トピック画像を受信し、一定間隔ごとに保存する。"""

    def __init__(self):
        rospy.init_node(NODE_NAME, anonymous=True)
        self.bridge = CvBridge()
        self.lock = Lock()
        self.image_data = {
            "color": None,
            "color_filtering": None,
            "ir_filtering": None,
            "ir": None
        }
        self.counter = 1
        self.save_interval_sec = SAVE_INTERVAL_SEC
        self.last_save_time = time.time()

        rospy.Subscriber(TOPIC_COLOR, Image, self.color_callback)
        rospy.Subscriber(TOPIC_COLOR_FILTERING, Image, self.color_filtering_callback)
        rospy.Subscriber(TOPIC_IR_FILTERING, Image, self.ir_filtering_callback)
        rospy.Subscriber(TOPIC_IR, Image, self.ir_callback)

        rospy.loginfo("画像保存ノード起動：リアルタイムで自動保存します")
        self.loop()

    def color_callback(self, msg):
        self.update_image("color", msg, "bgr8")

    def color_filtering_callback(self, msg):
        self.update_image("color_filtering", msg, "bgr8")

    def ir_filtering_callback(self, msg):
        self.update_image("ir_filtering", msg)

    def ir_callback(self, msg):
        self.update_image("ir", msg, "mono8")

    def update_image(self, key, msg, encoding=None):
        """ROS画像を変換して最新フレームを保持する。"""
        converted = self.bridge.imgmsg_to_cv2(msg, encoding) if encoding else self.bridge.imgmsg_to_cv2(msg)
        with self.lock:
            self.image_data[key] = converted

    def loop(self):
        """保存間隔に応じて保存処理を実行するメインループ。"""
        rate = rospy.Rate(LOOP_RATE_HZ)
        while not rospy.is_shutdown():
            now = time.time()
            if now - self.last_save_time >= self.save_interval_sec:
                self.save_all_images()
                self.last_save_time = now
            rate.sleep()

    def save_all_images(self):
        """4画像が揃っている場合のみ保存する。"""
        for directory in [SAVE_DIR_COLOR, SAVE_DIR_COLOR_FILTERING, SAVE_DIR_IR_FILTERING, SAVE_DIR_IR]:
            os.makedirs(directory, exist_ok=True)

        with self.lock:
            if any(v is None for v in self.image_data.values()):
                rospy.logwarn("すべての画像が揃っていません。保存をスキップします。")
                return

            base_name = f"{self.counter:04d}"
            cv2.imwrite(f"{SAVE_DIR_COLOR}/{base_name}_color.png", self.image_data["color"])
            cv2.imwrite(
                f"{SAVE_DIR_COLOR_FILTERING}/{base_name}_color_filtering.png",
                self.image_data["color_filtering"],
            )
            cv2.imwrite(
                f"{SAVE_DIR_IR_FILTERING}/{base_name}_ir_filtering.png",
                self.image_data["ir_filtering"],
            )
            cv2.imwrite(f"{SAVE_DIR_IR}/{base_name}_ir.png", self.image_data["ir"])

            rospy.loginfo(f"{base_name}*.png を保存しました")
            self.counter += 1


if __name__ == "__main__":
    try:
        MultiImageSaver()
    except rospy.ROSInterruptException:
        pass
