#!/usr/bin/env python3

import os
from datetime import datetime
import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

# ===== 設定（最初に編集する場所）=====
NODE_NAME = "dataset_maker"
SAVE_DIR = "/home/keisoku/1"
SAVE_INTERVAL_SEC = 0.05

# 赤外線画像を受信する入力トピック
INFRA_TOPIC = "/camera/infra/image_raw"
QUEUE_SIZE = 10

class DatasetMakerNode:
    """赤外線トピックを一定間隔で連番保存するノード。"""

    def __init__(self):
        rospy.init_node(NODE_NAME, anonymous=True)
        self.cv_bridge = CvBridge()

        self.infrared_sub_ = rospy.Subscriber(
            INFRA_TOPIC,
            Image,
            self.infrared_sub_callback,
            queue_size=QUEUE_SIZE,
        )

        self.save_dir = SAVE_DIR
        self.infrared_image_count = 1
        self.last_save_time_infrared = datetime.now()
        os.makedirs(self.save_dir, exist_ok=True)

    def infrared_sub_callback(self, msg):
        self.save_callback(
            msg, "Infra_frame", self.infrared_image_count, self.last_save_time_infrared
        )

    def save_callback(self, msg, prefix, count, last_save_time):
        current_time = datetime.now()
        elapsed_time = (current_time - last_save_time).total_seconds()

        if elapsed_time >= SAVE_INTERVAL_SEC:
            try:
                cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
                self.save_image(cv_image, prefix, count, current_time)
            except CvBridgeError as error:
                rospy.logerr("cv_bridge exception: %s", str(error))

    def save_image(self, image, prefix, count, current_time):
        save_path = os.path.join(self.save_dir, f"{count}.png")
        cv2.imwrite(save_path, image)
        rospy.loginfo(f"{prefix}を {count} に保存しました")
        if "Infra" in prefix:
            self.infrared_image_count += 1
            self.last_save_time_infrared = current_time

def main():
    _node = DatasetMakerNode()
    rospy.spin()

if __name__ == "__main__":
    main()
