#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
from datetime import datetime, timedelta

class DatasetMakerNode:

    def __init__(self):
        self.node_name = 'dataset_maker'
        rospy.init_node(self.node_name, anonymous=True)

        self.infrared_sub_ = rospy.Subscriber(
            '/camera/infra/image_raw',
            Image,
            self.infrared_sub_callback,
            queue_size=10
        )
        
        self.cv_bridge = CvBridge()
        # self.save_dir = "/home/keisoku/Robot_data/fake/infra_align_512/edgenasi"
        self.save_dir = "/home/keisoku/1"
        self.infrared_image_count = 1  # 赤外線画像の保存された数
        self.last_save_time_infrared = datetime.now()
        self.last_save_time_color = datetime.now()

        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

    def infrared_sub_callback(self, msg):
        self.save_callback(msg, "Infra_frame", self.infrared_image_count, self.last_save_time_infrared)

    

    def save_callback(self, msg, prefix, count, last_save_time):
        current_time = datetime.now()
        elapsed_time = (current_time - last_save_time).total_seconds()

        if elapsed_time >= 0.05:  # 0.05秒ごとに保存（必要に応じて調整）
            try:
                cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
                self.save_image(cv_image, prefix, count, current_time)
            except CvBridgeError as e:
                rospy.logerr("cv_bridge exception: %s" % str(e))

    def save_image(self, image, prefix, count, current_time):
        save_path = os.path.join(self.save_dir, f"{count}.png")
        cv2.imwrite(save_path, image)
        rospy.loginfo(f"{prefix}を {count} に保存しました")
        # 画像数を増やす
        if "Infra" in prefix:
            self.infrared_image_count += 1
            self.last_save_time_infrared = current_time
        

def main():
    node = DatasetMakerNode()
    rospy.spin()

if __name__ == '__main__':
    main()
