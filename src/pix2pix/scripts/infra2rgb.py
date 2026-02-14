#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
"""赤外線グレースケール画像を RGB 画像としてパブリッシュする。"""

# ===== 設定（必要ならここを変更）=====
NODE_NAME = "infra_to_rgb_converter"
INPUT_TOPIC = "/camera/infra/image_raw"  # 生赤外画像（mono8）を受信する入力トピック
OUTPUT_TOPIC = "/z/infra2rgb/image_raw"  # RGB化した赤外画像を配信する出力トピック
INPUT_ENCODING = "mono8"
OUTPUT_ENCODING = "rgb8"
PUBLISH_QUEUE_SIZE = 10

class InfraToRGBConverter:

    def __init__(self):
        self.bridge = CvBridge()
        self.infra_sub = rospy.Subscriber(INPUT_TOPIC, Image, self.infra_callback)
        self.rgb_pub = rospy.Publisher(
            OUTPUT_TOPIC, Image, queue_size=PUBLISH_QUEUE_SIZE
        )
        rospy.loginfo("Subscribed: %s", INPUT_TOPIC)
        rospy.loginfo("Publishing: %s", OUTPUT_TOPIC)

    def infra_callback(self, msg):
        """受信した mono8 画像を rgb8 に変換して publish する。"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding=INPUT_ENCODING)
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2RGB)
            rgb_msg = self.bridge.cv2_to_imgmsg(rgb_image, encoding=OUTPUT_ENCODING)
            rgb_msg.header = msg.header  # タイムスタンプと frame_id を引き継ぐ
            self.rgb_pub.publish(rgb_msg)
        except CvBridgeError as error:
            rospy.logerr("CvBridgeError: %s", error)


def main():
    rospy.init_node(NODE_NAME, anonymous=True)
    _converter = InfraToRGBConverter()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")


if __name__ == "__main__":
    main()
