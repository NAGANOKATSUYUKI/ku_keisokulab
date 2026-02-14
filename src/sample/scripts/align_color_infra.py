#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""RealSenseのトピックを受信し、infra をクロップしてパブッシュしつつ表示するノード。"""

import cv2
import threading
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

# ===== 設定（必要ならここを変更）=====
NODE_NAME = "realsense_viewer"

COLOR_SUB_TOPIC = "/camera/color/image_raw"  # 生カラー画像を受信する入力トピック
INFRA_SUB_TOPIC = "/camera/infra/image_raw"  # 生赤外画像を受信する入力トピック
DEPTH_SUB_TOPIC = "/camera/aligned_depth_to_color/image_raw"  # color基準に整列済み深度画像の入力トピック

COLOR_PUB_TOPIC = "/z/color/image"  # 処理後カラー画像を再配信する出力トピック
INFRA_PUB_TOPIC = "/z/infra/image"  # クロップ後赤外画像を再配信する出力トピック
DEPTH_PUB_TOPIC = "/z/depth/image"  # 処理後深度画像を再配信する出力トピック
PUBLISH_QUEUE_SIZE = 10

COLOR_ENCODING = "bgr8"
INFRA_ENCODING = "mono8"
DEPTH_ENCODING = "passthrough"

# Infra 画像の切り出し範囲
CROP_X_START = 90
CROP_Y_START = 50
CROP_WIDTH = 490
CROP_HEIGHT = 345

COLOR_WINDOW_NAME = "Realsense L515 - Color"
INFRA_WINDOW_NAME = "Realsense L515 - Infrared"
DEPTH_WINDOW_NAME = "Realsense L515 - Depth"


class RealsenseViewer:
    """RealSense の 3系統画像を扱うビューア兼中継クラス。"""

    def __init__(self):
        rospy.init_node(NODE_NAME, anonymous=True)
        self.bridge = CvBridge()
        self.lock = threading.Lock()

        self.color_image = None
        self.infra_image = None
        self.depth_image = None

        self.color_subscription = rospy.Subscriber(
            COLOR_SUB_TOPIC, Image, self.color_callback
        )
        self.infra_subscription = rospy.Subscriber(
            INFRA_SUB_TOPIC, Image, self.infra_callback
        )
        self.depth_subscription = rospy.Subscriber(
            DEPTH_SUB_TOPIC, Image, self.depth_callback
        )

        self.color_publisher = rospy.Publisher(
            COLOR_PUB_TOPIC, Image, queue_size=PUBLISH_QUEUE_SIZE
        )
        self.infra_publisher = rospy.Publisher(
            INFRA_PUB_TOPIC, Image, queue_size=PUBLISH_QUEUE_SIZE
        )
        self.depth_publisher = rospy.Publisher(
            DEPTH_PUB_TOPIC, Image, queue_size=PUBLISH_QUEUE_SIZE
        )

        self.crop_x_start = CROP_X_START
        self.crop_y_start = CROP_Y_START
        self.crop_width = CROP_WIDTH
        self.crop_height = CROP_HEIGHT

        self.display_thread = threading.Thread(target=self.display_images)
        self.display_thread.start()

    def publish_with_header(self, publisher, cv_image, encoding, header):
        """OpenCV 画像を ROS Image に変換して header を維持したまま publish する。"""
        processed_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding)
        processed_msg.header = header
        publisher.publish(processed_msg)

    def color_callback(self, msg):
        rospy.loginfo("Receiving color video frame")
        cv_image = self.bridge.imgmsg_to_cv2(msg, COLOR_ENCODING)

        with self.lock:
            self.color_image = cv_image

        self.publish_with_header(self.color_publisher, cv_image, COLOR_ENCODING, msg.header)

    def infra_callback(self, msg):
        rospy.loginfo("Receiving infrared video frame")
        cv_image = self.bridge.imgmsg_to_cv2(msg, INFRA_ENCODING)

        cropped_image = cv_image[
            self.crop_y_start : self.crop_y_start + self.crop_height,
            self.crop_x_start : self.crop_x_start + self.crop_width,
        ]

        with self.lock:
            self.infra_image = cropped_image

        self.publish_with_header(
            self.infra_publisher, cropped_image, INFRA_ENCODING, msg.header
        )

    def depth_callback(self, msg):
        rospy.loginfo("Receiving depth video frame")
        cv_image = self.bridge.imgmsg_to_cv2(msg, DEPTH_ENCODING)

        with self.lock:
            self.depth_image = cv_image

        self.publish_with_header(self.depth_publisher, cv_image, DEPTH_ENCODING, msg.header)

    def display_images(self):
        while not rospy.is_shutdown():
            with self.lock:
                if self.color_image is not None:
                    cv2.imshow(COLOR_WINDOW_NAME, self.color_image)
                if self.infra_image is not None:
                    cv2.imshow(INFRA_WINDOW_NAME, self.infra_image)
                if self.depth_image is not None:
                    cv2.imshow(DEPTH_WINDOW_NAME, self.depth_image)
            cv2.waitKey(1)


def main():
    _viewer = RealsenseViewer()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
    finally:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
