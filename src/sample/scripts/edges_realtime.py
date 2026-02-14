#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

# ===== 設定（最初に編集する場所）=====
NODE_NAME = "image_processor_node"
INPUT_TOPIC = "/camera/infra/image_raw"
OUTPUT_TOPIC = "/z/edge_image"
PUBLISH_QUEUE_SIZE = 1

INPUT_ENCODING = "rgb8"
OUTPUT_ENCODING = "bgr8"

CROP_REGION = (0, 0, 640, 480)  # x, y, width, height
RESIZE_DIM = (640, 480)  # width, height
IMAGE_WEIGHT = 0.85
EDGE_WEIGHT = 0.15
CANNY_THRESHOLD_1 = 20
CANNY_THRESHOLD_2 = 30


class ImageProcessor:
    """受信画像にエッジを重畳して publish する。"""

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(INPUT_TOPIC, Image, self.image_callback)
        self.image_pub = rospy.Publisher(
            OUTPUT_TOPIC, Image, queue_size=PUBLISH_QUEUE_SIZE
        )
        self.crop_region = CROP_REGION
        self.resize_dim = RESIZE_DIM
        self.image_weight = IMAGE_WEIGHT
        self.edge_weight = EDGE_WEIGHT

    def image_callback(self, data):
        """ROS Image を処理し、エッジ重畳画像として publish する。"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding=INPUT_ENCODING)
        except CvBridgeError as error:
            rospy.logerr("Could not convert from ROS Image to OpenCV Image: %s", error)
            return

        cv_image = self.crop_image(cv_image)
        if cv_image is None:
            return

        cv_image = self.resize_image(cv_image)
        result_image = self.add_edge_overlay(
            cv_image, self.image_weight, self.edge_weight
        )

        try:
            image_msg = self.bridge.cv2_to_imgmsg(result_image, OUTPUT_ENCODING)
            image_msg.header = data.header
            self.image_pub.publish(image_msg)
        except CvBridgeError as error:
            rospy.logerr("Could not convert from OpenCV Image to ROS Image: %s", error)

    def crop_image(self, image):
        """画像の一部をトリミングする。"""
        x, y, w, h = self.crop_region
        ih, iw = image.shape[:2]
        if x + w <= iw and y + h <= ih:
            return image[y : y + h, x : x + w]

        rospy.logwarn("Crop region exceeds image bounds")
        return None

    def resize_image(self, image):
        """画像をリサイズする。"""
        return cv2.resize(image, self.resize_dim)

    def add_edge_overlay(self, image, image_weight, edge_weight):
        """エッジ検出して元画像と合成する。"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, CANNY_THRESHOLD_1, CANNY_THRESHOLD_2)
        edges_color = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        combined = cv2.addWeighted(image, image_weight, edges_color, edge_weight, 0)
        return combined


def main():
    rospy.init_node(NODE_NAME, anonymous=True)
    _processor = ImageProcessor()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
