#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Depthしきい値でColor画像をマスクし、フィルタ画像を配信するノード。"""

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer
from sensor_msgs.msg import Image

# ===== 設定（最初に編集する場所）=====
NODE_NAME = "color_depth_filter_node"

# depthとcolorを同期購読する入力トピック
DEPTH_TOPIC = "/camera/aligned_depth_to_color/image_raw"  # color座標系に整列済みdepth
COLOR_TOPIC = "/camera/color/image_raw"  # 生カラー画像

# フィルタ結果を配信する出力トピック
COLOR_FILTERED_TOPIC = "/z/color_image_filtered"  # depthマスク適用後のカラー画像
DEPTH_FILTERED_TOPIC = "/z/depth_image_filtered"  # 互換維持のため確保（現在未publish）

SYNC_QUEUE_SIZE = 10
SYNC_SLOP_SEC = 0.1
PUBLISH_QUEUE_SIZE = 1

DEFAULT_DEPTH_THRESHOLD_MM = 1000
DEFAULT_TARGET_X = 320
DEFAULT_TARGET_Y = 240

VIS_MIN_DEPTH_MM = 100
VIS_MAX_DEPTH_MM = 500

class DepthFilterNode:
    """Depthに基づくマスクでColor画像をフィルタする。"""

    def __init__(self):
        rospy.init_node(NODE_NAME)
        self.bridge = CvBridge()

        self.depth_sub = Subscriber(DEPTH_TOPIC, Image)
        self.color_sub = Subscriber(COLOR_TOPIC, Image)
        self.ts = ApproximateTimeSynchronizer(
            [self.depth_sub, self.color_sub],
            queue_size=SYNC_QUEUE_SIZE,
            slop=SYNC_SLOP_SEC,
        )
        self.ts.registerCallback(self.depth_callback)

        self.color_image_pub = rospy.Publisher(
            COLOR_FILTERED_TOPIC, Image, queue_size=PUBLISH_QUEUE_SIZE
        )
        self.depth_image_pub = rospy.Publisher(
            DEPTH_FILTERED_TOPIC, Image, queue_size=PUBLISH_QUEUE_SIZE
        )

        self.threshold_mm = rospy.get_param("~depth_threshold_mm", DEFAULT_DEPTH_THRESHOLD_MM)
        self.target_x = rospy.get_param("~target_x", DEFAULT_TARGET_X)
        self.target_y = rospy.get_param("~target_y", DEFAULT_TARGET_Y)

    def depth_callback(self, depth_msg, color_msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(
                depth_msg, desired_encoding="passthrough"
            )
            color_image = self.bridge.imgmsg_to_cv2(
                color_msg, desired_encoding="passthrough"
            )
        except Exception as error:
            rospy.logerr("CVBridge error: %s", error)
            return

        self.print_depth_at_pixel(depth_image)
        self.display_depth_grayscale(depth_image)

        depth_mask = self.get_depth_mask(depth_image)
        color_masked = cv2.bitwise_and(color_image, color_image, mask=depth_mask)

        color_msg_out = self.bridge.cv2_to_imgmsg(color_masked, encoding="rgb8")
        color_msg_out.header = color_msg.header
        self.color_image_pub.publish(color_msg_out)

    def get_depth_mask(self, depth_image):
        """有効深度（0より大きく、しきい値未満）を255とするマスクを返す。"""
        valid_mask = (depth_image > 0) & (depth_image < self.threshold_mm)
        return (valid_mask * 255).astype(np.uint8)

    def display_depth_grayscale(self, depth_image):
        """深度画像を可視化用グレースケールへ変換する（戻り値のみ使用）。"""
        try:
            depth_image = np.where(depth_image == 0, VIS_MAX_DEPTH_MM, depth_image)
            depth_image = np.where(
                depth_image > self.threshold_mm, VIS_MAX_DEPTH_MM, depth_image
            )

            clipped = np.clip(depth_image, VIS_MIN_DEPTH_MM, VIS_MAX_DEPTH_MM)
            normalized = (
                1.0
                - (clipped - VIS_MIN_DEPTH_MM) / (VIS_MAX_DEPTH_MM - VIS_MIN_DEPTH_MM)
            ) * 255.0
            gray = normalized.astype(np.uint8)
            return gray
        except Exception as error:
            rospy.logerr("Grayscale display error: %s", error)
            return None

    def print_depth_at_pixel(self, depth_image):
        """指定ピクセルの深度値(mm)をログ出力する。"""
        h, w = depth_image.shape
        if 0 <= self.target_x < w and 0 <= self.target_y < h:
            depth_value = depth_image[self.target_y, self.target_x]
            rospy.loginfo(
                "Pixel (%d, %d) Depth: %s mm",
                self.target_x,
                self.target_y,
                depth_value,
            )
        else:
            rospy.logwarn(
                "Target pixel (%d, %d) out of bounds", self.target_x, self.target_y
            )

def main():
    DepthFilterNode()
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    cv2.destroyAllWindows()
