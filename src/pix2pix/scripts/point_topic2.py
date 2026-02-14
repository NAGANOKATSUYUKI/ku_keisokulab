#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""YOLO bbox中心のdepthから3D座標を推定し、Pointを配信する。"""

import rospy
from cv_bridge import CvBridge, CvBridgeError
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox
from geometry_msgs.msg import Point
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image

# ===== 設定（最初に編集する場所）=====
NODE_NAME = "Point_topic"
PUB_THRESHOLD_PARAM = "~pub_threshold"
DEFAULT_PUB_THRESHOLD = 0.30
TARGET_CLASS = "bottle"

# 入力トピック（必要に応じて切り替え）
DEPTH_TOPIC = "/camera/aligned_depth_to_color/image_raw"  # color座標系に整列済みdepth
CAMERA_INFO_TOPIC = "/camera/color/camera_info"  # colorカメラ内部パラメータ
BBOX_TOPIC = "/darknet_ros/bounding_boxes"  # YOLO検出結果（bbox）

# 出力トピック
POINT_TOPIC = "point_topic"  # 推定3D座標(Point)
PUBLISH_QUEUE_SIZE = 10


class Detector:
    """bbox中心点を3D座標化してPoint配信するクラス。"""

    def __init__(self):
        rospy.init_node(NODE_NAME)
        self.cv_bridge = CvBridge()
        self.bbox = BoundingBox()
        self.m_pub_threshold = rospy.get_param(
            PUB_THRESHOLD_PARAM, DEFAULT_PUB_THRESHOLD
        )

        self.cam_x = 0.0
        self.cam_y = 0.0
        self.bbox_depth = 0.0

        self.pub = rospy.Publisher(POINT_TOPIC, Point, queue_size=PUBLISH_QUEUE_SIZE)
        self.sub_cam_depth = rospy.Subscriber(DEPTH_TOPIC, Image, self.depth_callback)
        self.sub_cam_info = rospy.Subscriber(
            CAMERA_INFO_TOPIC, CameraInfo, self.camera_info_callback
        )
        self.sub_darknet_bbox = rospy.Subscriber(
            BBOX_TOPIC, BoundingBoxes, self.darknet_bbox_callback
        )

    def darknet_bbox_callback(self, darknet_bboxs):
        """YOLO bboxから対象クラスの中心座標を更新する。"""
        bboxs = darknet_bboxs.bounding_boxes
        bbox = BoundingBox()
        if len(bboxs) != 0:
            for det_bbox in bboxs:
                if (
                    det_bbox.Class == TARGET_CLASS
                    and det_bbox.probability >= self.m_pub_threshold
                ):
                    bbox = det_bbox

        self.bbox = bbox

        self.cam_x = self.bbox.xmin + (self.bbox.xmax - self.bbox.xmin) / 2
        self.cam_y = self.bbox.ymin + (self.bbox.ymax - self.bbox.ymin) / 2

    def depth_callback(self, depth_image_data):
        """bbox中心のdepth値を更新する。"""
        try:
            self.depth_image = self.cv_bridge.imgmsg_to_cv2(depth_image_data, "passthrough")
            depth_x = int(self.cam_x)
            depth_y = int(self.cam_y)
            h, w = self.depth_image.shape[:2]
            if 0 <= depth_x < w and 0 <= depth_y < h:
                self.bbox_depth = self.depth_image[depth_y][depth_x]
            else:
                self.bbox_depth = 0.0

        except CvBridgeError as error:
            rospy.logerr("CvBridge error: %s", error)

    def camera_info_callback(self, info_msg):
        """画素座標 + depth から3D座標を計算して配信する。"""
        if not (self.cam_x != 0.0 and self.cam_y != 0.0):
            rospy.logwarn("Not Detection")
            return

        try:
            x = self.cam_x - info_msg.K[2]
            y = self.cam_y - info_msg.K[5]
            z = self.bbox_depth

            x1 = z * x / info_msg.K[0]
            y1 = z * y / info_msg.K[4]

            x1 = x1 * 0.001
            y1 = y1 * 0.001
            z = z * 0.001
            rospy.loginfo("x =%.2f, y=%.2f, z=%.2f ", x1, y1, z)
        except Exception:
            rospy.logwarn("transform_ERROR")
            return

        try:
            point_msg = Point()
            point_msg.x = x1
            point_msg.y = y1
            point_msg.z = z
            self.pub.publish(point_msg)
        except Exception:
            rospy.logwarn("Publish_ERROR")


def main():
    rospy.loginfo("Unable to create tf")
    Detector()
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
