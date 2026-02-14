#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from cv_bridge import CvBridge, CvBridgeError
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox
from geometry_msgs.msg import Point, TransformStamped
from sensor_msgs.msg import Image
import tf2_ros

# ===== 設定（最初に編集する場所）=====
NODE_NAME = "Tfpoint_detector_node"
PUB_THRESHOLD_PARAM = "~pub_threshold"
DEFAULT_PUB_THRESHOLD = 0.20
TARGET_CLASS = "cup"

# 入力トピック
BBOX_TOPIC = "/darknet_ros/bounding_boxes"  # YOLO検出結果（bbox）を受信する入力トピック
DEPTH_TOPIC = "/camera/aligned_depth_to_color/image_raw"  # color座標系に整列済みdepth入力トピック

# 出力トピック
POINT_TOPIC = "Topic_tf_point"  # 推定3D座標(Point)を配信する出力トピック
PUBLISH_QUEUE_SIZE = 1

# 投影計算パラメータ（depth camera info固定値）
CX = 320.6761474609375
CY = 250.00796508789062
FX = 603.1981201171875
FY = 603.3027954101562
Y_CORRECTION_RATIO = 0.2

# 検出有効範囲
MIN_Z_M = 0.40
MAX_Z_M = 2.0
MIN_X_M = -0.4
MAX_X_M = 0.4

# TF設定
PARENT_FRAME_ID = "camera_color_optical_frame"
CHILD_FRAME_ID = "target_frame"

class Tfpoint_Detector:
    """bbox中心から3D座標を計算し、PointとTFを配信する。"""

    def __init__(self):
        rospy.init_node(NODE_NAME)
        self.bbox = BoundingBox()
        self.cv_bridge = CvBridge()
        self.pub_threshold = rospy.get_param(PUB_THRESHOLD_PARAM, DEFAULT_PUB_THRESHOLD)
        self.pub = rospy.Publisher(POINT_TOPIC, Point, queue_size=PUBLISH_QUEUE_SIZE)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.class_name = ""

        self.cam_x = 0.0
        self.cam_y = 0.0
        self.bbox_depth = 0.0
        self.x1 = 0.0
        self.y1 = 0.0
        self.z = 0.0

        self.sub_head_bbox = rospy.Subscriber(
            BBOX_TOPIC, BoundingBoxes, self.head_bbox_callback
        )
        self.sub_cam_depth = rospy.Subscriber(DEPTH_TOPIC, Image, self.depth_callback)

    def head_bbox_callback(self, darknet_bboxs):
        """対象クラスbboxの中心画素座標を更新する。"""
        bbox = BoundingBox()
        head_bboxs = darknet_bboxs.bounding_boxes
        if head_bboxs:
            for det_bbox in head_bboxs:
                if (
                    det_bbox.Class == TARGET_CLASS
                    and det_bbox.probability >= self.pub_threshold
                ):
                    bbox = det_bbox
                    self.class_name = bbox.Class

                    self.cam_x = bbox.xmin + (bbox.xmax - bbox.xmin) / 2
                    self.cam_y = bbox.ymin + (bbox.ymax - bbox.ymin) / 2

    def depth_callback(self, depth_image_data):
        """depth画像からbbox中心の距離を取得し、座標変換を実行する。"""
        try:
            self.depth_image = self.cv_bridge.imgmsg_to_cv2(depth_image_data, "passthrough")
            depth_x = int(self.cam_x)
            depth_y = int(self.cam_y)
            h, w = self.depth_image.shape[:2]
            if not (0 <= depth_x < w and 0 <= depth_y < h):
                return

            self.bbox_depth = self.depth_image[depth_y, depth_x]
            self.camera_info_callback()

        except CvBridgeError as error:
            rospy.logerr("CvBridge Error: %s", error)

    def camera_info_callback(self):
        """固定内部パラメータで3D座標へ変換し、範囲内なら配信する。"""
        if self.cam_x != 0.0 and self.cam_y != 0.0 or self.cam_x > self.cam_y:
            try:
                x = self.cam_x - CX
                y = self.cam_y - CY
                self.z = self.bbox_depth

                self.x1 = self.z * x / FX
                self.y1 = self.z * y / FY

                self.x1 = self.x1 * 0.001
                self.y1 = self.y1 * 0.001 + Y_CORRECTION_RATIO * self.y1 * 0.001
                self.z = self.z * 0.001
            except Exception as error:
                rospy.logwarn("Transform Error: %s", str(error))
                return

            if MIN_Z_M < self.z <= MAX_Z_M:
                if MIN_X_M < self.x1 < MAX_X_M:
                    self.coordinate_point_callback()
                    point_msg = Point()
                    point_msg.x = self.x1
                    point_msg.y = self.y1
                    point_msg.z = self.z
                    self.pub.publish(point_msg)
                    # rospy.loginfo("x =%.2f y=%.2f z=%.2f ", self.x1, self.y1, self.z)
                else:
                    rospy.logwarn("Out of range_X")
            else:
                rospy.logwarn("Not Detection --> Distance over z=%.2f ", self.z)
                self.cam_x = 0.0
                self.cam_y = 0.0
                self.x1 = 0.0
                self.y1 = 0.0
        else:
            rospy.logwarn("Not Detection")

    def coordinate_point_callback(self):
        """現在の推定座標をTFとして配信する。"""
        try:
            gt = TransformStamped()
            gt.header.stamp = rospy.Time.now()
            gt.header.frame_id = PARENT_FRAME_ID
            gt.child_frame_id = CHILD_FRAME_ID
            gt.transform.translation.x = self.x1
            gt.transform.translation.y = self.y1
            gt.transform.translation.z = self.z

            gt.transform.rotation.x = 0.0
            gt.transform.rotation.y = 0.0
            gt.transform.rotation.z = 0.0
            gt.transform.rotation.w = 1.0

            self.tf_broadcaster.sendTransform(gt)
            rospy.loginfo(
                "class:%s x = %.2f y = %.2f z = %.2f --> tf_publish",
                self.class_name,
                self.x1,
                self.y1,
                self.z,
            )
        except Exception as error:
            rospy.logerr("Unable to create tf: %s", str(error))


def main():
    Tfpoint_Detector()
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
