#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Pointトピックの座標をTFとして配信するノード。"""

import rospy
import tf2_ros
import numpy as np
from geometry_msgs.msg import Point, TransformStamped

# ===== 設定（最初に編集する場所）=====
NODE_NAME = "tf_pulish_node"

# 入力トピック
POINT_TOPIC = "point_topic"  # 3D座標(Point)を受信する入力トピック

# TFフレーム名
PARENT_FRAME_ID = "head_rgbd_sensor_link"  # 原点側フレーム
CHILD_FRAME_ID = "target_frame"  # 対象側フレーム

# 座標受理条件
MAX_Z_M = 1.3


class TfPublish:
    """受信PointをTF変換として配信するクラス。"""

    def __init__(self):
        self.camera_coords = np.array([1.0, 1.0, 1.0])
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        rospy.init_node(NODE_NAME)
        rospy.Subscriber(POINT_TOPIC, Point, self.coordinate_point_callback)

    def tf_publish(self):
        """現在の座標をTFとしてpublishする。"""
        gt = TransformStamped()
        gt.header.stamp = rospy.Time.now()
        gt.header.frame_id = PARENT_FRAME_ID
        gt.child_frame_id = CHILD_FRAME_ID
        gt.transform.translation.x = self.camera_coords[0]
        gt.transform.translation.y = self.camera_coords[1]
        gt.transform.translation.z = self.camera_coords[2]
        gt.transform.rotation.w = 1.0
        gt.transform.rotation.x = 0.0
        gt.transform.rotation.y = 0.0
        gt.transform.rotation.z = 0.0

        self.tf_broadcaster.sendTransform(gt)

    def coordinate_point_callback(self, coordinate_msg):
        """受信Pointを条件判定し、許可された座標のみTFへ反映する。"""
        try:
            if (
                coordinate_msg.x != 0.0
                and coordinate_msg.y != 0.0
                or coordinate_msg.x > coordinate_msg.y
            ):
                if coordinate_msg.z < MAX_Z_M:
                    self.camera_coords[0] = coordinate_msg.x
                    self.camera_coords[1] = coordinate_msg.y
                    self.camera_coords[2] = coordinate_msg.z

                    self.tf_publish()
                    rospy.loginfo(
                        "x= %0.1f, y= %0.1f, z=%0.1f",
                        self.camera_coords[0],
                        self.camera_coords[1],
                        self.camera_coords[2],
                    )
                else:
                    rospy.logwarn("tf_publish --> NG")
            else:
                rospy.logwarn("tf_publish --> NG")
        except Exception:
            rospy.loginfo("Unable to create tf")


def main():
    rospy.loginfo("Unable to create tf")
    TfPublish()
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
