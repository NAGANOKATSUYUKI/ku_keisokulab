#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
import numpy as np
from geometry_msgs.msg import Point, TransformStamped
from sensor_msgs.msg import CameraInfo

class Tf_publish(object):
    def __init__(self):
        self.camera_coords = np.array([1.0, 1,0, 1.0])

        rospy.init_node('tf_publish_node')
        rospy.Subscriber("point_topic", Point, self.coordinate_callback)
        rospy.Subscriber("/hsrb/head_rgbd_sensor/depth_registered/camera_info", CameraInfo, self.camerainfo_callback)
        #rospy.Subscriber("camera/aligned_depth_to_color/camera_info", CameraInfo, self.camerainfo_callback)
        rospy.loginfo("tf_publish --> OK")


    def coordinate_callback(self, msg):
        self.normalized_coords = np.dot(self.K_inv, np.array([msg.x, msg.y, 1.0]))#内積
        self.camera_coords = self.normalized_coords * msg.z *0.001#カメラ座標ベクトル

        self.tf_publisher()


    def camerainfo_callback(self, msg):#カメラインフォ取得
        self.K_inv = np.linalg.inv(np.array([[msg.K[0], msg.K[1], msg.K[2]],
                                             [msg.K[3], msg.K[4], msg.K[5]],
                                             [msg.K[6], msg.K[7], msg.K[8]]]))


    def tf_publisher(self):
        # Create a transform message
        tf_broadcaster = tf2_ros.TransformBroadcaster()

        gt = TransformStamped()
        gt.header.stamp = rospy.Time.now()
        gt.header.frame_id = "head_rgbd_sensor_link"   #map , head_rgbd_sensor_link#原点
        gt.child_frame_id = "target_frame"#対象
        gt.transform.translation.x = self.camera_coords[0]  # self.camera_coords[2]
        gt.transform.translation.y = self.camera_coords[1]  # -self.camera_coords[0]
        gt.transform.translation.z = self.camera_coords[2]  # -self.camera_coords[1]
        #対象の座標軸設定
        gt.transform.rotation.w = 1.0
        gt.transform.rotation.x = 0.0
        gt.transform.rotation.y = 0.0
        gt.transform.rotation.z = 0.0

        # Publish the transform
        tf_broadcaster.sendTransform(gt)


if __name__ == '__main__':
    Tf_publish()
    rospy.spin()