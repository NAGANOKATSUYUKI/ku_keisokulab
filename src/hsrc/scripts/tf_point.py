#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
import numpy as np
from geometry_msgs.msg import Point, TransformStamped
from sensor_msgs.msg import CameraInfo

class Tf_publish():
    def __init__(self):
        self.camera_coords = np.array([1.0, 1.0, 1.0])

        rospy.init_node("tf_pulish_node")
        rospy.Subscriber("point_topic", Point, self.CoordinatePoint_callback)
        rospy.Subscriber("/hsrb/head_rgbd_sensor/depth_registered/camera_info", CameraInfo, self.CameraInfo_callback)

    #tfの作成
    def tf_publish(self):
        tf_broadcaster = tf2_ros.TransformBroadcaster()

        gt = TransformStamped()
        gt.header.stamp = rospy.Time.now()
        gt.header.frame_id = "head_rgbd_sensor_link"#原点
        gt.child_frame_id = "target_frame"#対象
        gt.transform.translation.x = self.camera_coords[0]
        gt.transform.translation.y = self.camera_coords[1]
        gt.transform.translation.z = self.camera_coords[2]
        #対象の座標軸設定
        gt.transform.rotation.w = 1.0
        gt.transform.rotation.x = 0.0
        gt.transform.rotation.y = 0.0
        gt.transform.rotation.z = 0.0

        #
        tf_broadcaster.sendTransform(gt)

    #camera_info取得
    def CameraInfo_callback(self, info_msg):
        self.caminfo = np.linalg.inv(np.array([[info_msg.K[0],info_msg.K[1],info_msg.K[2]],
                                               [info_msg.K[3],info_msg.K[4],info_msg.K[5]],
                                               [info_msg.K[6],info_msg.K[7],info_msg.K[8]]]))
        
    #座標の指定
    def CoordinatePoint_callback(self, coordinate_msg):
        try:
            self.normalized_coords = np.dot(self.caminfo, np.array([coordinate_msg.x, coordinate_msg.y, 1.0]))#内積
            self.camera_coords = self.normalized_coords * coordinate_msg.z *0.001#カメラ座標ベクトル

            self.tf_publish()
            # rospy.loginfo("tf_publish --> OK")
            rospy.loginfo("x=%s",self.camera_coords)
        except :
            rospy.loginfo("Unable to create tf")

if __name__=="__main__":
    try:
        Tf_publish()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass