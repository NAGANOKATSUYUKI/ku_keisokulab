#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
import numpy as np
from geometry_msgs.msg import Point, TransformStamped

class Tf_publish():
    def __init__(self):
        self.camera_coords = np.array([1.0, 1.0, 1.0])

        rospy.init_node("tf_pulish_node")
        rospy.Subscriber("point_topic", Point, self.CoordinatePoint_callback)

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
        
    #座標の指定
    def CoordinatePoint_callback(self, coordinate_msg):
        try:
            if coordinate_msg.x != 0.0 and coordinate_msg.y != 0.0 or coordinate_msg.x > coordinate_msg.y :
                if coordinate_msg.z < 1.3 :
                    self.camera_coords[0] =  coordinate_msg.x
                    self.camera_coords[1] =  coordinate_msg.y
                    self.camera_coords[2] =  coordinate_msg.z

                    self.tf_publish()
                    # rospy.loginfo("tf_publish --> OK")
                    rospy.loginfo("x= %0.1f, y= %0.1f, z=%0.1f", self.camera_coords[0],self.camera_coords[1], self.camera_coords[2])
                else:
                    rospy.logwarn("tf_publish --> NG")
            else:
                rospy.logwarn("tf_publish --> NG")
        except :
            rospy.loginfo("Unable to create tf")

if __name__=="__main__":
    rospy.loginfo("Unable to create tf")
    try:
        
        Tf_publish()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass


#1113