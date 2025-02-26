#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image, CameraInfo
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox
from cv_bridge import CvBridge, CvBridgeError
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped

class Tfpoint_Detector:
    def __init__(self):
        rospy.init_node("Tfpoint_detector_node")
        self.bbox = BoundingBox()
        self.cv_bridge = CvBridge()
        self.pub_threshold = rospy.get_param("~pub_threshold", 0.20)
        self.pub = rospy.Publisher("Topic_tf_point", Point, queue_size=10)

        # average variables
        self.sum_x = 0
        self.sum_y = 0
        self.sum_z = 0
        self.cam_x = 0.0
        self.cam_y = 0.0

        # detection subscription
        self.sub_head_bbox = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.HeadBboxCallback)
        self.sub_cam_depth     = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.DepthCallback)
        # self.sub_cam_depth = rospy.Subscriber("/hsrb/head_rgbd_sensor/depth_registered/image_raw", Image, self.DepthCallback)
        # self.sub_camera_info = rospy.Subscriber("/hsrb/head_rgbd_sensor/depth_registered/camera_info", CameraInfo, self.CameraInfoCallback)

    def HeadBboxCallback(self, darknet_bboxs):
        bbox = BoundingBox()
        head_bboxs = darknet_bboxs.bounding_boxes
        if head_bboxs:
            for i in range(len(head_bboxs)):
                if head_bboxs[i].Class == "bottle" and head_bboxs[i].probability >= self.pub_threshold:
                    bbox = head_bboxs[i]
                    self.class_name = bbox.Class

                    #中心座標
                    self.cam_x = bbox.xmin + (bbox.xmax - bbox.xmin) / 2
                    self.cam_y = bbox.ymin + (bbox.ymax - bbox.ymin) / 2
                else:
                    pass
        else:
            pass        

    def DepthCallback(self, depth_image_data):
        try:
            self.depth_image = self.cv_bridge.imgmsg_to_cv2(depth_image_data, "passthrough")
            depth_x = int(self.cam_x)
            depth_y = int(self.cam_y)
            self.bbox_depth = self.depth_image[depth_y, depth_x]
            
            self.CameraInfoCallback()

        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: %s", e)

    #座標変換
    def CameraInfoCallback(self):
        if self.cam_x != 0.0 and self.cam_y != 0.0 or self.cam_x > self.cam_y:
            try:
                self.x = self.cam_x - 320.7234912485017  #depth_infoの値を入力
                self.y = self.cam_y - 242.2827148742709
                self.z = self.bbox_depth

                self.x1 = self.z * self.x / 533.8084805746594
                self.y1 = self.z * self.y / 534.057713759154

                self.x1 = self.x1 * 0.001
                self.y1 = self.y1 * 0.001 + 0.2 * self.y1 *0.001
                self.z = self.z * 0.001
            except Exception as e:
                rospy.logwarn("Transform Error: %s", str(e))

            if 0.40 < self.z <= 2 :
                if -0.4 < self.x1 < 0.4:
                    self.CoordinatePointCallback()
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
            # rospy.loginfo("x =%.2f y=%.2f z=%.2f ", self.cam_x, self.cam_y, self.z)

    def CoordinatePointCallback(self):
        try:
            static_tf_broadcaster = tf2_ros.TransformBroadcaster()

            gt = TransformStamped()
            gt.header.stamp = rospy.Time.now()
            gt.header.frame_id = "camera_color_optical_frame"
            gt.child_frame_id = "target_frame"
            gt.transform.translation.x = self.x1
            gt.transform.translation.y = self.y1
            gt.transform.translation.z = self.z

            gt.transform.rotation.x = 0.0
            gt.transform.rotation.y = 0.0
            gt.transform.rotation.z = 0.0
            gt.transform.rotation.w = 1.0

            static_tf_broadcaster.sendTransform(gt)
            rospy.loginfo("class:%s x = %.2f y = %.2f z = %.2f --> tf_publish", self.class_name, self.x1, self.y1, self.z)
        except Exception as e:
            rospy.logerr("Unable to create tf: %s", str(e))

if __name__ == '__main__':
    try:
        
        Tfpoint_Detector()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
