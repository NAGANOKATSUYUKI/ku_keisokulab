#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

# # 画像の読み込み
# image_path = "/home/keisoku/catkin_ws/photos/realsense_viewer.png"

# image = cv2.imread(image_path)

# cv2.imshow("image",image)
# cv2.waitKey(0)
# cv2.destroyAllWindows()


def image_callback(msg):
    try:
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

        # height,width = cv_image.shape[:2]
        # new_width = 640  
        # new_height = int(height * (new_width / width))
        # cv_image_resized = cv2.resize(cv_image, (new_width, new_height))
        
        cv2.imshow("Image", cv_image)
        cv2.waitKey(1)
        
    except Exception as e:
        print(e)

def main():
    rospy.init_node('Image')
    image_subscriber = rospy.Subscriber('/camera/color/image_raw', Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
