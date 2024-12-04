#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading

class RealsenseViewer:
    def __init__(self):
        rospy.init_node('realsense_viewer', anonymous=True)
        self.bridge = CvBridge()

        # Color 
        self.color_subscription = rospy.Subscriber(
            '/camera/color/image_raw',
            Image, self.color_callback)
        # Infrared 
        self.infra_subscription = rospy.Subscriber(
            '/camera/infra/image_raw',
            Image, self.infra_callback)
        # Depth
        self.depth_subscription = rospy.Subscriber(
            '/camera/aligned_depth_to_color/image_raw',
            Image, self.depth_callback)

        # Publishers
        self.color_publisher = rospy.Publisher('/z/color/image', Image, queue_size=10)
        self.infra_publisher = rospy.Publisher('/z/infra/image', Image, queue_size=10)
        self.depth_publisher = rospy.Publisher('/z/depth/image', Image, queue_size=10)

        # Define cropping coordinates for infrared image
        # self.crop_x_start = 0
        # self.crop_y_start = 0
        # self.crop_width = 1024
        # self.crop_height = 768
        self.crop_x_start = 90
        self.crop_y_start = 50
        self.crop_width = 490
        self.crop_height = 345

        # Create thread for displaying images
        self.display_thread = threading.Thread(target=self.display_images)
        self.display_thread.start()

        self.color_image = None
        self.infra_image = None
        self.depth_image = None
        self.lock = threading.Lock()

    def color_callback(self, msg):
        rospy.loginfo('Receiving color video frame')
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        with self.lock:
            self.color_image = cv_image
            
        processed_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        processed_msg.header = msg.header
        self.color_publisher.publish(processed_msg)
    
    def infra_callback(self, msg):
        rospy.loginfo('Receiving infrared video frame')
        cv_image = self.bridge.imgmsg_to_cv2(msg, "mono8")
        
        cropped_image = cv_image[self.crop_y_start:self.crop_y_start+self.crop_height,
                                 self.crop_x_start:self.crop_x_start+self.crop_width]
        with self.lock:
            self.infra_image = cropped_image
        processed_msg = self.bridge.cv2_to_imgmsg(cropped_image, "mono8")
        processed_msg.header = msg.header
        self.infra_publisher.publish(processed_msg)
        
    def depth_callback(self, msg):
        rospy.loginfo('Receiving depth video frame')
        cv_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        with self.lock:
            self.depth_image = cv_image
            
        processed_msg = self.bridge.cv2_to_imgmsg(cv_image, "passthrough")
        processed_msg.header = msg.header
        self.depth_publisher.publish(processed_msg)
    
    def display_images(self):
        while not rospy.is_shutdown():
            with self.lock:
                if self.color_image is not None:
                    cv2.imshow("Realsense L515 - Color", self.color_image)
                if self.infra_image is not None:
                    cv2.imshow("Realsense L515 - Infrared", self.infra_image)
                if self.depth_image is not None:
                    cv2.imshow("Realsense L515 - Depth", self.depth_image)
            cv2.waitKey(1)

def main():
    viewer = RealsenseViewer()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
    finally:
        cv2.destroyAllWindows()  # Ensure that all OpenCV windows are closed

if __name__ == '__main__':
    main()
