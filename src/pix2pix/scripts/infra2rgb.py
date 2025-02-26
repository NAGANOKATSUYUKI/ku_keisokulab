#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class InfraToRGBConverter:
    def __init__(self):
        self.bridge = CvBridge()
        self.infra_sub = rospy.Subscriber('/camera/infra/image_raw', Image, self.infra_callback)
        # self.infra_sub = rospy.Subscriber('/z/image2movie_infra', Image, self.infra_callback)
        
        self.rgb_pub = rospy.Publisher('/z/infra2rgb/image_raw', Image, queue_size=10)

    def infra_callback(self, msg):
        try:
            # Convert the ROS Image message to a CV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")

            # Convert the grayscale image to RGB
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2RGB)

            # Convert the CV image back to a ROS Image message
            rgb_msg = self.bridge.cv2_to_imgmsg(rgb_image, encoding="rgb8")

            # Publish the RGB image
            self.rgb_pub.publish(rgb_msg)
        except CvBridgeError as e:
            rospy.logerr("CvBridgeError: %s" % e)

def main():
    rospy.init_node('infra_to_rgb_converter', anonymous=True)
    infra_to_rgb_converter = InfraToRGBConverter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")

if __name__ == '__main__':
    main()
