#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class ImageProcessor:
    def __init__(self):
        self.bridge = CvBridge()
        #Subscriber
        self.image_sub = rospy.Subscriber("/camera/infra/image_raw", Image, self.image_callback)
        # self.image_sub = rospy.Subscriber("/z/infra2rgb/image_raw", Image, self.image_callback)
        # self.image_sub = rospy.Subscriber("/z/image2movie_infra", Image, self.image_callback)
        
        #Publisher
        self.image_pub = rospy.Publisher("/z/edge_image", Image, queue_size=1)
        self.padding_width = 3

    def image_callback(self, data):
        try:
            # ROSImage_msg to OpenCV_Image
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="rgb8")
        except CvBridgeError as e:
            rospy.logerr(f"Could not convert from ROS Image to OpenCV Image: {e}")
            return

        # エッジ検出
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 20, 30)
        edges_color = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        # フレームとエッジの組合せ
        comb_images = cv2.addWeighted(cv_image, 0.8, edges_color, 0.2, 0)

        try:
            # OpenCVイメージをROSイメージメッセージに変換してパブリッシュ
            image_msg = self.bridge.cv2_to_imgmsg(comb_images, "bgr8")
            self.image_pub.publish(image_msg)
        except CvBridgeError as e:
            rospy.logerr(f"Could not convert from OpenCV Image to ROS Image: {e}")
            return

        # # 結果を表示（オプション）
        # cv2.imshow("frame_comb", comb_images)
        # cv2.waitKey(1)

def main():
    rospy.init_node('image_processor_node', anonymous=True)
    ip = ImageProcessor()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
