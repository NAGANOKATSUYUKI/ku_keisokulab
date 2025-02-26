#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import signal
import sys

class ImagePublisher:
    def __init__(self):
        rospy.init_node('image_publisher', anonymous=True)
        self.bridge = CvBridge()

        # ディレクトリ1の設定
        self.image_pub1 = rospy.Publisher('/z/image2movie_infra', Image, queue_size=10)
        self.image_folder1 = '/home/keisoku/gomi3/0829_mov/edgenasi' 
        # self.image_folder1 = '/home/keisoku/Robot_data/infra_align_512/edgenasi' 
        self.image_files1 = sorted([os.path.join(self.image_folder1, f) for f in os.listdir(self.image_folder1) if f.endswith('.png')])

        # ディレクトリ2の設定
        self.image_pub2 = rospy.Publisher('/z/image2movie_color', Image, queue_size=10)
        self.image_folder2 = '/home/keisoku/gomi3/0829_mov/edge'  # ディレクトリ2の画像が保存されているパスを指定
        self.image_files2 = sorted([os.path.join(self.image_folder2, f) for f in os.listdir(self.image_folder2) if f.endswith('.png')])

        self.running = True
        signal.signal(signal.SIGINT, self.signal_handler)

    def signal_handler(self, sig, frame):
        self.running = False
        rospy.logwarn('Shutting down gracefully...')
        rospy.signal_shutdown('Ctrl+C pressed')

    def publish_images(self):
        rate = rospy.Rate(20)  # 30fpsで画像を送信

        # 両方のディレクトリから順に画像を送信
        for img_file1, img_file2 in zip(self.image_files1, self.image_files2):
            if not self.running:
                break
            
            image1 = cv2.imread(img_file1)
            image2 = cv2.imread(img_file2)

            if image1 is None:
                rospy.logwarn(f'Failed to read image file: {img_file1}')
                continue
            if image2 is None:
                rospy.logwarn(f'Failed to read image file: {img_file2}')
                continue

            ros_image1 = self.bridge.cv2_to_imgmsg(image1, 'bgr8')
            ros_image2 = self.bridge.cv2_to_imgmsg(image2, 'bgr8')

            self.image_pub1.publish(ros_image1)
            self.image_pub2.publish(ros_image2)

            rospy.loginfo(f'Published images{img_file1}')
            rate.sleep()

if __name__ == '__main__':
    try:
        image_publisher = ImagePublisher()
        image_publisher.publish_images()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo('Node terminated.')
