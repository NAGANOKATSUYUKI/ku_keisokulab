#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
from datetime import datetime, timedelta
import message_filters

# class DatasetMakerNode:

#     def __init__(self):
#         rospy.init_node('dataset_maker', anonymous=True)
#         self.cv_bridge = CvBridge()
#         self.last_save_time_infrared = datetime.now()
#         self.last_save_time_color = datetime.now()
#         self.last_save_time_depth = datetime.now()  
        
#         #topic_name
#         self.infrared_sub_ = rospy.Subscriber(
#             # '/camera/infra/image_raw',
#             '/z/infra/image',
#             Image, self.infrared_sub_callback, queue_size=10)
        
#         self.color_sub_ = rospy.Subscriber(
#             # '/camera/color/image_raw',
#             '/z/color/image',
#             Image, self.color_sub_callback, queue_size=10
#         )
#         self.depth_sub = rospy.Subscriber(
#             # '/camera/aligned_depth_to_color/image_raw',
#             '/z/depth/image',
#             Image, self.depth_sub_callback, queue_size=10
#         )
        
#         #save_name
#         self.save_dir = "/home/keisoku/20241202_tukigaoka"
#         if not os.path.exists(self.save_dir):
#             os.makedirs(self.save_dir)
        
#         self.infrared_image_count = 1  # 赤外線
#         self.color_image_count = 1     # カラー
#         self.depth_image_count = 1     # 深度

#     def infrared_sub_callback(self, msg):
#         self.save_callback(msg, "Infra_frame", self.infrared_image_count, self.last_save_time_infrared)

#     def color_sub_callback(self, msg):
#         self.save_callback(msg, "Color_frame", self.color_image_count, self.last_save_time_color)
        
#     def depth_sub_callback(self, msg):
#         self.save_callback(msg, "Depth_frame", self.depth_image_count, self.last_save_time_depth)

#     def save_callback(self, msg, prefix, count, last_save_time):
#         current_time = datetime.now()
#         elapsed_time = (current_time - last_save_time).total_seconds()

#         if elapsed_time >= 0.05:  # 0.05秒ごとに保存
#             try:
#                 cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "passthrough")
#                 self.save_image(cv_image, prefix, count, current_time)
#             except CvBridgeError as e:
#                 rospy.logerr("cv_bridge exception: %s" % str(e))

#     def save_image(self, image, prefix, count, current_time):
#         save_path = os.path.join(self.save_dir, f"{prefix}_{count}.png")
#         cv2.imwrite(save_path, image)
#         rospy.loginfo(f"{prefix}を {count} に保存しました")
#         # 画像数を増やす
#         if "Infra" in prefix:
#             self.infrared_image_count += 1
#             self.last_save_time_infrared = current_time
#         elif "Color" in prefix:
#             self.color_image_count += 1
#             self.last_save_time_color = current_time
#         elif "Depth" in prefix:
#             self.depth_image_count += 1  # 修正: self.depth_iamge_count -> self.depth_image_count
#             self.last_save_time_depth = current_time

class DatasetMakerNode:
    def __init__(self) :
        rospy.init_node('dataset_maker', anonymous=True)
        self.cv_bridge = CvBridge()
        self.last_save_time = datetime.now()
        
        self.infrared_sub = message_filters.Subscriber('/z/infra/image', Image)
        self.color_sub = message_filters.Subscriber('/z/color/image', Image)
        self.depth_sub = message_filters.Subscriber('/z/depth/image', Image)
        
        # トピックを同期
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.infrared_sub, self.color_sub, self.depth_sub],
            queue_size=10,
            slop=0.1
        )
        self.sync.registerCallback(self.sync_callback)
        
        # 保存ディレクトリ
        self.save_dir = "/home/keisoku/20241203_gomi"
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

        # 画像カウント
        self.infrared_image_count = 1  # 赤外線
        self.color_image_count = 1     # カラー
        self.depth_image_count = 1     # 深度
        
    def sync_callback(self, infra_msg, color_msg, depth_msg):
        """同期されたメッセージを処理"""
        current_time = datetime.now()
        elapsed_time = (current_time - self.last_save_time).total_seconds()

        if elapsed_time >= 0.05:  # 0.05秒ごとに保存
            try:
                # 画像を変換して保存
                infra_image = self.cv_bridge.imgmsg_to_cv2(infra_msg, "passthrough")
                color_image = self.cv_bridge.imgmsg_to_cv2(color_msg, "bgr8")
                depth_image = self.cv_bridge.imgmsg_to_cv2(depth_msg, "passthrough")

                self.save_image(infra_image, "Infra_frame", self.infrared_image_count)
                self.save_image(color_image, "Color_frame", self.color_image_count)
                self.save_image(depth_image, "Depth_frame", self.depth_image_count)

                # タイムスタンプとカウントの更新
                self.last_save_time = current_time
                self.infrared_image_count += 1
                self.color_image_count += 1
                self.depth_image_count += 1
            except CvBridgeError as e:
                rospy.logerr("cv_bridge exception: %s" % str(e))            
    
    def save_image(self, image, prefix, count):
        save_path = os.path.join(self.save_dir, f"{prefix}_{count}.png")
        cv2.imwrite(save_path, image)
        rospy.loginfo(f"{prefix}を {count} に保存しました")

def main():
    node = DatasetMakerNode()
    rospy.spin()

if __name__ == '__main__':
    main()
