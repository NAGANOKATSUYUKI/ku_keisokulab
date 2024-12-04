#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox
import cv2

class Detector():
    def __init__(self) :

        self.cv_bridge = CvBridge()
        self.bbox = BoundingBox()
        self.m_pub_threshold = rospy.get_param("~pub_threshold", 0.40)

        sub_cam_rgb      = rospy.Subscriber("/camera/color/image_raw", Image, self.ImageCallback)
        sub_darknet_bbox = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.DarknetBboxCallback)
        sub_cam_depth    = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.DepthCallback) #rs_camera.launch の中のalin_depthをtrueにしたことでいけた。サイズも６４０＊４８０になっている

    def ImageCallback(self, cam_image_msg):
        try:
            # ROSのImageメッセージをOpenCVの画像形式に変換
            cam_image = self.cv_bridge.imgmsg_to_cv2(cam_image_msg, "bgr8")
            height, width = cam_image.shape[:2]

            # #トリミング
            # crop_x = 320  #原点からの基準座標
            # crop_y = 120  #
            # crop_width = 640  #基準座標からのトリミング幅
            # crop_height = 480 #          　トリミング高さ

            # crop_image = cam_image[crop_y:crop_y+crop_height,crop_x:crop_x+crop_width]
            # height, width = crop_image.shape[:2]

            # #画像のリサイズ
            # new_width = 640
            # new_height= 480

            # resized_width = new_width
            # resized_height = int(height * (new_width / width))

            # if resized_height > new_height :
            #     aspect = float(new_height) / resized_height
            #     resized_height = new_height
            #     resized_width = int(resized_width * aspect)

            # cam_image_resized = cv2.resize(cam_image, (resized_width, resized_height)) #画像のリサイズはいけているが、YOLOからのもらうサイズが1280*720になっているため、検出の矩形と点ずれている
            # resized_height, resized_width = cam_image_resized.shape[:2]

        except CvBridgeError as e:
            rospy.logerr(e)

        # # スケーリングあり
        # if self.bbox.probability > 0.0 :
        #     # スケーリング率計算
        #     original_width  = 1280
        #     original_height = 720

        #     scale_x = original_width / resized_width
        #     scale_y = original_height / resized_height

        #     #スケーリング調整
        #     bbox_xmax = int(self.bbox.xmax / scale_x)
        #     bbox_xmin = int(self.bbox.xmin / scale_x)
        #     bbox_ymax = int(self.bbox.ymax / scale_y)
        #     bbox_ymin = int(self.bbox.ymin / scale_y)

        #     #中心座標
        #     w = bbox_xmax - bbox_xmin
        #     h = bbox_ymax - bbox_ymin
        #     x = bbox_xmin + w/2
        #     y = bbox_ymin + h/2
        #     class_name = str(self.bbox.Class)
                
        #     #深度距離
        #     depth_x = int((bbox_xmax + bbox_xmin) // 2)
        #     depth_y = int((bbox_ymax + bbox_ymin) // 2)
        #     bbox_depth = self.depth_image[depth_y][depth_x]

        #     rospy.loginfo("Class: %s, Score: %.2f, center: x=%.1f, y=%.1f, Dist: %dmm" % (class_name, self.bbox.probability, x, y, bbox_depth))
        #     rospy.loginfo("color:width= %d, height= %d, depth:width= %d, height= %d" %(resized_width, resized_height, self.depth_width, self.depth_height ))                
        #     # 検出したものを円で囲む
        #     cv2.circle(cam_image_resized, (int(x), int(y)), 5, (0,0,255), -1)

        #     #検出したものを矩形で囲む
        #     cv2.rectangle(cam_image_resized, (bbox_xmin, bbox_ymin), (bbox_xmax, bbox_ymax),(0,0,255), 2)
        #     cv2.putText(cam_image_resized, class_name, (bbox_xmin, bbox_ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255),2 )
            
        #     cv2.imshow("cam_image", cam_image_resized)
        #     cv2.waitKey(1)
        #     cv2.imshow("depth_image", self.depth_image)
        #     cv2.waitKey(1)
        # else :
        #     cv2.imshow("cam_image", cam_image_resized)
        #     cv2.waitKey(1)
        #     cv2.imshow("depth_image", self.depth_image)
        #     cv2.waitKey(1)

        # スケーリングなし
        if self.bbox.probability > 0.0 :
            #中心座標
            w = self.bbox.xmax - self.bbox.xmin
            h = self.bbox.ymax - self.bbox.ymin
            x = self.bbox.xmin + w/2
            y = self.bbox.ymin + h/2
            class_name = str(self.bbox.Class)
                
            #深度距離
            depth_x = int((self.bbox.xmax + self.bbox.xmin) // 2)
            depth_y = int((self.bbox.ymax + self.bbox.ymin) // 2)
            bbox_depth = self.depth_image[depth_y][depth_x]

            rospy.loginfo("Class: %s, Score: %.2f, center: x=%.1f, y=%.1f, Dist: %dmm" % (class_name, self.bbox.probability, x, y, bbox_depth))
            rospy.loginfo("color:width= %d, height= %d, depth:width= %d, height= %d" %(width, height, self.depth_width, self.depth_height ))

        
            #検出したものを円で囲む
            cv2.circle(cam_image, (int(x), int(y)), 5, (0,0,255), -1)

            #検出したものを矩形で囲む
            cv2.rectangle(cam_image, (self.bbox.xmin, self.bbox.ymin), (self.bbox.xmax, self.bbox.ymax),(0,0,255), 2)
            cv2.putText(cam_image, class_name, (self.bbox.xmin, self.bbox.ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255),2 )
            
            cv2.imshow("cam_image", cam_image)
            cv2.waitKey(1)
            cv2.imshow("depth_image", self.depth_image)
            cv2.waitKey(1)
        else :
            cv2.imshow("cam_image", cam_image)
            cv2.waitKey(1)
            cv2.imshow("depth_image", self.depth_image)
            cv2.waitKey(1)
    
    def DepthCallback(self, depth_image_data):
        self.depth_image = None
        try:
            self.depth_image = self.cv_bridge.imgmsg_to_cv2(depth_image_data, "passthrough")
            self.depth_height, self.depth_width = self.depth_image.shape[:2]
            # #画像のリサイズ
            # new_width = 640
            # new_height= int(self.depth_height * (new_width / self.depth_width))
            # self.depth_image_resized = cv2.resize(self.depth_image, (new_width, new_height))

            # #トリミング
            # crop_x = 320
            # crop_y = 120
            # crop_width = 640
            # crop_height = 480
            # self.crop_depth_image = self.depth_image[crop_y:crop_y+crop_height,crop_x:crop_x+crop_width]
            # self.depth_height, self.depth_width = self.crop_depth_image.shape[:2]

        except CvBridgeError as e:
            rospy.logerr(e)

    def DarknetBboxCallback(self, darknet_bboxs):

        bboxs = darknet_bboxs.bounding_boxes
        bbox = BoundingBox()
        if len(bboxs) != 0 :
            for i, bb in enumerate(bboxs) :
                if bboxs[i].Class == 'cell phone' and bboxs[i].probability >= self.m_pub_threshold:
                    bbox = bboxs[i]        
        self.bbox = bbox
    
if __name__ == '__main__':
    try:
        rospy.init_node('yolo_object_detection', anonymous=True)

        data = Detector()
        rospy.loginfo("data Initialized")
        rospy.spin()  # メッセージの受信と処理を継続
        

    except rospy.ROSInterruptException:
        pass
