#!/usr/bin/env python3
import os, csv, signal, cv2, rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError
from geometry_msgs.msg import Point
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox

class ImagePublisher:
    def __init__(self):
        rospy.init_node('image_publisher', anonymous=True)
        self.bridge = CvBridge()

        # ディレクトリ1の設定
        self.image_pub1 = rospy.Publisher('/z/image2movie_infra', Image, queue_size=10)
        self.image_folder1 = '/home/keisoku/ROBOT_data/Robot_data0718/fake/infra/edge' 
        # self.image_folder1 = '/home/keisoku/Robot_data/infra_align_512/edgenasi' 
        self.image_files1 = sorted([os.path.join(self.image_folder1, f) for f in os.listdir(self.image_folder1) if f.endswith('.png')])

        # ディレクトリ2の設定
        self.image_pub2 = rospy.Publisher('/z/image2movie_color', Image, queue_size=10)
        self.image_folder2 = '/home/keisoku/pytorch-CycleGAN-and-pix2pix/datasets/Data3/Data/color'  # ディレクトリ2の画像が保存されているパスを指定
        self.image_files2 = sorted([os.path.join(self.image_folder2, f) for f in os.listdir(self.image_folder2) if f.endswith('.png')])

        self.running = True
        signal.signal(signal.SIGINT, self.signal_handler)
        
        #yolo
        self.cv_bridge = CvBridge()
        self.bbox = BoundingBox()
        self.m_pub_threshold = rospy.get_param("~pub_threshold", 0.1)
        self.pub = rospy.Publisher("point_head_topic", Point, queue_size=10)
        self.image_number = 0
        self.bbox_list = []

        
        # CSVファイルの初期化
        self.csv_file1 = "/home/keisoku/gomi/detection_data.csv"
        with open(self.csv_file1, 'w', newline='') as file1:
            writer = csv.writer(file1)
            writer.writerow(["Image Number", "Class", "Probability"])
            
        # CSVファイルの初期化
        self.csv_file2 = "/home/keisoku/gomi/detection_count.csv"
        with open(self.csv_file2, 'w', newline='') as file2:
            writer = csv.writer(file2)
            writer.writerow(["Image Number", "Total Objects", "Bottle Count", "Cup Count", "Bowl Count"])
        
        

    def signal_handler(self, sig, frame):
        self.running = False
        rospy.logwarn('Shutting down gracefully...')
        rospy.signal_shutdown('Ctrl+C pressed')

    def publish_images(self):
        rate = rospy.Rate(1)  # fpsで画像を送信

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
            
            
            #yoloが検出した結果をもらう
            sub_darknet_bbox = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.DarknetBboxCallback)

            rate.sleep()
    
    def DarknetBboxCallback(self, darknet_bboxs):
        self.bbox_list = darknet_bboxs.bounding_boxes
        self.ImageCallback()
        
    def ImageCallback(self, img_msg):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
            total_objects = 0
            object_counts = {'bottle': 0, 'cup': 0, 'bowl': 0}

            with open(self.csv_file1, 'a', newline='') as file1, open(self.csv_file2, 'a', newline='') as file2:
                writer1 = csv.writer(file1)
                writer2 = csv.writer(file2)

                for bbox in self.bbox_list:
                    if bbox.Class in object_counts and bbox.probability >= self.m_pub_threshold:
                        cv2.rectangle(cv_image, (bbox.xmin, bbox.ymin), (bbox.xmax, bbox.ymax), (0, 255, 0), 2)
                        label = f"{bbox.Class}: {bbox.probability:.2f}"
                        cv2.putText(cv_image, label, (bbox.xmin, bbox.ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        
                        object_counts[bbox.Class] += 1
                        total_objects += 1

                        # CSVに書き込み (cup, bowl, bottle のみ)
                        writer1.writerow([self.image_number, bbox.Class, bbox.probability])
                
                # 各画像のオブジェクト数をCSVに書き込み
                writer2.writerow([self.image_number, total_objects, object_counts['bottle'], object_counts['cup'], object_counts['bowl']])

            cv2.imshow("Detection Image", cv_image)
            cv2.waitKey(1)
            
            self.image_number += 1
            
            self.save_dir = "/home/keisoku/gomi"
            image_filename = f"{self.save_dir}/{self.image_number}.png"
            
            if not os.path.exists(self.save_dir):
                os.makedirs(self.save_dir)
            
            cv2.imwrite(image_filename, cv_image)
            rospy.loginfo(f"Image saved to {image_filename}")

            # 各信頼度を表示
            for bbox in self.bbox_list:
                if bbox.probability >= self.m_pub_threshold:
                    rospy.loginfo(f"Detected {bbox.Class} with probability: {bbox.probability:.2f}")

            # 画像全体のオブジェクト数を表示
            rospy.loginfo(f"Total number of objects detected: {total_objects}")

            # 物体ごとのオブジェクト数を表示
            for obj_class, count in object_counts.items():
                rospy.loginfo(f"Number of {obj_class}s detected: {count}")

        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
        except Exception as e:
            rospy.logerr("Unexpected error: {0}".format(e))        

    

if __name__ == '__main__':
    try:
        image_publisher = ImagePublisher()
        image_publisher.publish_images()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo('Node terminated.')
        
    # imageの読み込み
    # pub
    # yolo_detection 
    # bboxで取得画像に上書き
    # 信頼度をcsvに書く 
    # 次の画像の読み込み
    