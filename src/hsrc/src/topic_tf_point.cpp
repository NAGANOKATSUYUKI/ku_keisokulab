#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

class Tfpoint_Detector {
public:
    Tfpoint_Detector() {
        bbox = darknet_ros_msgs::BoundingBox();
        pub_threshold = ros::param::param("~pub_threshold", 0.01);
        pub = nh.advertise<geometry_msgs::Point>("point_topic", 10);

        //average_variable
        i = 0;
        sum_x = 0;
        sum_y = 0;
        sum_z = 0;

        //検出されて初めてサブスクライブする方法
        sub_darknet_bbox = nh.subscribe("/darknet_ros/bounding_boxes", 1, &Tfpoint_Detector::DarknetBboxCallback, this);
    }

    //target detection
    void DarknetBboxCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& darknet_bboxs) {
        darknet_ros_msgs::BoundingBox bbox;
        const std::vector<darknet_ros_msgs::BoundingBox>& head_bboxs = darknet_bboxs->bounding_boxes;
        if (head_bboxs.size() != 0) {
            for (int i = 0; i < head_bboxs.size(); i++) {
                if (head_bboxs[i].Class == "pet" && head_bboxs[i].probability >= pub_threshold) {
                    bbox = head_bboxs[i];
                    class_name = bbox.Class;
                }else{
                    //
                }
            }
        }else{
            //
        }
        // if (bboxs.size() != 0) {
        //     darknet_ros_msgs::BoundingBox best_bottle; // 最も信頼性の高い "bottle" オブジェクトを格納する変数
        //     double best_bottle_probability = 0.0; // 最も信頼性の高い "bottle" オブジェクトの信頼度
        //     for (int i = 0; i < bboxs.size(); i++) {
        //         if (bboxs[i].Class == "bottle" && bboxs[i].probability >= pub_threshold) {
        //             if (bboxs[i].probability > best_bottle_probability) {
        //                 best_bottle = bboxs[i]; // より高い信頼性の "bottle" オブジェクトを選択
        //                 best_bottle_probability = bboxs[i].probability; // 信頼度を更新
        //             }
        //         }
        //     }
        //     if (best_bottle_probability > 0.0) {
        //         bbox = best_bottle;
        //         class_name = bbox.Class;
        //     }
        // }
        
        cam_x = bbox.xmin + (bbox.xmax - bbox.xmin) / 2;
        cam_y = bbox.ymin + (bbox.ymax - bbox.ymin) / 2;
        sub_swich = true;
        sub_cam_depth = nh.subscribe("/hsrb/head_rgbd_sensor/depth_registered/image_raw", 1, &Tfpoint_Detector::DepthCallback, this);
        // ROS_INFO("x = %.2d, y = %.2d", cam_x, cam_y);
    }

    //depth 
    void DepthCallback(const sensor_msgs::Image::ConstPtr& depth_image_data) {
        try {
            if ( sub_swich == true) {
                cv_bridge::CvImagePtr cv_ptr;
                cv_ptr = cv_bridge::toCvCopy(depth_image_data, sensor_msgs::image_encodings::TYPE_32FC1);
                int depth_x = static_cast<int>(cam_x);
                int depth_y = static_cast<int>(cam_y);
                bbox_depth = cv_ptr->image.at<float>(depth_y, depth_x);
                // ROS_INFO("%d, %d, %f", depth_x, depth_y, bbox_depth);
                sub_camera_info = nh.subscribe("/hsrb/head_rgbd_sensor/depth_registered/camera_info", 1, &Tfpoint_Detector::CameraInfoCallback, this);
                sub_swich = false;
            }else{
                sub_cam_depth.shutdown();
                sub_camera_info.shutdown();
            }
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("Cvbridge error: %s", e.what());
        }
    }

    //座標軸変換
    void CameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& info_msg) {

        //検出しないときは通さない
        if(cam_x != 0.0 && cam_y != 0.0 || cam_x > cam_y ){
            try {
                crrection_x = cam_x - info_msg->K[2];//320
                crrection_y = cam_y - info_msg->K[5];//280
                z = bbox_depth;
                x1 = z * (crrection_x / info_msg->K[0]);
                y1 = z * (crrection_y / info_msg->K[4]);
                x = x1 * 0.001 ;
                y = y1 * 0.001 + 0.2 * y1 *0.001;
                z = z * 0.001;
                // ROS_INFO("x = %.2f, y = %.2f, z = %.2f", x, y, z);
            }
            catch (...) {
                ROS_WARN("transform_ERROR");
            }

            //topic Publish
            try {
                geometry_msgs::Point point_msg;
                if (0.40 < z && z <= 1.5){          //距離の制限
                    if (-0.4 < x && x < 0.4){       //　幅の制限
                        CoordinatePointCallback();
                        sum_x = sum_x + x;
                        sum_y = sum_y + y;
                        sum_z = sum_z + z;
                        i = i + 1;
                        ROS_INFO("class:%s x = %.2f y = %.2f z = %.2f %d", class_name.c_str(), x, y, z, i);
                    }else{
                        ROS_WARN("Out of range");
                    }
                }else{
                    ROS_WARN("Distance over");
                }

                if (i == 30){
                    x = sum_x / 30;
                    y = sum_y / 30; 
                    z = sum_z / 30;
                    sum_x = 0;
                    sum_y = 0;
                    sum_z = 0;
                    point_msg.x = x;
                    point_msg.y = y;
                    point_msg.z = z;
                    pub.publish(point_msg);
                    i = 0;
                }else{
                    //
                }
            }
            catch (...) {
                ROS_WARN("TopicPublish_ERROR");
            }
        }else{
            ROS_WARN("Not Detection");
        }    
    }

    //tf作成
    void CoordinatePointCallback() {
        try {
            static tf2_ros::TransformBroadcaster tf_broadcaster;

            geometry_msgs::TransformStamped gt;
            gt.header.stamp = ros::Time::now();
            gt.header.frame_id = "head_rgbd_sensor_link"; 
            gt.child_frame_id = "target_frame"; 
            gt.transform.translation.x = x;
            gt.transform.translation.y = y;
            gt.transform.translation.z = z;

            tf2::Quaternion q;
            q.setRPY(0, 0, 0); 
            gt.transform.rotation.x = q.x();
            gt.transform.rotation.y = q.y();
            gt.transform.rotation.z = q.z();
            gt.transform.rotation.w = q.w();

            tf_broadcaster.sendTransform(gt);
            // ROS_INFO("x = %.2f, y = %.2f, z = %.2f tf_publish --> OK", x, y, z);
        }
        catch (std::exception& e) {
            ROS_ERROR_STREAM("Unable to create tf: " << e.what());
        }
    }

private:
    ros::Publisher pub; 
    ros::Subscriber sub_cam_depth;
    ros::Subscriber sub_darknet_bbox;
    ros::Subscriber sub_camera_info;
    ros::NodeHandle nh;
    cv_bridge::CvImagePtr cv_bridge;
    darknet_ros_msgs::BoundingBox bbox;
    double pub_threshold;
    int cam_x, cam_y, i;
    float bbox_depth, z;
    float crrection_x , crrection_y, x, y;
    float x1, y1;
    float sum_x,sum_y,sum_z;
    std:: string class_name;
    bool sub_swich;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "Tfpoint_detector_node");
    Tfpoint_Detector tfpoint_detector;
    ros::spin();
    return 0;
}

//1109
//1113
//1114
