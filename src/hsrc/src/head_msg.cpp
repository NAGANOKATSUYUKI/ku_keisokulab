#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

class Tfpoint_Detector {
public:
    Tfpoint_Detector() {
        bbox = darknet_ros_msgs::BoundingBox();
        pub_threshold = ros::param::param("~pub_threshold", 0.40);
        pub = nh.advertise<geometry_msgs::Point>("point_topic", 10);

        //検出されて初めてサブスクライブする方法
        sub_darknet_bbox = nh.subscribe("/darknet_ros0/bounding_boxes", 1, &Tfpoint_Detector::DarknetBboxCallback, this);
    }

    //target detection
    void DarknetBboxCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& darknet_bboxs) {
        darknet_ros_msgs::BoundingBox bbox;
        const std::vector<darknet_ros_msgs::BoundingBox>& head_bboxs = darknet_bboxs->bounding_boxes;
        if (head_bboxs.size() != 0) {
            for (int i = 0; i < head_bboxs.size(); i++) {
                if (head_bboxs[i].Class == "bottle" && head_bboxs[i].probability >= pub_threshold) {
                    bbox = head_bboxs[i];
                    class_name = bbox.Class;
                }else{
                    //
                }
            }
        }else{
            //
        }
        
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
                        point_msg.x = x;
                        point_msg.y = y;
                        point_msg.z = z;
                        pub.publish(point_msg);
                        ROS_INFO("class:%s x = %.2f y = %.2f z = %.2f", class_name.c_str(), x, y, z);
                    }else{
                        ROS_WARN("Out of range");
                    }
                }else{
                    ROS_WARN("Distance over");
                }
            }
            catch (...) {
                ROS_WARN("TopicPublish_ERROR");
            }
        }else{
            ROS_WARN("Not Detection");
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
    int cam_x, cam_y;
    float bbox_depth, z;
    float crrection_x , crrection_y, x, y;
    float x1, y1;
    std:: string class_name;
    bool sub_swich;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "Tfpoint_detector_node");
    Tfpoint_Detector tfpoint_detector;
    ros::spin();
    return 0;
}

//1116