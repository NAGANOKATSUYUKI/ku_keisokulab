#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/opencv.hpp>


class Detector {
public:
    Detector() {
        bbox = darknet_ros_msgs::BoundingBox();
        pub_threshold = ros::param::param("~pub_threshold", 0.40);

        sub_cam_depth = nh.subscribe("/hsrb/head_rgbd_sensor/depth_registered/image_raw", 1, &Detector::DepthCallback, this);
        sub_darknet_bbox = nh.subscribe("/darknet_ros/bounding_boxes", 1, &Detector::DarknetBboxCallback, this);
        sub_camera_info = nh.subscribe("/hsrb/head_rgbd_sensor/depth_registered/camera_info", 1, &Detector::CameraInfoCallback, this);

    }
    void DepthCallback(const sensor_msgs::Image::ConstPtr& depth_image_data) {
        try {
            cv_bridge::CvImagePtr cv_ptr;
            cv_ptr = cv_bridge::toCvCopy(depth_image_data, sensor_msgs::image_encodings::TYPE_32FC1);
            int depth_x = static_cast<int>(cam_x);
            int depth_y = static_cast<int>(cam_y);
            bbox_depth = cv_ptr->image.at<float>(depth_y, depth_x);
            // ROS_INFO("%d, %d, %f", depth_x, depth_y, bbox_depth);
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("Cvbridge error: %s", e.what());
        }
    }

    void CameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& info_msg) {
        try {
            x = cam_x - info_msg->K[2];
            y = cam_y - info_msg->K[5];
            z = bbox_depth;
        }
        catch (...) {
            ROS_INFO("NO_1");
        }

        try {
            x1 = z * x / info_msg->K[0];
            y1 = z * y / info_msg->K[4];
        }
        catch (...) {
            ROS_INFO("NO_2");
        }

        try {
            x1 = x1 * 0.001;
            y1 = y1 * 0.001;
            z = z * 0.001;
            ROS_INFO("x = %.2f, y = %.2f, z = %.2f", x1, y1, z);
        }
        catch (...) {
            ROS_INFO("NO_3");
        }

        try {
            ros::Publisher pub = nh.advertise<geometry_msgs::Point>("point_topic", 10);
            geometry_msgs::Point point_msg;
            point_msg.x = x1;
            point_msg.y = y1;
            point_msg.z = z;
            pub.publish(point_msg);
        }
        catch (...) {
            ROS_INFO("NO_4");
        }
    }

    void DarknetBboxCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& darknet_bboxs) {
        darknet_ros_msgs::BoundingBox bbox;
        const std::vector<darknet_ros_msgs::BoundingBox>& bboxs = darknet_bboxs->bounding_boxes;
        if (bboxs.size() != 0) {
            for (int i = 0; i < bboxs.size(); i++) {
                if (bboxs[i].Class == "bottle" && bboxs[i].probability >= pub_threshold) {
                    bbox = bboxs[i];
                }
            }
        }
        int w = bbox.xmax - bbox.xmin;
        int h = bbox.ymax - bbox.ymin;
        cam_x = bbox.xmin + w / 2;
        cam_y = bbox.ymin + h / 2;
        // ROS_INFO("x = %.2d, y = %.2d", cam_x, cam_y);

    }

private:
    ros::Subscriber sub_cam_depth;
    ros::Subscriber sub_darknet_bbox;
    ros::Subscriber sub_camera_info;
    ros::NodeHandle nh;
    cv_bridge::CvImagePtr cv_bridge;
    darknet_ros_msgs::BoundingBox bbox;
    double pub_threshold;
    int cam_x;
    int cam_y;
    float bbox_depth;
    float x;
    float y;
    float z;
    float x1;
    float y1;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "object_detector_node");
    Detector detector;
    ros::spin();
    return 0;
}