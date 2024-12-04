#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;

        cv::imshow("Image", image);
        cv::waitKey(1);

    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

}

int main(int argc,char** argv) {

    ros::init(argc, argv, "image_viewer");
    ros::NodeHandle nh;

    ros::Subscriber image_sub = nh.subscribe<sensor_msgs::Image>("/camera/color/image_raw", 1,imageCallback);

    ros::spin();
    return 0;
}