#include <ros/ros.h>
#include <sensor_msgs/Image.h>

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    // ここでカメラからの画像データを処理する
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "realsense_camera_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/camera/color/image_raw", 1, imageCallback);
    
    ros::spin();

    return 0;
}