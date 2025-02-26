#include <ros/ros.h>
#include <std_msgs/String.h>

void callback(const std_msgs::String::ConstPtr& data)
{
    ROS_INFO("Received data: %s", data->data.c_str());
}

void listener()
{
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/talk_request", 10, callback);
    ros::spin();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "listener");
    listener();

    return 0;
}
