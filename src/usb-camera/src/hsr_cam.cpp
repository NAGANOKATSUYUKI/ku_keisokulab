#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    // ここで画像データを処理する
    try
    {
        // 画像データをOpenCV形式に変換
        //all
        //cv::Mat image = cv_bridge::toCvShare(msg, msg->encoding)->image; 

        // //bgr
        cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image; 

        // //mono
        // cv::Mat image = cv_bridge::toCvShare(msg, "mono8")->image; 
        // cv::Mat image = cv_bridge::toCvShare(msg, "mono16")->image; 

        // 画像を表示
        cv::imshow("Topic Image", image);
        cv::waitKey(1);
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_node");
    ros::NodeHandle nh;
    
    //depth
    // ros::Subscriber sub = nh.subscribe("/depthcloud_encoded", 1, imageCallback);
    // ros::Subscriber sub = nh.subscribe("/hsrb/head_rgbd_sensor/depth_registered/image", 1, imageCallback);
    // ros::Subscriber sub = nh.subscribe("/hsrb/head_rgbd_sensor/depth_registered/image_raw", 1, imageCallback);
    // ros::Subscriber sub = nh.subscribe("/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw", 1, imageCallback);

    // //rgb 
    ros::Subscriber sub = nh.subscribe("/hsrb/head_rgbd_sensor/rgb/image_raw", 1, imageCallback); 
    // ros::Subscriber sub = nh.subscribe("/hsrb/head_rgbd_sensor/rgb/image_rect_raw", 1, imageCallback); 
    
    // //infra
    // ros::Subscriber sub = nh.subscribe("/hsrb/head_rgbd_sensor/ir/image", 1, imageCallback);
    
    // //hand
    // ros::Subscriber sub = nh.subscribe("/hsrb/hand_camera/image_raw", 1, imageCallback); 

    // //stereo
    // ros::Subscriber sub = nh.subscribe("/hsrb/head_r_stereo_camera/image_raw", 1, imageCallback); 
    // ros::Subscriber sub = nh.subscribe("/hsrb/head_r_stereo_camera/image_rect_raw", 1, imageCallback); 
    // ros::Subscriber sub = nh.subscribe("/hsrb/head_l_stereo_camera/image_raw", 1, imageCallback); 
    // ros::Subscriber sub = nh.subscribe("/hsrb/head_l_stereo_camera/image_rect_raw", 1, imageCallback); 

    ROS_INFO("cv_bridge");

    //ウィンドウの作成
    cv::namedWindow("Topic Image", cv::WINDOW_NORMAL);
    ROS_INFO("image_show");

    ros::spin();

    return 0;
}
