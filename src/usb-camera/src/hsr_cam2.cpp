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
        cv::Mat image = cv_bridge::toCvShare(msg, msg->encoding)->image; 

        std::string windowName = "";

        if (msg->header.frame_id == "/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw")
            windowName = "Depth Image";
        else if (msg->header.frame_id == "/hsrb/head_rgbd_sensor/rgb/image_rect_raw")
            windowName = "RGB Image";
        else if (msg->header.frame_id == "/hsrb/hand_camera/image_raw")
            windowName = "Hand Image";
        else if (msg->header.frame_id == "/hsrb/head_r_stereo_camera/image_rect_raw")
            windowName = "Sterao_r Image";
        else if (msg->header.frame_id == "/hsrb/head_l_stereo_camera/image_rect_raw")
            windowName = "Sterao_l Image";
        // 画像を表示
        cv::imshow("windowName", image);
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

    ROS_INFO("cv_bridge");

    //ウィンドウの作成
    //cv::namedWindow("Depth Image", cv::WINDOW_NORMAL);
    //cv::namedWindow("RGB Image", cv::WINDOW_NORMAL);
    //cv::namedWindow("Hand Image", cv::WINDOW_NORMAL);
    cv::namedWindow("Sterep_r Image", cv::WINDOW_NORMAL);
    cv::namedWindow("Sterep_l Image", cv::WINDOW_NORMAL);
    ROS_INFO("image_show");

    ros::spin();

    return 0;
}
