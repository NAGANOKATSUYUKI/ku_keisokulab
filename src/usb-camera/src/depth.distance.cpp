#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

// カメラパラメータのためのグローバル変数
cv::Mat cameraMatrix;
cv::Mat distCoeffs;

// RGB-D画像を受け取るためのコールバック関数
void imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  try
  {
    // ROSイメージメッセージをOpenCVイメージに変換する
    cv_bridge::CvImagePtr cvImage = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat colorImage = cvImage->image;

    // デプスイメージを使用して深度測定を行う
    // ここに独自の深度測定アルゴリズムを実装してください

    // ポイントクラウドをパブリッシュする
    sensor_msgs::PointCloud2 pointCloud;
    // ポイントクラウドデータを生成するコードに置き換えてください

    // ポイントクラウドのヘッダを設定する
    pointCloud.header = msg->header;

    // ポイントクラウドをパブリッシュする
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("/point_cloud", 1);
    pub.publish(pointCloud);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridgeの例外エラー: %s", e.what());
  }
}

// カメラパラメータを受け取るためのコールバック関数
void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
  // カメラパラメータを後で使用するために保存する
  cameraMatrix = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->K.data())).clone();
  distCoeffs = cv::Mat(1, 5, CV_64F, const_cast<double*>(msg->D.data())).clone();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rgbd_depth_measurement");
  ros::NodeHandle nh;

  // RGBイメージのトピックにサブスクライブする
  ros::Subscriber sub = nh.subscribe("/hsrb/head_rgbd_sensor/rgb/image_raw", 1, imageCallback);

  // カメラ情報のトピックにサブスクライブする
  ros::Subscriber cameraInfoSub = nh.subscribe("/hsrb/head_rgbd_sensor/rgb/camera_info", 1, cameraInfoCallback);

  ros::spin();

  return 0;
}
