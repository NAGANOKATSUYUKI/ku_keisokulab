/* 一定周期繰り返し処理 */
#include <ros/ros.h>                        // ROSヘッダファイル

int main(int argc, char **argv) {
  ros::init(argc, argv, "hello");           // ROS初期化（ノード名を"hello"に設定）
  ros::NodeHandle nh;                       // ノードハンドルの生成
  ros::Rate rate(1);                        // 周期設定（1Hzに設定）
  while(ros::ok()) {                        // Ctrl+Cが押されるまで繰り返し
    ROS_INFO_STREAM("Hello World!");        // 標準出力へ"Hello World"を表示
    rate.sleep();                           // 指定された周期待ち（ループ内の他処理時間を引いた時間分を待つ）
  }
}
