# darknet_ros Guide

`darknet_ros` は YOLO(Darknet) を ROS で使うための物体検出パッケージです。

## Purpose
- 入力画像トピックから物体検出を実行
- bbox情報・検出数・描画済み画像をROSトピックで配信

## Main Outputs
- `/darknet_ros/found_object`
- `/darknet_ros/bounding_boxes`
- `/darknet_ros/detection_image`

## Directory Structure
- `darknet_ros/launch/`: 起動設定
- `darknet_ros/config/`: ROS I/O設定・モデル設定yaml
- `darknet_ros/yolo_network_config/cfg`: ネットワーク定義
- `darknet_ros/yolo_network_config/weights`: 学習済み重み
- `darknet_ros_msgs/`: `BoundingBoxes` などのメッセージ

## Launch Files (Key Ones)
- `darknet_ros.launch`: 基本起動（`ros.yaml` + モデルyaml読込）
- `yolo_v3.launch`: YOLOv3起動用ラッパ
- `darknet_ros_rename.launch`: 名前空間/トピック名を引数で変更
- `hsrb_v3x2.launch`: 2カメラ同時検出（2インスタンス起動）
- `hsrb_darknet.launch`: HSRB頭部カメラ向け設定
- `darknet_ros_nodelet.launch`: nodelet版
- `darknet_ros_gdb.launch`: gdbデバッグ用
- `darknet_ros_valgrind.launch`: valgrind検証用

## ros.yaml (What It Controls)
- `subscribers.camera_reading.topic`: 検出入力画像トピック
- `publishers.*.topic`: 検出結果出力トピック
- `actions.camera_reading.name`: action名
- `image_view.*`: OpenCV表示・ログ表示の挙動

## yaml Roles
- `ros.yaml`: ROS入出力の設定
- `yolov3.yaml` など: モデル/閾値/クラス名の設定

## Practical Customization
1. 入力画像を変える: `config/ros.yaml`
2. モデルを変える: launchの `network_param_file`
3. クラス/閾値を変える: `config/*.yaml`
