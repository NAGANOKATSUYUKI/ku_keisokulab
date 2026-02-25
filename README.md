# ku-keisoku

知識の遺産・ダウンロードOK

## パッケージ説明

このワークスペースは `src/` 配下に複数パッケージを持つ ROS1(catkin) 構成です。

### src 直下ディレクトリ

| ディレクトリ | 概要 | リンク |
|---|---|---|
| `sample` | 画像処理・ROS入出力・カスタムメッセージのサンプル集 | [src/sample](src/sample) / [README](src/sample/README.md) |
| `pix2pix` | RealSense画像の前処理、3D座標推定、TF配信、把持連携スクリプト | [src/pix2pix](src/pix2pix) / [README](src/pix2pix/README.md) |
| `multi_function_switch` | HSRBのスイッチ連動・SE再生の実験用スクリプト群 | [src/multi_function_switch](src/multi_function_switch) / [README](src/multi_function_switch/README.md) |
| `darknet_ros` | YOLO(Darknet)連携一式（複数ROSパッケージを内包） | [src/darknet_ros](src/darknet_ros) / [README](src/darknet_ros/README_WORKSPACE_JA.md) |
| `realsense-ros` | Intel RealSense公式ROSラッパ（複数ROSパッケージを内包） | [src/realsense-ros](src/realsense-ros) / [README](src/realsense-ros/README_WORKSPACE_JA.md) |

### ROSパッケージ一覧（`src/` 内）

| パッケージ名 | 概要 | リンク |
|---|---|---|
| `sample` | サンプル用ROSパッケージ（message定義、Python/C++ノード） | [src/sample](src/sample) |
| `pix2pix` | 画像処理・座標推定向けROSパッケージ | [src/pix2pix](src/pix2pix) |
| `darknet_ros` | YOLO物体検出ノード本体 | [src/darknet_ros/darknet_ros](src/darknet_ros/darknet_ros) |
| `darknet_ros_msgs` | YOLO検出結果用メッセージ定義 | [src/darknet_ros/darknet_ros_msgs](src/darknet_ros/darknet_ros_msgs) |
| `realsense2_camera` | RealSenseカメラドライバ本体 | [src/realsense-ros/realsense2_camera](src/realsense-ros/realsense2_camera) |
| `realsense2_description` | RealSenseカメラURDF/モデル定義 | [src/realsense-ros/realsense2_description](src/realsense-ros/realsense2_description) |

## その他

- 基本的に ROS1 用で Python/C++ ノードを作成しています。ROS2 で使う場合は、ROS1依存ライブラリ部分の修正が必要です。
- 補足README
  - [sample のREADME](src/sample/README.md)
  - [pix2pix のREADME](src/pix2pix/README.md)
  - [multi_function_switch のREADME](src/multi_function_switch/README.md)
  - [darknet_ros の説明README](src/darknet_ros/README_WORKSPACE_JA.md)
  - [realsense-ros の説明README](src/realsense-ros/README_WORKSPACE_JA.md)

## 画像処理・物体検出
その他扱っていたパッケージ（GitHubから取得して使用）

- Depth Anything2: 単眼深度推定モデル  
  https://github.com/DepthAnything/Depth-Anything-V2
- SAM2: セグメントモデル  
  https://github.com/facebookresearch/sam2
- FoalGAN: 赤外線推定モデル  
  https://github.com/FuyaLuo/FoalGAN
- YOLOv4: YOLO version4  
  https://github.com/kiyoshiiriemon/yolov4_darknet
- X-AnyLabeling: AI付きアノテーションツール  
  https://github.com/CVHub520/X-AnyLabeling
