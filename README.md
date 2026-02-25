# ku-keisoku

知識の遺産・ダウンロードOK
## 赤外線画像の疑似カラー化とデータ作成手順 
- [疑似カラーREADME.md](わからないことあればここ/疑似カラーREADME.md)
- 暗空間内でのロボットによる物体把持動画:
![デモ](movie_image/ボトル把持.gif)
[物体把持動画(mp4)](movie_image/ボトル把持.mp4)

# パッケージ説明

このワークスペースは `src/` 配下に複数パッケージを持つ ROS1(catkin) 構成です。

## src 直下ディレクトリ

| ディレクトリ | 概要 | リンク |
|---|---|---|
| `sample` | 画像処理・ROS入出力・カスタムメッセージのサンプル集 | [src/sample](src/sample) / [README](src/sample/README.md) |
| `pix2pix` | 赤外線画像の疑似カラーとロボット動作制御に必要なスクリプト | [src/pix2pix](src/pix2pix) / [README](src/pix2pix/README.md) |
| `multi_function_switch` | HSRのスイッチ連動・SE再生の実験用スクリプト | [src/multi_function_switch](src/multi_function_switch) / [README](src/multi_function_switch/README.md) |
| `darknet_ros` | YOLO(Darknet)連携一式（複数ROSパッケージを内包） | [src/darknet_ros](src/darknet_ros) / [README](src/darknet_ros/README_WORKSPACE_JA.md) |
| `realsense-ros` | Intel RealSense公式ROSラッパ（複数ROSパッケージを内包） | [src/realsense-ros](src/realsense-ros) / [README](src/realsense-ros/README_WORKSPACE_JA.md) |

### ROSパッケージ一覧（`src/` 内）

| パッケージ名 | 概要 | リンク |
|---|---|---|
| `sample` | 画像処理用ROSパッケージ（message定義、Python/C++ノード） | [src/sample](src/sample) |
| `pix2pix` | RealSense画像の前処理、3D座標推定、TF配信、把持連携スクリプト | [src/pix2pix](src/pix2pix) |
| `darknet_ros` | YOLO物体検出ノード本体 | [src/darknet_ros/darknet_ros](src/darknet_ros/darknet_ros) |
| `darknet_ros_msgs` | YOLO検出結果用メッセージ定義 | [src/darknet_ros/darknet_ros_msgs](src/darknet_ros/darknet_ros_msgs) |
| `realsense2_camera` | RealSenseカメラドライバ本体 | [src/realsense-ros/realsense2_camera](src/realsense-ros/realsense2_camera) |
| `realsense2_description` | RealSenseカメラURDF/モデル定義 | [src/realsense-ros/realsense2_description](src/realsense-ros/realsense2_description) |

## その他

- 基本的に ROS1 用で Python/C++ ノードを作成しています。ROS2 で使う場合は、ROS1依存ライブラリ部分の修正が必要です。
- 補足README（`わからないことあればここ`）

| ドキュメント | 概要 | リンク |
|---|---|---|
| ROS_初学者導入編 | ROS1の基礎、ロードマップ、サンプル解説 | [わからないことあればここ/ROS_初学者導入編.md](わからないことあればここ/ROS_初学者導入編.md) |
| git_readme | Gitの初期設定から日常運用フローまで | [わからないことあればここ/git_readme.md](わからないことあればここ/git_readme.md) |
| nvidia_cudnn_readme | NVIDIA/CUDA/cuDNNの導入と確認手順 | [わからないことあればここ/nvidia_cudnn_readme.md](わからないことあればここ/nvidia_cudnn_readme.md) |
| HSR コントローラ操作 | DUALSHOCK3のペアリングと操縦手順 | [わからないことあればここ/HSR編/コントローラ操作_readme.md](わからないことあればここ/HSR編/コントローラ操作_readme.md) |
| hsr_joy_controllerコマンド変更 | 有線/無線切替時の設定変更メモ | [わからないことあればここ/HSR編/hsr_joy_controllerコマンド変更](わからないことあればここ/HSR編/hsr_joy_controllerコマンド変更) |

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
