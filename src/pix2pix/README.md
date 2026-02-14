# pix2pix Directory Guide

`src/pix2pix` は、RealSense入力を使った depth/infra/color 前処理、3D座標推定、TF配信、把持連携のためのROSスクリプト群です。

## What This Directory Contains
- ROSパッケージ定義: `CMakeLists.txt`, `package.xml`
- Launch: `realsense_tf.launch`
- RViz設定: `realsense_to_object.rviz`
- カメラ設定JSON: `l515_camera_0619.json`
- 処理スクリプト: `scripts/*.py`

## launch / rviz
- `realsense_tf.launch`
  - `head_rgbd_sensor_link` と `camera_link` の静的TFを配信。
  - ロボット座標系とカメラ座標系を固定関係で接続する用途。
- `realsense_to_object.rviz`
  - カメラ画像、TF、対象フレーム表示用のRVizプロファイル。

## Scripts Overview
- depthマスク系: `color_depth_filtering.py`, `infra_depth_filtering.py`
- 座標推定/TF: `point_topic2.py`, `tf_point2.py`, `topic_tf_point.py`
- 前処理: `edges_realtime.py`, `infra2rgb.py`
- 保存/データ化: `save_image.py`, `save_click_image.py`, `save_color_infra.py`, `save_filterframe.py`
- ファイル操作: `rename.py`, `resize_image.py`, `csv_check.py`
- ロボット把持操作: `grasping_key.py`

## Typical Pipeline
1. `realsense_tf.launch` で静的TFを配信
2. `*_depth_filtering.py` で必要な画像を前処理
3. `point_topic2.py` / `topic_tf_point.py` で3D座標化
4. `tf_point2.py` で `target_frame` をTF配信
5. `grasping_key.py` で把持動作へ接続

## Notes
- トピック名とパスは先頭定数で変更可能です。
- HSR実機依存のスクリプトは、対応コントローラが起動済みである必要があります。
