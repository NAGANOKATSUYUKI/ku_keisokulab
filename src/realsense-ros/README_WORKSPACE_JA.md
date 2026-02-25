# realsense-ros Guide

`realsense-ros` は Intel RealSense カメラを ROS で扱うための公式ラッパーパッケージです。

## Purpose
- Color / Depth / Infra / IMU をROSトピックとして配信
- depth-color整列（`align_depth`）
- 点群生成（`filters:=pointcloud`）
- TFやカメラ情報の配信

## Main Packages
- `realsense2_camera`: ドライバ本体
- `realsense2_description`: URDF/3Dモデル

## Quick Start
```bash
roslaunch realsense2_camera rs_camera.launch
```

点群を使う場合:
```bash
roslaunch realsense2_camera rs_camera.launch filters:=pointcloud
```

## Common Topics
- `/camera/color/image_raw`
- `/camera/depth/image_rect_raw`
- `/camera/aligned_depth_to_color/image_raw`
- `/camera/infra/image_raw`（機種により `infra1/infra2`）

## Important Parameters
- `align_depth`: depthをcolorへ整列
- `filters`: `pointcloud, spatial, temporal, hole_filling` など
- `enable_*`: 各ストリームON/OFF
- `<stream>_width`, `<stream>_height`, `<stream>_fps`: 解像度/FPS
- `publish_tf`, `tf_publish_rate`: TF配信設定

## Operational Tips
- 実際のトピックは機種依存なので `rostopic list` で確認
- 複数台構成では `tf_prefix` / frame_id を明示設定
