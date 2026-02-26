# sample Directory Guide

`src/sample` は、画像前処理・データセット作成・ROS入出力のサンプルスクリプトをまとめたディレクトリです。

## What This Directory Contains
- ROSパッケージ定義: `CMakeLists.txt`, `package.xml`
- カスタムメッセージ: `msg/sample_message.msg`
- 画像処理/ROS連携スクリプト: `scripts/*.py`
- C++サンプルノード: `src/*.cpp`

## Scripts Overview
- 画像変換・前処理: `clahe.py`, `edges_image.py`, `edges_realtime.py`, `infra2rgb.py`, `sobel_image.py`
- ファイル操作: `rename.py`, `resize_image.py`, `image_crop.py`, `image_crop2.py`, `image_combi.py`
- 融合/比較: `image2fusion.py`, `click_CheckPixel_image.py`, `csvfile_sort.py`
- 画像配信/保存: `image2movie.py`, `save_image.py`, `save_click_image.py`, `save_color_infra.py`, `save_infra.py`
- 検出関連: `image2yolo.py`, `yolo_image.py`
- ROS通信学習用: `sample_publisher.py`, `sample_subscriber.py`
- 幾何変換: `HomographyProjecterTf.py`

## C++ Nodes
- `src/sample.cpp`: 1Hzで `Hello World!` を出力する最小ノード
- `src/voice.cpp`: `/talk_request` をsubscribeして文字列を表示

## Typical Use
1. `scripts/` の先頭設定値（パス・トピック）を編集
2. 実行したいスクリプトを単体実行
3. 保存系は出力先ディレクトリの存在を確認

## Notes
- 多くのスクリプトが絶対パスを使うため、環境ごとに設定変更が必要です。
- `__pycache__` は実行時生成キャッシュで、管理対象外です。
