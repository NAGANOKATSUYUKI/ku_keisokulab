# multi_function_switch Guide

`multi_function_switch` は HSRB のマルチファンクションスイッチ連動と音再生テスト用のディレクトリです。

## Purpose
- 実機スイッチ押下に応じてLED制御と音再生
- キーボード入力で音ロジック単体テスト

## Main Files
- `multi_function_switch.py`
  - `/hsrb/switch_input` を購読
  - `/hsrb/switch_led` を制御
  - 押下時に `sound_play` で `.wav` を再生

- `sound_switch.py`
  - `pygame` + `pynput` でEnter入力を監視
  - 確率分岐で複数SEを再生（ローカルテスト用途）

- `sound_dir/*.wav`
  - 再生音源ファイル

## Dependencies
- ROS側: `rospy`, `std_msgs/Bool`, `sound_play`
- ローカルテスト側: `pygame`, `pynput`

## Notes
- 音声ファイルパスは絶対パス前提なので、環境差分に注意。
- ファイル名不一致があると再生失敗するため、`sound_dir` 実ファイル名を確認。
