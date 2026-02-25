# ROS（Robot Operating System）入門 README

---

## 📚 目次
- [はじめに](#はじめに)
- [ROSとは](#rosとは)
- [ROS初心者ロードマップ](#ros初心者ロードマップ)
- [ROS1実践チュートリアル（研究向け）](#ros1実践チュートリアル研究向け)
- [ROSノードのサンプルコード](#rosノードのサンプルコード)
- [ROS1→ROS2移行方法](#ros1ros2移行方法)
- [参考Tips](#参考tips)

---

## はじめに
このREADMEは、Ubuntu環境でROS（特にROS1）をこれから触る人向けに作成した入門資料です。ロボット開発・研究用途を想定し、できるだけ分かりやすく整理しています。

---

## ROSとは
ROS（Robot Operating System）はロボットソフトウェア開発のためのミドルウェアです。

### 主な役割
- センサデータ取得（カメラ・LiDARなど）
- ロボット制御（アーム・移動ロボット）
- ノード間通信
- 可視化・シミュレーション

### 基本概念
#### ノード
機能ごとに分かれたプログラム単位。

例：
- カメラ取得ノード
- 画像認識ノード
- ロボット制御ノード

#### トピック通信
ノード間のデータ共有方式（Publish / Subscribe）。

## ROS1→ROS2移行方法

### 主な違い
| 項目 | ROS1 | ROS2 |
|------|------|------|
| 通信 | 独自通信 | DDS |
| マスター | 必要 | 不要 |
| セキュリティ | なし | あり |
| リアルタイム | 弱い | 強い |

#### roscore
ROS通信を管理するマスター。

```bash
roscore
```

### なぜ `roscore` が必要なのか
ROS1ではノード同士が通信する際に、**ROSマスター（roscore）が通信の仲介役**になります。

具体的には：
- ノードの登録管理
- トピックやサービスの接続情報管理
- ノード同士の通信先の解決

つまり、`roscore` を起動しないとノード同士が「どこに通信すればいいか分からない」状態になり、通信できません。

---

### ROS2で不要な理由
ROS2では中央管理サーバを置かず、**DDS（Data Distribution Service）という分散通信技術**を採用しています。

そのため：
- 自動でノードを検出
- Peer-to-Peer通信
- マスター不要

これによりマルチロボットや分散システムでも安定して動作する設計になっています。

---

## ROS初心者ロードマップ

### Step1：Linux / Ubuntu基礎
- ターミナル操作
- ファイル構造理解
- パッケージ管理

### Step2：ROS基礎理解
- ノード
- トピック通信
- パッケージ構成

### Step3：実際に動かす
- turtlesim
- RViz
- Gazebo

### Step4：研究用途レベル
- センサ統合
- 画像認識連携
- ロボット制御

### Step5：応用
- 自律ロボット
- SLAM
- AI連携

---

## ROS1実践チュートリアル（研究向け）

### ① ワークスペース作成
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
```
※自作以外のパッケージ(darknet_ros,realsense-rosなど)も同時にビルドすると失敗する場合あり
　この場合,パッケージを指定して一つずつビルドすると解決しやすい．

環境読み込み：
```bash
source devel/setup.bash
```

---

### ② パッケージ作成
```bash
cd src
catkin_create_pkg my_pkg rospy std_msgs
```

---

### ③ ノード作成
PythonまたはC++で作成可能。

---

### ④ 実行
```bash
roscore
rosrun my_pkg sample_node.py
```

---

## ROSノードのサンプルコード

### Python例（Publisher）
```python
#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

rospy.init_node('sample_node')
pub = rospy.Publisher('chatter', String, queue_size=10)
rate = rospy.Rate(1)

while not rospy.is_shutdown():
    pub.publish('Hello ROS')
    rate.sleep()
```

実行権限付与：
```bash
chmod +x sample_node.py
```
---

### 移行の流れ

#### ① ROS2環境構築
```bash
sudo apt install ros-humble-desktop
```

#### ② ビルドツール変更
ROS1：
```bash
catkin_make
```

ROS2：
```bash
colcon build
```

---

#### ③ コマンド変更
| ROS1 | ROS2 |
|------|------|
| rosrun | ros2 run |
| roslaunch | ros2 launch |
| rostopic | ros2 topic |

---

#### ④ コード修正
主に以下を変更：
- ノード初期化方法
- QoS設定
- 通信ライブラリ

---

## 参考Tips

### よくあるトラブル
- 環境読み込み忘れ → `source devel/setup.bash`
- roscore未起動
- パーミッション不足

### 学習のコツ
- 小さいノードから作る
- 可視化ツールを活用
- ログ確認を習慣化

### その他
- ubuntu VScodeインストール方法
    https://invisiblepotato.com/ubuntu07/
- rosとは
    https://robokaru.jp/fundamental-knowledge/ros/

- ワークスペース作成流れ
    http://forestofazumino.web.fc2.com/ros/ros_simple_program.html

---

## 最後に
ROSは最初は難しく感じますが、ノード・通信・パッケージの3点を理解すると一気に扱いやすくなります。
研究やロボット開発にぜひ活用してください。

---
