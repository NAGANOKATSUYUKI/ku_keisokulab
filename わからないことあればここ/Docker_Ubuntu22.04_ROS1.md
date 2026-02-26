# Ubuntu 22.04 + Docker + ROS1 Noetic セットアップ README

Ubuntu 22.04 ホスト上で Docker を使い、`Ubuntu 20.04 + ROS1 Noetic` 開発環境を構築する手順です。

## 目次

- [目的](#目的)
- [前提](#前提)
- [1. Docker のインストール（ホスト）](#1-docker-のインストールホスト)
- [2. ROS1 Noetic コンテナ起動](#2-ros1-noetic-コンテナ起動)
- [3. catkin ワークスペース構築（コンテナ内）](#3-catkin-ワークスペース構築コンテナ内)
- [4. Python ノード作成・実行](#4-python-ノード作成実行)
- [5. GUI（rviz）を使う](#5-guirvizを使う)
- [6. ポイントまとめ](#6-ポイントまとめ)
- [7. 推奨運用](#7-推奨運用)

## 目的

この README で以下を行います。

- Docker のインストール
- ROS1 Noetic コンテナの起動
- catkin ワークスペース作成とビルド
- Python ノードの作成と実行
- GUI（rviz）の起動

## 前提

- ホスト OS: Ubuntu 22.04
- ネットワーク接続があること
- `sudo` 権限があるユーザーで実行すること

## 1. Docker のインストール（ホスト）

### 1-1. 古い Docker を削除（任意）

```bash
sudo apt-get remove -y docker docker-engine docker.io containerd runc || true
```

### 1-2. 必要パッケージをインストール

```bash
sudo apt-get update
sudo apt-get install -y ca-certificates curl gnupg
```

### 1-3. Docker 公式 GPG キーを追加

```bash
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg
```

### 1-4. Docker リポジトリを追加

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] \
https://download.docker.com/linux/ubuntu \
$(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
```

### 1-5. Docker をインストール

```bash
sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

### 1-6. `sudo` なしで Docker を使う（任意）

```bash
sudo usermod -aG docker "$USER"
newgrp docker
```

### 1-7. 動作確認

```bash
docker run --rm hello-world
```

## 2. ROS1 Noetic コンテナ起動

### 2-1. ワークスペース作成（ホスト）

```bash
mkdir -p ~/catkin_ws/src
```

### 2-2. コンテナ起動

```bash
docker run -it --rm \
  --name ros1_noetic \
  -v ~/catkin_ws:/root/catkin_ws \
  osrf/ros:noetic-desktop-full \
  bash
```

## 3. catkin ワークスペース構築（コンテナ内）

```bash
source /opt/ros/noetic/setup.bash
cd /root/catkin_ws
catkin_make
source devel/setup.bash
```

## 4. Python ノード作成・実行

### 4-1. パッケージ作成

```bash
cd /root/catkin_ws/src
catkin_create_pkg my_pkg rospy std_msgs
```

### 4-2. スクリプト作成

```bash
mkdir -p my_pkg/scripts
nano my_pkg/scripts/talker.py
```

`my_pkg/scripts/talker.py`:

```python
#!/usr/bin/env python3
import rospy
from std_msgs.msg import String


def main():
    rospy.init_node("talker")
    pub = rospy.Publisher("chatter", String, queue_size=10)
    rate = rospy.Rate(10)

    i = 0
    while not rospy.is_shutdown():
        msg = String(data=f"hello {i}")
        pub.publish(msg)
        rospy.loginfo(msg.data)
        i += 1
        rate.sleep()


if __name__ == "__main__":
    main()
```

### 4-3. 実行権限付与

```bash
chmod +x my_pkg/scripts/talker.py
```

### 4-4. ビルド

```bash
cd /root/catkin_ws
catkin_make
source devel/setup.bash
```

### 4-5. 実行

端末 A（コンテナ内）:

```bash
roscore
```

端末 B（ホストから別シェル）:

```bash
docker exec -it ros1_noetic bash
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash
rosrun my_pkg talker.py
```

確認:

```bash
rostopic echo /chatter
```

## 5. GUI（rviz）を使う

### 5-1. ホスト側で X11 許可

```bash
xhost +local:docker
```

### 5-2. GUI コンテナ起動

```bash
docker run -it --rm \
  --name ros1_noetic_gui \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v ~/catkin_ws:/root/catkin_ws \
  osrf/ros:noetic-desktop-full \
  bash
```

コンテナ内で実行:

```bash
source /opt/ros/noetic/setup.bash
rviz
```

## 6. ポイントまとめ

- ROS Noetic は Ubuntu 20.04 世代
- Docker を使えば Ubuntu 22.04 上でも利用可能
- ワークスペースはホスト共有（ボリュームマウント）推奨
- GUI 利用時は X11 設定が必要

## 7. 推奨運用

- `--rm` でコンテナをクリーン運用する
- 重要データは必ずボリューム共有する
- 研究・検証用途ではホストを汚さない Docker 運用が安全
