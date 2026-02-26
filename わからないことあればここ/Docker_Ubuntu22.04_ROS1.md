# Ubuntu 22.04 + Docker + ROS1 Noetic 環境構築ガイド

## 📌 目的

Ubuntu 22.04 ホスト環境で Docker を使用し、 **Ubuntu 20.04 + ROS1
Noetic** の開発環境を構築する。

本READMEでは以下を説明します：

-   Dockerのインストール
-   ROS1 Noetic コンテナ起動
-   catkinワークスペース作成・ビルド
-   Pythonノード作成・実行
-   GUI（rviz）使用方法

------------------------------------------------------------------------

# 1. Dockerのインストール（Ubuntu 22.04 ホスト）

## 1-1 古いDockerの削除（任意）

sudo apt-get remove -y docker docker-engine docker.io containerd runc
\|\| true

## 1-2 必要パッケージのインストール

sudo apt-get update sudo apt-get install -y ca-certificates curl gnupg

## 1-3 Docker公式GPGキー追加

sudo install -m 0755 -d /etc/apt/keyrings curl -fsSL
https://download.docker.com/linux/ubuntu/gpg \|\
sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg sudo chmod a+r
/etc/apt/keyrings/docker.gpg

## 1-4 Dockerリポジトリ追加

echo\
"deb \[arch=\$(dpkg --print-architecture)
signed-by=/etc/apt/keyrings/docker.gpg\]\
https://download.docker.com/linux/ubuntu\
\$(. /etc/os-release && echo \$VERSION_CODENAME) stable" \|\
sudo tee /etc/apt/sources.list.d/docker.list \> /dev/null

## 1-5 Dockerインストール

sudo apt-get update sudo apt-get install -y docker-ce docker-ce-cli
containerd.io\
docker-buildx-plugin docker-compose-plugin

## 1-6 sudo無しで使えるようにする（任意）

sudo usermod -aG docker \$USER newgrp docker

## 1-7 動作確認

docker run --rm hello-world

------------------------------------------------------------------------

# 2. ROS1 Noetic コンテナ起動

## 2-1 ワークスペース作成（ホスト側）

mkdir -p \~/catkin_ws/src

## 2-2 コンテナ起動

docker run -it --rm\
--name ros1_noetic\
-v \~/catkin_ws:/root/catkin_ws\
osrf/ros:noetic-desktop-full\
bash

------------------------------------------------------------------------

# 3. catkin ワークスペース構築（コンテナ内）

source /opt/ros/noetic/setup.bash cd /root/catkin_ws catkin_make source
devel/setup.bash

------------------------------------------------------------------------

# 4. Pythonノード作成・実行

## 4-1 パッケージ作成

cd /root/catkin_ws/src catkin_create_pkg my_pkg rospy std_msgs

## 4-2 スクリプト作成

mkdir -p my_pkg/scripts nano my_pkg/scripts/talker.py

### talker.py

#!/usr/bin/env python3 import rospy from std_msgs.msg import String

def main(): rospy.init_node("talker") pub = rospy.Publisher("chatter",
String, queue_size=10) rate = rospy.Rate(10)

    i = 0
    while not rospy.is_shutdown():
        msg = String(data=f"hello {i}")
        pub.publish(msg)
        rospy.loginfo(msg.data)
        i += 1
        rate.sleep()

if **name** == "**main**": main()

## 4-3 実行権限

chmod +x my_pkg/scripts/talker.py

## 4-4 ビルド

cd /root/catkin_ws catkin_make source devel/setup.bash

## 4-5 実行

端末A： roscore

端末B： docker exec -it ros1_noetic bash source
/opt/ros/noetic/setup.bash source /root/catkin_ws/devel/setup.bash
rosrun my_pkg talker.py

確認： rostopic echo /chatter

------------------------------------------------------------------------

# 5. GUI（rviz）使用方法

## ホスト側

xhost +local:docker

## GUIコンテナ起動

docker run -it --rm\
--name ros1_noetic_gui\
--net=host\
-e DISPLAY=\$DISPLAY\
-e QT_X11_NO_MITSHM=1\
-v /tmp/.X11-unix:/tmp/.X11-unix:rw\
-v \~/catkin_ws:/root/catkin_ws\
osrf/ros:noetic-desktop-full\
bash

コンテナ内： source /opt/ros/noetic/setup.bash rviz

------------------------------------------------------------------------

# 6. ポイントまとめ

-   ROS NoeticはUbuntu20.04世代
-   Dockerで22.04上でも利用可能
-   ワークスペースはホスト共有推奨
-   GUI利用はX11設定必要

------------------------------------------------------------------------

# 7. 推奨運用

-   Dockerは --rm でクリーン運用
-   データは必ずボリューム共有
-   研究用途ならDockerが最も安全
