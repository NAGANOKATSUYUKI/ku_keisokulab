#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""custom メッセージと Bool メッセージを subscribe する。"""

import rospy
from sample.msg import sample_message
from std_msgs.msg import Bool

# ===== 設定（最初に編集する場所）=====
CUSTOM_NODE_NAME = "sample_py_subscriber"
CUSTOM_TOPIC = "sample_topic"

BOOL_NODE_NAME = "sample_Bool_sub"
BOOL_TOPIC = "sample_Bool"

def callback_custom(msg):
    """sample_message 受信時のコールバック。"""
    rospy.loginfo("I heard: message = [%s], count = [%d]", msg.message, msg.count)

def subscribe_custom_message():
    """sample_message を subscribe し続ける。"""
    rospy.init_node(CUSTOM_NODE_NAME, anonymous=True)
    rospy.Subscriber(CUSTOM_TOPIC, sample_message, callback_custom)
    rospy.spin()

def callback_bool(msg):
    """Bool 受信時のコールバック。"""
    rospy.loginfo("Received: %s", msg.data)

def subscribe_bool_message():
    """Bool を subscribe し続ける。"""
    rospy.init_node(BOOL_NODE_NAME, anonymous=True)
    rospy.Subscriber(BOOL_TOPIC, Bool, callback_bool)
    rospy.spin()

def main():
    # subscribe_custom_message()
    subscribe_bool_message()

if __name__ == "__main__":
    main()
