#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""custom メッセージと Bool メッセージを publish する。"""

import rospy
from sample.msg import sample_message
from std_msgs.msg import Bool

# ===== 設定（最初に編集する場所）=====
CUSTOM_NODE_NAME = "sample_py_publisher"
CUSTOM_TOPIC = "sample_topic"
CUSTOM_RATE_HZ = 2
CUSTOM_TEXT = "hello world"

BOOL_NODE_NAME = "sample_Bool_pub"
BOOL_TOPIC = "sample_Bool"
BOOL_RATE_HZ = 1
BOOL_DEFAULT_DATA = True

def publish_custom_message():
    """`sample_message` を定期 publish する。"""
    rospy.init_node(CUSTOM_NODE_NAME, anonymous=True)
    pub = rospy.Publisher(CUSTOM_TOPIC, sample_message, queue_size=10)
    rate = rospy.Rate(CUSTOM_RATE_HZ)

    count = 0
    while not rospy.is_shutdown():
        rospy.loginfo("message = %s, count = %d", CUSTOM_TEXT, count)

        msg = sample_message()
        msg.message = CUSTOM_TEXT
        msg.count = count

        pub.publish(msg)
        rate.sleep()
        count += 1

def publish_bool_message():
    """`Bool` を定期 publish する。"""
    rospy.init_node(BOOL_NODE_NAME, anonymous=True)
    pub = rospy.Publisher(BOOL_TOPIC, Bool, queue_size=10)
    rate = rospy.Rate(BOOL_RATE_HZ)

    count = 0
    while not rospy.is_shutdown():
        rospy.loginfo("Publishing %s, count = %d", str(BOOL_DEFAULT_DATA), count)
        msg = Bool()
        msg.data = BOOL_DEFAULT_DATA
        pub.publish(msg)
        rate.sleep()
        count += 1

def main():
    try:
        # publish_custom_message()
        publish_bool_message()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()
