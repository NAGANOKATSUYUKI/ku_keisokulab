#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rospy
from sample.msg import sample_message
from std_msgs.msg import Bool


def callback(msg):
    # 受信したデータを出力する
    rospy.loginfo("I heard: message = [%s], count = [%d]" % (msg.message, msg.count))

def subscriber():
    # 初期化し、ノードの名前を"sample_py_subscriber"とする
    rospy.init_node('sample_py_subscriber', anonymous=True)
    # "sample_topic"というトピックからsample_messageというメッセージを受信する
    rospy.Subscriber('sample_topic', sample_message, callback)

    rospy.spin()



def callback_bool(msg):
    # 受信したデータを出力する
    rospy.loginfo("Received: {}".format(msg.data))

def subscriber_Bool():
    # 初期化し、ノードの名前を"sample_py_subscriber"とする
    rospy.init_node('sample_Bool_sub', anonymous=True)
    # "sample_topic"というトピックからsample_messageというメッセージを受信する
    rospy.Subscriber('sample_Bool', Bool, callback_bool)

    rospy.spin()

if __name__ == '__main__':
    # subscriber()
    subscriber_Bool()
