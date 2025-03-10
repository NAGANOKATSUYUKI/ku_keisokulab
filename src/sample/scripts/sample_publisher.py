#!/usr/bin/env python3
# license removed for brevity

import rospy
# 生成したメッセージのヘッダファイル
from sample.msg import sample_message
from std_msgs.msg import Bool


def publisher():
    # 初期化し、ノードの名前を"sample_c_publisher"とする
    rospy.init_node('sample_py_publisher', anonymous=True)
    # sample_messageというメッセージを"sample_topic"というトピックに送信する
    pub = rospy.Publisher('sample_topic', sample_message, queue_size=10)
    # 1秒間に2回データを送信する
    rate = rospy.Rate(2)

    count = 0
    while not rospy.is_shutdown():
        str = "hello world"
        rospy.loginfo("message = %s, count = %d" %(str, count))

        # 送信するメッセージの作成
        msg = sample_message()
        msg.message = str
        msg.count = count

        # 送信
        pub.publish(msg)
        rate.sleep()
        count += 1

def publisher_Bool():
    rospy.init_node('sample_Bool_pub', anonymous=True)

    pub = rospy.Publisher('sample_Bool', Bool, queue_size=10)

    rate = rospy.Rate(1)

    def_data = None
    count = 0
    while not rospy.is_shutdown():
        rospy.loginfo("Publishing  %s, count = %d" %(str(def_data), count))

        # 送信するメッセージの作成
        msg = Bool()

        def_data = True
        msg.data = def_data

        # 送信
        pub.publish(msg)
        rate.sleep()
        count += 1

if __name__ == '__main__':
    try:
        # publisher()
        publisher_Bool()
    except rospy.ROSInterruptException: pass
