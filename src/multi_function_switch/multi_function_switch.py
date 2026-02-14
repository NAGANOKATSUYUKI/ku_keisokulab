#!/usr/bin/env python3

import sys
import rospy
import time
from sound_play.libsoundplay import SoundClient
from std_msgs.msg import Bool

class MultiFunctionSwitch(object):
    def __init__(self):
        self.status = False
        
        # Create Publisher to control switch led
        switch_led_topic = '/hsrb/switch_led'
        self._switch_led_pub = rospy.Publisher(switch_led_topic, Bool, queue_size=10)
        
        # Wait for connection
        while self._switch_led_pub.get_num_connections() == 0:
            rospy.sleep(0.1)

        # Subscribe switch input from multi function switch
        switch_input_topic = '/hsrb/switch_input'
        self._switch_input_sub = rospy.Subscriber(switch_input_topic, Bool, self.switch_callback)
       
        # Wait for connection
        try:
            rospy.wait_for_message(switch_input_topic, Bool, timeout=10)
        
        except Exception as e:
            rospy.logerr(e)
            sys.exit(1)
        
        # サウンドクライアントを初期化
        self.sound_client = SoundClient()

        # 再生する音声ファイルのパス(.wav推奨)
        self.audio_file = "/home/keisoku/catkin_ws/src/multi_function_switch/sound_dir/ヴヴヴ復活プチュン.wav"


    def switch_callback(self, data):
        # Light led on while multi function switch is pushed
        if data.data != self.status:
            self._switch_led_pub.publish(data.data)
            self.status = data.data

            if self.status:  # スイッチが押されたとき
                self.sound_client.playWave(self.audio_file)  # 音声ファイルを再生
                rospy.loginfo('Switch pressed, sound played!')  # ログメッセージを表示


    def wait_until_pushed(self):
        # ここでは特にループは不要、コールバックで処理が行われる
        rospy.loginfo('Waiting for switch press...')


if __name__ == '__main__':
    rospy.init_node('hsrb_light_switch_led')
    multi_function_switch = MultiFunctionSwitch()
    rospy.loginfo('Please push multi function switch')

    rospy.spin()  # メインスレッドを継続的に待機



