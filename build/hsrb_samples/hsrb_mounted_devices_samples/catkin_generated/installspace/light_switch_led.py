#!/usr/bin/env python3
# Copyright (C) 2020 Toyota Motor Corporation
"""Light Multi Function Switch LED Sample"""
import sys

import rospy
from std_msgs.msg import Bool

_CONNECTION_TIMEOUT = 10.0


class MultiFunctionSwitch(object):
    """Publish and subscribe multi_function_switch data"""

    def __init__(self):
        self._pre_status = False
        # Create Publisher to control switch led
        switch_led_topic = '/hsrb/switch_led'
        self._switch_led_pub = rospy.Publisher(switch_led_topic,
                                               Bool, queue_size=10)
        # Wait for connection
        while self._switch_led_pub.get_num_connections() == 0:
            rospy.sleep(0.1)

        # Subscribe switch input from multi function switch
        switch_input_topic = '/hsrb/switch_input'
        self._switch_input_sub = rospy.Subscriber(switch_input_topic,
                                                  Bool, self.__switch_input_cb)
        # Wait for connection
        try:
            rospy.wait_for_message(switch_input_topic, Bool,
                                   timeout=_CONNECTION_TIMEOUT)
        except Exception as e:
            rospy.logerr(e)
            sys.exit(1)

    def run(self):
        rospy.spin()

    def __switch_input_cb(self, data):
        # Light led on while multi function switch is pushed
        if data.data != self._pre_status:
            self._switch_led_pub.publish(data.data)
            self._pre_status = data.data


def main():
    multi_function_switch = MultiFunctionSwitch()
    rospy.loginfo('Please push multi function switch')
    multi_function_switch.run()


if __name__ == '__main__':
    rospy.init_node('hsrb_light_switch_led')
    main()
