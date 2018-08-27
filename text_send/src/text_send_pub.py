#!/usr/bin/env python
# -*- coding: UTF-8 -*-
'''text_send ROS Node'''
# license removed for brevity
import rospy
from std_msgs.msg import String

def talker():
    '''text_send Publisher'''
    pub = rospy.Publisher('voice/xf_tts_topic', String, queue_size=10)
    rospy.init_node('text_send', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    hello_str = "你好"
    rospy.loginfo(hello_str)
    pub.publish(hello_str)
    rate.sleep()
    pub.publish(hello_str)

if __name__ == '__main__':
    try:
        talker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
