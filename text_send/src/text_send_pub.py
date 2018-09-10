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
    rospy.sleep(1)
    hello_str = "多功能社区服务车启动成功"
    rospy.loginfo(hello_str)
    pub.publish(hello_str)


if __name__ == '__main__':
    try:
        talker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
