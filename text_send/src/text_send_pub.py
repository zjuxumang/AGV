#!/usr/bin/env python
# -*- coding: UTF-8 -*-
'''text_send ROS Node'''
# license removed for brevity
import rospy
from std_msgs.msg import String

def callback(data):
    '''text_send Callback Function'''
    if(data.data=='B'):
        pub.publish("进入通知巡游播报模式")
        talker()
    elif data.data=='\x09':
        pub.publish("退出客户端")
    elif data.data=='\x08':
        pub.publish("客户端启动成功")

def talker():
    hello_str = "你好"
    rospy.loginfo(hello_str)
    pub.publish(hello_str)

if __name__ == '__main__':
    try:
        '''text_send Publisher'''
        pub = rospy.Publisher('voice/xf_tts_topic', String, queue_size=10)
        rospy.Subscriber("current_cmd", String, callback)
        rospy.init_node('text_send', anonymous=True)
        rospy.sleep(0.3)
        
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
