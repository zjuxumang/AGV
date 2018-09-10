#!/usr/bin/env python
# -*- coding: UTF-8 -*-
'''text_send ROS Node'''
# license removed for brevity
import rospy
from std_msgs.msg import String

def callback(data):
    '''text_send Callback Function'''

if __name__ == '__main__':
    try:
        '''text_send Publisher'''
        rospy.Subscriber("current_cmd", String, callback)
        rospy.init_node('text_send', anonymous=True)
        rospy.sleep(0.3)
        
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
