#!/usr/bin/env python
'''cmd_manager ROS Node'''
import rospy
import os
from std_msgs.msg import String
from std_srvs.srv import *

def callback(cmd):
    '''cmd_manager Callback Function'''
    if cmd.data == 'A':
        print('A')
        os.system('roslaunch robot_bringup start.launch &')
    elif cmd.data == 'M':
        print('M')
        #os.system('roslaunch my_nav gmapping.launch &')

def listener():
    '''cmd_manager Subscriber'''
    rospy.init_node('cmd_manager', anonymous=True)
    print "Ready to service."
    rospy.Subscriber("current_cmd", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
